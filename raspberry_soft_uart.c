
#include "raspberry_soft_uart.h"
#include "queue.h"

#include <linux/gpio.h> 
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/version.h>
#include <linux/workqueue.h>

// To get gpiod_set_debounce() working for replacing gpio_set_debounce()
#include <linux/gpio/consumer.h>

struct hrtimer_identifier_data {
    struct hrtimer hr_timer;
    uint8_t current_uart_index;
    int8_t bit_index;
    uint32_t character;
};

static irqreturn_t handle_rx_start(int irq, void *device);
static enum hrtimer_restart handle_tx(struct hrtimer* timer);
static enum hrtimer_restart handle_rx(struct hrtimer* timer);
static void receive_character(const int index, const unsigned char character);

static struct queue* queues_tx = NULL;
static struct tty_struct** current_ttys = NULL;
static struct hrtimer_identifier_data** timer_tx_id_data = NULL;
static struct hrtimer_identifier_data** timer_rx_id_data = NULL;
static ktime_t period;
static ktime_t half_period;
struct gpio_desc** gpio_tx_descs = NULL;
struct gpio_desc** gpio_rx_descs = NULL;
static void (*rx_callback)(unsigned char) = NULL;

static const int uart_indexes[8] = {0, 1, 2, 3, 4, 5, 6, 7};

/* Convert BCM GPIO number to system GPIO number for Raspberry Pi 5 */
static int bcm_to_rpi5_gpio(const int bcm_gpio)
{
    /* On Raspberry Pi 5, BCM GPIO 0-27 are mapped to system GPIO 571-598 */
    if (bcm_gpio >= 0 && bcm_gpio <= 27) {
        return bcm_gpio + 571;
    }
    return -1; /* Invalid GPIO */
}

/* Convert BCM GPIO number to system GPIO number for Raspberry Pi 4 new kernels */
static int bcm_to_rpi4_gpio(const int bcm_gpio)
{
    /* On Raspberry Pi 4 new kernels, BCM GPIO 0-27 are mapped to system GPIO 512-577 */
    if (bcm_gpio >= 0 && bcm_gpio <= 27) {
        return bcm_gpio + 512;
    }
    return -1; /* Invalid GPIO */
}

/**
 * Initializes the Raspberry Soft UART infrastructure.
 * This must be called during the module initialization.
 * The GPIO pin used as TX is configured as output.
 * The GPIO pin used as RX is configured as input.
 * @param gpio_tx GPIO pins used as TXs
 * @param gpio_rx GPIO pins used as RXs
 * @return 1 if the initialization is successful. 0 otherwise.
 */
int raspberry_soft_uart_init(const unsigned _gpio_tx[], const unsigned _gpio_rx[])
{
    printk(KERN_INFO "soft_uart: calling raspberry_soft_uart_init.\n");
    bool success = true;

    // Initializes mem.
    if (!queues_tx)
        queues_tx = (struct queue*) kmalloc(sizeof(struct queue) * N_PORTS, GFP_KERNEL);
    if (!gpio_tx_descs)
        gpio_tx_descs = (struct gpio_desc**) kmalloc(sizeof(struct gpio_desc*) * N_PORTS, GFP_KERNEL);
    if (!gpio_rx_descs)
        gpio_rx_descs = (struct gpio_desc**) kmalloc(sizeof(struct gpio_desc*) * N_PORTS, GFP_KERNEL);
    if (!timer_tx_id_data)
        timer_tx_id_data = (struct hrtimer_identifier_data**) kmalloc(sizeof(struct hrtimer_identifier_data*) * N_PORTS, GFP_KERNEL);
    if (!timer_rx_id_data)
        timer_rx_id_data = (struct hrtimer_identifier_data**) kmalloc(sizeof(struct hrtimer_identifier_data*) * N_PORTS, GFP_KERNEL);
    if (!current_ttys)
        current_ttys = (struct tty_struct**) kmalloc(sizeof(struct tty_struct*) * N_PORTS, GFP_KERNEL);

    // Myles: for debugging gpio_request
    int request_result = 0;

    // Set up uart one by one
    int i;
    for (i = 0; i < N_PORTS; ++i)
    {
        // Give NULLs
        current_ttys[i] = NULL;

        // Initializes the TX timer.
        timer_tx_id_data[i] = (struct hrtimer_identifier_data*) kmalloc(sizeof(struct hrtimer_identifier_data), GFP_KERNEL);
        hrtimer_init(&(timer_tx_id_data[i]->hr_timer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        (timer_tx_id_data[i]->hr_timer).function = &handle_tx;
        timer_tx_id_data[i]->current_uart_index = i;
        timer_tx_id_data[i]->bit_index = -1;
        timer_tx_id_data[i]->character = 0;
        
        // Initializes the RX timer.
        timer_rx_id_data[i] = (struct hrtimer_identifier_data*) kmalloc(sizeof(struct hrtimer_identifier_data), GFP_KERNEL);
        hrtimer_init(&(timer_rx_id_data[i]->hr_timer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        (timer_rx_id_data[i]->hr_timer).function = &handle_rx;
        timer_rx_id_data[i]->current_uart_index = i;
        timer_rx_id_data[i]->bit_index = -1;
        timer_rx_id_data[i]->character = 0;

        // Set up the GPIO for TX
        if (gpio_request(bcm_to_rpi4_gpio(_gpio_tx[i]), "soft_uart_tx") != 0)
        {
            printk(KERN_ALERT "soft_uart: Failed to request tx GPIO %d.\n", bcm_to_rpi4_gpio(_gpio_tx[i]));
            success = false;
            goto Done_init;
        }
        gpio_tx_descs[i] = gpio_to_desc(bcm_to_rpi4_gpio(_gpio_tx[i]));

        // Set up the GPIO for RX
        if (gpio_request(bcm_to_rpi4_gpio(_gpio_rx[i]), "soft_uart_rx") != 0)
        {
            printk(KERN_ALERT "soft_uart: Failed to request rx GPIO %d.\n", bcm_to_rpi4_gpio(_gpio_rx[i]));
            success = false;
            goto Done_init;
        }
        gpio_rx_descs[i] = gpio_to_desc(bcm_to_rpi4_gpio(_gpio_rx[i]));

        // Configures the GPIO for TX
        request_result = gpiod_direction_output(gpio_tx_descs[i], 1);
        success &= request_result == 0;

        // Configures the GPIO for RX
        request_result = gpiod_direction_input(gpio_rx_descs[i]);
        success &= request_result == 0;
        
        // Initializes the interruption.
        request_result = request_irq(
            gpiod_to_irq(gpio_rx_descs[i]),
            handle_rx_start,
            IRQF_TRIGGER_FALLING,
            "soft_uart_irq_handler",
            &(uart_indexes[i]));
        success &= request_result == 0;
        disable_irq(gpiod_to_irq(gpio_rx_descs[i]));
        printk(KERN_INFO "soft_uart: result after requesting gpio_rx irq: %d (%d).\n", gpiod_to_irq(gpio_rx_descs[i]), request_result);
    }
        
Done_init:
    return success;
}

/**
 * Finalizes the Raspberry Soft UART infrastructure.
 */
int raspberry_soft_uart_finalize(const unsigned _gpio_tx[], const unsigned _gpio_rx[])
// int raspberry_soft_uart_finalize(void)
{
    printk(KERN_INFO "soft_uart: finalizing soft uart...\n");

    // free resources
    int i;
    for (i = 0; i < N_PORTS; ++i)
    {
        free_irq(gpiod_to_irq(gpio_rx_descs[i]), &(uart_indexes[i]));

        gpiod_set_value(gpio_tx_descs[i], 0);

        gpio_free(bcm_to_rpi4_gpio(_gpio_tx[i]));
        gpio_free(bcm_to_rpi4_gpio(_gpio_rx[i]));
    }

    // below is for freeing mem

    // free TX queues
    if (queues_tx)
        kfree(queues_tx);

    // free all pin number holders
    if (gpio_tx_descs)
        kfree(gpio_tx_descs);
    if (gpio_rx_descs)
        kfree(gpio_rx_descs);

    // free all hrtimers
    for (i = 0; i < N_PORTS; ++i)
    {
        if (timer_tx_id_data[i])
            kfree(timer_tx_id_data[i]);
        if (timer_rx_id_data[i])
            kfree(timer_rx_id_data[i]);
    }
    if (timer_tx_id_data)
        kfree(timer_tx_id_data);
    if (timer_rx_id_data)
        kfree(timer_rx_id_data);

    // free tty related
    if (current_ttys)
        kfree(current_ttys);

    return 1;
}

/**
 * Opens the Soft UART.
 * @param tty
 * @return 1 if the operation is successful. 0 otherwise.
 */
int raspberry_soft_uart_open(struct tty_struct* tty)
{
    printk(KERN_INFO "soft_uart: opening index: %d, NULL status: %d.\n", tty->index, current_ttys[tty->index] == NULL);

    int success = 0;
    timer_rx_id_data[tty->index]->bit_index = -1;
    if (current_ttys[tty->index] == NULL)
    {
        current_ttys[tty->index] = tty;
        initialize_queue(&(queues_tx[tty->index]));
        success = 1;
        // enable_irq(gpio_to_irq(gpio_rx[tty->index]));
        enable_irq(gpiod_to_irq(gpio_rx_descs[tty->index]));
    }
    return success;
}

/**
 * Closes the Soft UART.
 */
int raspberry_soft_uart_close(struct tty_struct* tty)
{
    printk(KERN_INFO "soft_uart: closing index: %d, NULL status: %d\n", tty->index, current_ttys[tty->index] == NULL);

    disable_irq(gpiod_to_irq(gpio_rx_descs[tty->index]));
    hrtimer_cancel(&(timer_tx_id_data[tty->index]->hr_timer));
    hrtimer_cancel(&(timer_rx_id_data[tty->index]->hr_timer));
    current_ttys[tty->index] = NULL;
    
    return 1;
}

/**
 * Sets the Soft UART baudrate.
 * @param baudrate desired baudrate
 * @return 1 if the operation is successful. 0 otherwise.
 */
int raspberry_soft_uart_set_baudrate(const int index, const int baudrate) 
{
    printk(KERN_INFO "soft_uart: setting index: %d with baudrate: %d\n", index, baudrate);

    period = ktime_set(0, 1000000000/baudrate);
    half_period = ktime_set(0, 1000000000/baudrate/2);
    return 1;
}

/**
 * Adds a given string to the TX queue.
 * @index index of tty
 * @paran string given string
 * @param string_size size of the given string
 * @return The amount of characters successfully added to the queue.
 */
int raspberry_soft_uart_send_string(const int index, const unsigned char* string, const int string_size)
{
    int result = enqueue_string(&(queues_tx[index]), string, string_size);
    
    // Starts the TX timer if it is not already running.
    if (!hrtimer_active(&(timer_tx_id_data[index]->hr_timer)))
    {
        hrtimer_start(&(timer_tx_id_data[index]->hr_timer), period, HRTIMER_MODE_REL);
    }
    
    return result;
}

/*
 * Gets the number of characters that can be added to the TX queue.
 * @return number of characters.
 */
int raspberry_soft_uart_get_tx_queue_room(const int index)
{
    return get_queue_room(&(queues_tx[index]));
}

/*
 * Gets the number of characters in the TX queue.
 * @return number of characters.
 */
int raspberry_soft_uart_get_tx_queue_size(const int index)
{
    return get_queue_size(&(queues_tx[index]));
}

/**
 * Sets the callback function to be called on received character.
 * @param callback the callback function
 */
int raspberry_soft_uart_set_rx_callback(void (*callback)(const unsigned char))
{
	rx_callback = callback;
	return 1;
}

//-----------------------------------------------------------------------------
// Internals
//-----------------------------------------------------------------------------

/**
 * If we are waiting for the RX start bit, then starts the RX timer. Otherwise,
 * does nothing.
 */
static irqreturn_t handle_rx_start(int irq, void *device)
{
    int uart_index = *(int*)device;

    if (timer_rx_id_data[uart_index]->bit_index == -1)
    {
        hrtimer_start(&(timer_rx_id_data[uart_index]->hr_timer), half_period, HRTIMER_MODE_REL);
    }
    return IRQ_HANDLED;
}


/**
 * Dequeues a character from the TX queue and sends it.
 */
static enum hrtimer_restart handle_tx(struct hrtimer* timer)
{
    ktime_t current_time = ktime_get();

    struct hrtimer_identifier_data *tx_id_data = container_of(timer, struct hrtimer_identifier_data, hr_timer);

    enum hrtimer_restart result = HRTIMER_NORESTART;
    bool must_restart_timer = false;
    
    // Start bit.
    if ((tx_id_data->bit_index) == -1)
    {
        if (dequeue_character(&(queues_tx[tx_id_data->current_uart_index]), &(tx_id_data->character)))
        {
            gpiod_set_value(gpio_tx_descs[tx_id_data->current_uart_index], 0);
            (tx_id_data->bit_index)++;
            must_restart_timer = true;
        }
    }
    
    // Data bits.
    else if ((tx_id_data->bit_index) < 8)
    {
        gpiod_set_value(gpio_tx_descs[tx_id_data->current_uart_index], 1 & ((tx_id_data->character) >> (tx_id_data->bit_index)));
        (tx_id_data->bit_index)++;
        must_restart_timer = true;
    }
    
    // Stop bit.
    else
    {
        gpiod_set_value(gpio_tx_descs[tx_id_data->current_uart_index], 1);

        (tx_id_data->character) = 0;
        (tx_id_data->bit_index) = -1;
        must_restart_timer = get_queue_size(&(queues_tx[tx_id_data->current_uart_index])) > 0;
    }
    
    // Restarts the TX timer.
    if (must_restart_timer)
    {
        hrtimer_forward(&(tx_id_data->hr_timer), current_time, period);
        result = HRTIMER_RESTART;
    }
    
    return result;
}

/*
 * Receives a character and sends it to the kernel.
 */
static enum hrtimer_restart handle_rx(struct hrtimer* timer)
{
    ktime_t current_time = ktime_get();
    struct hrtimer_identifier_data *rx_id_data = container_of(timer, struct hrtimer_identifier_data, hr_timer);
    int bit_value = gpiod_get_value(gpio_rx_descs[rx_id_data->current_uart_index]);
    enum hrtimer_restart result = HRTIMER_NORESTART;
    bool must_restart_timer = false;

    // printk(KERN_INFO "soft_uart: handle_rx called with current_uart_index: %d and rx_bit_index: %d and bit_value: %d.\n", rx_id_data->current_uart_index, rx_id_data->bit_index, bit_value);
    
    // Start bit.
    if ((rx_id_data->bit_index) == -1)
    {
        // TEMP: handle invalid frame coming in
        if (bit_value == 0)
        {
            (rx_id_data->bit_index) = 0;
            (rx_id_data->character) = 0;
            must_restart_timer = true;
        }
    }
    // Data bits.
    else if ((rx_id_data->bit_index) < 8)
    {
        if (bit_value == 0)
        {
            (rx_id_data->character) &= 0xfeff;
        }
        else
        {
            (rx_id_data->character) |= 0x0100;
        }
        
        (rx_id_data->bit_index)++;
        (rx_id_data->character) >>= 1;
        must_restart_timer = true;
    }
    
    // Stop bit.
    else
    {
        receive_character(rx_id_data->current_uart_index, (rx_id_data->character));
        (rx_id_data->bit_index) = -1;
    }
    
    // Restarts the RX timer.
    if (must_restart_timer)
    {
        hrtimer_forward(&(rx_id_data->hr_timer), current_time, period);
        result = HRTIMER_RESTART;
    }
    
    return result;
}

/**
 * Adds a given (received) character to the RX buffer, which is managed by the kernel,
 * and then flushes (flip) it.
 * @param character given character
 */
void receive_character(const int index, const unsigned char character)
{
    // printk(KERN_INFO "soft_uart: receive_character for index: %d with character: %c\n", index, character);

    if (rx_callback != NULL) {
        (*rx_callback)(character);
    } else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
        if (current_ttys[index] != NULL && current_ttys[index]->port != NULL)
        {
            tty_insert_flip_char(current_ttys[index]->port, character, TTY_NORMAL);
            tty_flip_buffer_push(current_ttys[index]->port);
        }
#else
        if (tty != NULL)
        {
            tty_insert_flip_char(current_ttys[index], character, TTY_NORMAL);
            tty_flip_buffer_push(tty);
        }
#endif
    }
}
