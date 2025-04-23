
#include "raspberry_soft_uart.h"
#include "queue.h"

#include <linux/gpio.h> 
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/version.h>

// To get gpiod_set_debounce() working for replacing gpio_set_debounce()
#include <linux/gpio/consumer.h>

struct hrtimer_identifier_data {
    struct hrtimer hr_timer;
    int current_uart_index;
};

static irqreturn_t handle_rx_start(int irq, void *device);
static enum hrtimer_restart handle_tx(struct hrtimer* timer);
static enum hrtimer_restart handle_rx(struct hrtimer* timer);
static void receive_character(int index, unsigned char character);

static struct queue* queues_tx = NULL;
static struct tty_struct** current_ttys = NULL;
static struct mutex* current_tty_mutexes = NULL;
static struct hrtimer_identifier_data** timer_tx_id_data = NULL;
static struct hrtimer_identifier_data** timer_rx_id_data = NULL;
static ktime_t period;
static ktime_t half_period;
// static int* gpio_tx = NULL;
struct gpio_desc** gpio_tx_descs = NULL;
// static int* gpio_rx = NULL;
struct gpio_desc** gpio_rx_descs = NULL;
static int rx_bit_index = -1;
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

/**
 * Initializes the Raspberry Soft UART infrastructure.
 * This must be called during the module initialization.
 * The GPIO pin used as TX is configured as output.
 * The GPIO pin used as RX is configured as input.
 * @param gpio_tx GPIO pins used as TXs
 * @param gpio_rx GPIO pins used as RXs
 * @return 1 if the initialization is successful. 0 otherwise.
 */
// int raspberry_soft_uart_init(const unsigned _gpio_tx[], const unsigned _gpio_rx[])
int raspberry_soft_uart_init(const char* _gpio_tx[], const char* _gpio_rx[])
{
    printk(KERN_INFO "soft_uart: calling raspberry_soft_uart_init.\n");
    bool success = true;

    // Initializes mem.
    if (!queues_tx)
        queues_tx = (struct queue*) kmalloc(sizeof(struct queue) * N_PORTS, GFP_KERNEL);
    // if (!gpio_tx)
    //     gpio_tx = (int*) kmalloc(sizeof(int) * N_PORTS, GFP_KERNEL);
    // if (!gpio_rx)
    //     gpio_rx = (int*) kmalloc(sizeof(int) * N_PORTS, GFP_KERNEL);
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
    if (!current_tty_mutexes)
        current_tty_mutexes = (struct mutex*) kmalloc(sizeof(struct mutex) * N_PORTS, GFP_KERNEL);

    // Myles: for debugging gpio_request
    int request_result = 0;

    // Set up uart one by one
    int i;
    for (i = 0; i < N_PORTS; ++i)
    {
        // Give NULLs
        current_ttys[i] = NULL;

        // Initializes the mutex
        mutex_init(&(current_tty_mutexes[i]));

        // Initializes the TX timer.
        timer_tx_id_data[i] = (struct hrtimer_identifier_data*) kmalloc(sizeof(struct hrtimer_identifier_data), GFP_KERNEL);
        hrtimer_init(&(timer_tx_id_data[i]->hr_timer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        (timer_tx_id_data[i]->hr_timer).function = &handle_tx;
        // (timer_tx_id_data[i]->hr_timer)._softexpires = timer_tx_id_data[i];
        timer_tx_id_data[i]->current_uart_index = i;
        
        // Initializes the RX timer.
        timer_rx_id_data[i] = (struct hrtimer_identifier_data*) kmalloc(sizeof(struct hrtimer_identifier_data), GFP_KERNEL);
        hrtimer_init(&(timer_rx_id_data[i]->hr_timer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        (timer_rx_id_data[i]->hr_timer).function = &handle_rx;
        // (timer_rx_id_data[i]->hr_timer)._softexpires = timer_rx_id_data[i];
        timer_rx_id_data[i]->current_uart_index = i;

        // gpio_tx[i] = _gpio_tx[i];
        // gpio_rx[i] = _gpio_rx[i];

        // if (gpio_request(_gpio_tx[i], "soft_uart_tx") != 0)
        // {
        //     printk(KERN_ALERT "soft_uart: Failed to request GPIO %d for TX pin %s.\n", _gpio_tx[i], _gpio_tx[i]);
        //     success = false;
        //     goto Done_init;
        // }
        // gpio_tx_descs[i] = gpio_to_desc(_gpio_tx[i]);
        // gpio_tx_descs[i] = gpio_to_desc(bcm_to_rpi5_gpio(_gpio_tx[i]));
        gpio_tx_descs[i] = gpiod_get(NULL, _gpio_tx[i], GPIOD_OUT_LOW);
        if (IS_ERR(gpio_tx_descs[i]))
        {
            printk(KERN_ALERT "soft_uart: Failed to get GPIO descriptor for TX pin %s.\n", _gpio_tx[i]);
            success = false;
            goto Done_init;
        }

        // if (gpio_request(_gpio_rx[i], "soft_uart_rx") != 0)
        // {
        //     printk(KERN_ALERT "soft_uart: Failed to request GPIO %d for RX pin %s.\n", _gpio_rx[i], _gpio_rx[i]);
        //     success = false;
        //     goto Done_init;
        // }
        // gpio_rx_descs[i] = gpio_to_desc(_gpio_rx[i]);
        // gpio_rx_descs[i] = gpio_to_desc(bcm_to_rpi5_gpio(_gpio_rx[i]));
        gpio_rx_descs[i] = gpiod_get(NULL, _gpio_rx[i], GPIOD_IN);
        if (IS_ERR(gpio_rx_descs[i]))
        {
            printk(KERN_ALERT "soft_uart: Failed to get GPIO descriptor for RX pin %s.\n", _gpio_rx[i]);
            success = false;
            goto Done_init;
        }

        // request_result = gpio_request(gpio_tx[i], "soft_uart_tx");
        // success &= request_result == 0;
        // printk(KERN_INFO "soft_uart: result after requesting _gpio_tx: %d: %d (%d).\n", gpio_tx[i], success, request_result);
        // request_result = gpio_direction_output(gpio_tx[i], 1);
        request_result = gpiod_direction_output(gpio_tx_descs[i], 1);
        success &= request_result == 0;
        // printk(KERN_INFO "soft_uart: result after requesting _gpio_tx direction: %d (%d).\n", success, request_result);

        // request_result = gpio_request(gpio_rx[i], "soft_uart_rx");
        // success &= request_result == 0;
        // printk(KERN_INFO "soft_uart: result after requesting gpio_rx: %d: %d (%d).\n", gpio_rx[i], success, request_result);
        // request_result = gpio_direction_input(gpio_rx[i]);
        request_result = gpiod_direction_input(gpio_rx_descs[i]);
        success &= request_result == 0;
        // printk(KERN_INFO "soft_uart: result after requesting gpio_rx direction: %d (%d).\n", success, request_result);
        
        // Initializes the interruption.
        // request_result = request_irq(
        //     gpio_to_irq(gpio_rx[i]),
        //     handle_rx_start,
        //     IRQF_TRIGGER_FALLING,
        //     "soft_uart_irq_handler",
        //     &(uart_indexes[i]));
        // success &= request_result == 0;
        // disable_irq(gpio_to_irq(gpio_rx[i]));
        request_result = request_irq(
            gpiod_to_irq(gpio_rx_descs[i]),
            handle_rx_start,
            IRQF_TRIGGER_FALLING,
            "soft_uart_irq_handler",
            &(uart_indexes[i]));
        success &= request_result == 0;
        disable_irq(gpiod_to_irq(gpio_rx_descs[i]));
        printk(KERN_INFO "soft_uart: result after requesting gpio_rx irq: %d (%d).\n", success, request_result);
    }
        
Done_init:
    return success;
}

/**
 * Finalizes the Raspberry Soft UART infrastructure.
 */
// int raspberry_soft_uart_finalize(const unsigned _gpio_tx[], const unsigned _gpio_rx[])
int raspberry_soft_uart_finalize(void)
{
    printk(KERN_INFO "soft_uart: finalizing soft uart...\n");

    // free resources
    int i;
    for (i = 0; i < N_PORTS; ++i)
    {
        // free_irq(gpio_to_irq(gpio_rx[i]), &(uart_indexes[i]));
        free_irq(gpiod_to_irq(gpio_rx_descs[i]), &(uart_indexes[i]));

        // gpio_set_value(gpio_tx[i], 0);
        gpiod_set_value(gpio_tx_descs[i], 0);

        // gpio_free(_gpio_tx[i]);
        // gpio_free(_gpio_rx[i]);
        gpiod_put(gpio_tx_descs[i]);
        gpiod_put(gpio_rx_descs[i]);
    }

    // below is for freeing mem

    // free TX queues
    if (queues_tx)
        kfree(queues_tx);

    // free all pin number holders
    // if (gpio_tx)
    //     kfree(gpio_tx);
    // if (gpio_rx)
    //     kfree(gpio_rx);
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
    // if (timer_tx)
    //     kfree(timer_tx);
    if (timer_tx_id_data)
        kfree(timer_tx_id_data);
    // if (timer_rx)
    //     kfree(timer_rx);
    if (timer_rx_id_data)
        kfree(timer_rx_id_data);

    // free tty related
    if (current_ttys)
        kfree(current_ttys);
    if (current_tty_mutexes)
        kfree(current_tty_mutexes);

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
    mutex_lock(&(current_tty_mutexes[tty->index]));
    rx_bit_index = -1;
    if (current_ttys[tty->index] == NULL)
    {
        current_ttys[tty->index] = tty;
        initialize_queue(&(queues_tx[tty->index]));
        success = 1;
        // enable_irq(gpio_to_irq(gpio_rx[tty->index]));
        enable_irq(gpiod_to_irq(gpio_rx_descs[tty->index]));
    }
    mutex_unlock(&(current_tty_mutexes[tty->index]));
    return success;
}

/**
 * Closes the Soft UART.
 */
int raspberry_soft_uart_close(struct tty_struct* tty)
{
    printk(KERN_INFO "soft_uart: calling raspberry_soft_uart_close.\n");
    printk(KERN_INFO "soft_uart: closing index: %d, NULL status: %d\n", tty->index, current_ttys[tty->index] == NULL);

    mutex_lock(&(current_tty_mutexes[tty->index]));
    // disable_irq(gpio_to_irq(gpio_rx[tty->index]));
    disable_irq(gpiod_to_irq(gpio_rx_descs[tty->index]));
    // hrtimer_cancel(&(timer_tx[tty->index]));
    hrtimer_cancel(&(timer_tx_id_data[tty->index]->hr_timer));
    // hrtimer_cancel(&(timer_rx[tty->index]));
    hrtimer_cancel(&(timer_rx_id_data[tty->index]->hr_timer));
    current_ttys[tty->index] = NULL;
    mutex_unlock(&(current_tty_mutexes[tty->index]));
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
    gpiod_set_debounce(gpio_rx_descs[index], 1000/baudrate/2);
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
    // printk(KERN_INFO "soft_uart: sending string with size: %d to index: %d\n", string_size, index);

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
int raspberry_soft_uart_set_rx_callback(void (*callback)(unsigned char))
{
    // printk(KERN_INFO "soft_uart: calling raspberry_soft_uart_set_rx_callback.\n");

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
    // printk(KERN_INFO "soft_uart: calling handle_rx_start.\n");

    // if (device == NULL)
    //     printk(KERN_INFO "soft_uart: handle_rx_start has NULL for uart index.\n");

    int uart_index = *(int*)device;

    // printk(KERN_INFO "soft_uart: calling handle_rx_start with uart_index: %d.\n", uart_index);

    if (rx_bit_index == -1)
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
    struct hrtimer_identifier_data *tx_id_data = container_of(timer, struct hrtimer_identifier_data, hr_timer);

    // if (tx_id_data == NULL)
    //     printk(KERN_INFO "soft_uart: handle_tx has NULL for tx_id_data.\n");
    // printk(KERN_INFO "soft_uart: handle_tx for index: %d\n", tx_id_data->current_uart_index);

    ktime_t current_time = ktime_get();
    static unsigned char character = 0;
    static int bit_index = -1;
    enum hrtimer_restart result = HRTIMER_NORESTART;
    bool must_restart_timer = false;
    
    // Start bit.
    if (bit_index == -1)
    {
        if (dequeue_character(&(queues_tx[tx_id_data->current_uart_index]), &character))
        {
        // gpio_set_value(gpio_tx[tx_id_data->current_uart_index], 0);
        gpiod_set_value(gpio_tx_descs[tx_id_data->current_uart_index], 0);
        bit_index++;
        must_restart_timer = true;
        }
    }
    
    // Data bits.
    else if (0 <= bit_index && bit_index < 8)
    {
        // gpio_set_value(gpio_tx[tx_id_data->current_uart_index], 1 & (character >> bit_index));
        gpiod_set_value(gpio_tx_descs[tx_id_data->current_uart_index], 1 & (character >> bit_index));
        bit_index++;
        must_restart_timer = true;
    }
    
    // Stop bit.
    else if (bit_index == 8)
    {
        // gpio_set_value(gpio_tx[tx_id_data->current_uart_index], 1);
        gpiod_set_value(gpio_tx_descs[tx_id_data->current_uart_index], 1);
        character = 0;
        bit_index = -1;
        must_restart_timer = get_queue_size(&(queues_tx[tx_id_data->current_uart_index])) > 0;
    }
    
    // Restarts the TX timer.
    if (must_restart_timer)
    {
        hrtimer_forward(&(timer_tx_id_data[tx_id_data->current_uart_index]->hr_timer), current_time, period);
        result = HRTIMER_RESTART;
    }
    
    return result;
}

/*
 * Receives a character and sends it to the kernel.
 */
static enum hrtimer_restart handle_rx(struct hrtimer* timer)
{

    struct hrtimer_identifier_data *rx_id_data = container_of(timer, struct hrtimer_identifier_data, hr_timer);

    // if (rx_id_data == NULL)
    //     printk(KERN_INFO "soft_uart: handle_rx has NULL for rx_id_data.\n");
    // printk(KERN_INFO "soft_uart: handle_rx for index: %d\n", rx_id_data->current_uart_index);

    ktime_t current_time = ktime_get();
    static unsigned int character = 0;
    // int bit_value = gpio_get_value(gpio_rx[rx_id_data->current_uart_index]);
    int bit_value = gpiod_get_value(gpio_rx_descs[rx_id_data->current_uart_index]);
    enum hrtimer_restart result = HRTIMER_NORESTART;
    bool must_restart_timer = false;
    
    // Start bit.
    if (rx_bit_index == -1)
    {
        rx_bit_index++;
        character = 0;
        must_restart_timer = true;
    }
    
    // Data bits.
    else if (0 <= rx_bit_index && rx_bit_index < 8)
    {
        if (bit_value == 0)
        {
        character &= 0xfeff;
        }
        else
        {
        character |= 0x0100;
        }
        
        rx_bit_index++;
        character >>= 1;
        must_restart_timer = true;
    }
    
    // Stop bit.
    else if (rx_bit_index == 8)
    {
        receive_character(rx_id_data->current_uart_index, character);
        rx_bit_index = -1;
    }
    
    // Restarts the RX timer.
    if (must_restart_timer)
    {
        hrtimer_forward(&(timer_rx_id_data[rx_id_data->current_uart_index]->hr_timer), current_time, period);
        result = HRTIMER_RESTART;
    }
    
    return result;
}

/**
 * Adds a given (received) character to the RX buffer, which is managed by the kernel,
 * and then flushes (flip) it.
 * @param character given character
 */
void receive_character(int index, unsigned char character)
{
    // printk(KERN_INFO "soft_uart: receive_character for index: %d\n", index);

    mutex_lock(&(current_tty_mutexes[index]));
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
    mutex_unlock(&(current_tty_mutexes[index]));
}
