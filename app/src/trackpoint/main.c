/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <kernel.h>
#include <zmk/trackpoint.h>
#include <drivers/gpio.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>

#define TP_CLK_PIN 13 // 8
#define TP_DAT_PIN 14 // 9
#define TP_RST_PIN 15 // 10
#define HIGH 1
#define LOW 0

uint8_t readBit;
uint8_t readResult;
uint8_t readBitMask;

K_MUTEX_DEFINE(readMutex);
K_CONDVAR_DEFINE(readBitRes);

struct DataReport {
    uint8_t state;
    int8_t x;
    int8_t y;
} dataReport;

const struct device *gpiodev;
static struct k_work_delayable initialize_trackpoint;
static struct k_work_delayable some_work;
static struct k_work_delayable poll_trackpoint;

static struct gpio_callback gpio_read_ctx;

K_THREAD_STACK_DEFINE(trackpoint_work_stack_area, 2048);
static struct k_work_q trackpoint_work_q;

struct k_work_q *zmk_trackpoint_work_q() {
    return &trackpoint_work_q;
}

static void read_data_bit(const struct device *gpio,
                       struct gpio_callback *cb, uint32_t pins)
{
    k_mutex_lock(&readMutex, K_FOREVER);
    // If I'm called, it means the clock fell and I may need to read a data bit
    // Ignore (eat) the first bit and last 2 bits (10th, 11th)
    if(readBit > 0 && readBit < 9) {
        if (gpio_pin_get(gpio, TP_DAT_PIN) == HIGH) {
            readResult = readResult | readBitMask;
        }
        readBitMask = readBitMask << 1;
    }
    // If this is the eleventh data bit, unblock the read
    if (readBit == 10) {
        k_condvar_signal(&readBitRes);
    }
    readBit++;
    k_mutex_unlock(&readMutex);
}

void gohi(uint8_t pin) {
    gpio_pin_set_raw(gpiodev, pin, 1);
}

void golo(uint8_t pin) {
    gpio_pin_set_raw(gpiodev, pin, 0);
}

int readPin(uint8_t pin) {
    return gpio_pin_get(gpiodev, pin);
}

void write(uint8_t data) {
    uint8_t i;
    uint8_t parity = 1;

    gohi(TP_DAT_PIN);
    gohi(TP_CLK_PIN);
    k_sleep(K_USEC(300));
    golo(TP_CLK_PIN);
    k_sleep(K_USEC(300));
    golo(TP_DAT_PIN);
    k_sleep(K_USEC(10));
    gohi(TP_CLK_PIN);	// start bit
    /* wait for device to take control of clock */
    while (readPin(TP_CLK_PIN) == HIGH)
        ;	// this loop intentionally left blank
    // clear to send data
    for (i=0; i < 8; i++)
    {
        // wait for clock
        if (data & 0x01)
        {
            gohi(TP_DAT_PIN);
        } else {
            golo(TP_DAT_PIN);
        }
        while (readPin(TP_CLK_PIN) == LOW)
            ;
        while (readPin(TP_CLK_PIN) == HIGH)
            ;
        parity = parity ^ (data & 0x01);
        data = data >> 1;
    }
    // parity bit
    if (parity)
    {
        gohi(TP_DAT_PIN);
    } else {
        golo(TP_DAT_PIN);
    }
    // clock cycle - like ack.
    while (readPin(TP_CLK_PIN) == LOW)
        ;
    while (readPin(TP_CLK_PIN) == HIGH)
        ;
    // stop bit
    gohi(TP_DAT_PIN);
    k_sleep(K_USEC(30));
    while (readPin(TP_CLK_PIN) == HIGH)
        ;
    golo(TP_CLK_PIN); return;
    // mode switch
    while ((readPin(TP_CLK_PIN) == LOW) || (readPin(TP_DAT_PIN) == LOW))
        ;
    // hold up incoming data
    golo(TP_CLK_PIN);
}

uint8_t read(void) {
    k_mutex_lock(&readMutex, K_FOREVER);
    k_sleep(K_MSEC(1)); // Let things settle a bit
    // Cleanup
    readBit = 0;
    readResult = 0;
    readBitMask = 0x01;
    gpio_pin_interrupt_configure(gpiodev, TP_CLK_PIN, GPIO_INT_EDGE_TO_INACTIVE);
    k_sleep(K_USEC(100));
    // Ready to receive
    gohi(TP_CLK_PIN);
    gohi(TP_DAT_PIN);
    // Wait for the read to finish (reads 11 bytes, stores 8)
    k_condvar_wait(&readBitRes, &readMutex, K_MSEC(5));
    // No need to be interrupting now
    gpio_pin_interrupt_configure(gpiodev, TP_CLK_PIN, GPIO_INT_DISABLE);
    // hold up incoming data
    golo(TP_CLK_PIN);
    k_mutex_unlock(&readMutex);
    return readResult;
}

static void poll_trackpoint_fn(struct k_work *work)
{

    write(0xeb);
    uint8_t res = read();
    printk("\n[TP] 0xeb ack: 0x%x\n", res);
    if(0xfa == res) {
        dataReport.state = read();
        dataReport.x = read();
        dataReport.y = read();
        printk("\n[TP] The state is %d, %d, %d.\n", dataReport.state, dataReport.x, dataReport.y);
        if(dataReport.x != 0 && dataReport.y != 0) {
            zmk_hid_mouse_movement_set(0, 0);
            zmk_hid_mouse_scroll_set(0, 0);
            zmk_hid_mouse_movement_update(CLAMP(dataReport.x, INT8_MIN, INT8_MAX),
                                          CLAMP(-dataReport.y, INT8_MIN, INT8_MAX));
            zmk_endpoints_send_mouse_report();
        }
    }
    k_work_reschedule_for_queue(&trackpoint_work_q, work, K_MSEC(50));
}

static void initialize_trackpoint_fn(struct k_work *work)
{
    uint8_t code;
    // Running this _with_ remote mode crashes things :(
    gpio_pin_set(gpiodev, TP_RST_PIN, LOW);
//    write(0xff);
//    printk("\n[TP] Init ack: 0x%x\n", read());
//    printk("\n[TP] Init byte1: 0x%x\n", read());
//    printk("\n[TP] Init byte2: 0x%x\n", read());
    write(0xf0); // Set remote mode
    printk("\n[TP] Wrote\n");
    printk("\n[TP] Set remote mode response: 0x%x", read());
    k_work_init_delayable(&poll_trackpoint, poll_trackpoint_fn);
    k_work_reschedule_for_queue(&trackpoint_work_q, &poll_trackpoint, K_MSEC(50));
}

static void some_work_fn(struct k_work *work)
{
    gpio_pin_toggle(gpiodev, TP_DAT_PIN);
    k_work_reschedule_for_queue(&trackpoint_work_q, work, K_MSEC(500));
}

int zmk_trackpoint_init() {
    gpiodev = device_get_binding("GPIO_1");
    gpio_pin_configure(gpiodev, TP_CLK_PIN, GPIO_INPUT | GPIO_OUTPUT_HIGH );
    gpio_pin_configure(gpiodev, TP_DAT_PIN, GPIO_INPUT | GPIO_OUTPUT_HIGH );
    gpio_pin_configure(gpiodev, TP_RST_PIN, GPIO_OUTPUT);
    gpio_pin_set(gpiodev, TP_RST_PIN, HIGH);
    k_work_queue_start(&trackpoint_work_q, trackpoint_work_stack_area,
                   K_THREAD_STACK_SIZEOF(trackpoint_work_stack_area),
                   CONFIG_SYSTEM_WORKQUEUE_PRIORITY,
                   NULL);
    // Initialize gpio callbacks
    gpio_init_callback(&gpio_read_ctx, read_data_bit, BIT(TP_CLK_PIN));
    gpio_add_callback(gpiodev, &gpio_read_ctx);
    // FOR TRACKPOINT
    k_work_init_delayable(&initialize_trackpoint, initialize_trackpoint_fn);
    k_work_reschedule_for_queue(&trackpoint_work_q, &initialize_trackpoint, K_MSEC(2000));
    // FOR LED
//    k_work_init_delayable(&some_work, some_work_fn);
//    k_work_reschedule_for_queue(&trackpoint_work_q, &some_work, K_MSEC(500));
    return 0;
}