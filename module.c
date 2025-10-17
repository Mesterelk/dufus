#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <stdio.h>

// ===================== CONFIG =====================
#define NUM_INPUT_CS 6
#define NUM_OUTPUT_CS 6
const uint cs_input_pins[NUM_INPUT_CS]  = {2, 3, 4, 5, 6, 7};
const uint cs_output_pins[NUM_OUTPUT_CS] = {10, 11, 12, 13, 14, 15};

#define SPI_PORT spi0
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_BROADCAST_ADDR 0x6F

#define DISCOVERY_CMD 0xA0
#define CMD_B000 0x00
#define CMD_B001 0x01

const uint8_t MODULE_TYPE = 2;
const uint32_t SPI_SPEED = 1000000;
const uint32_t I2C_SPEED = 100000;
const uint32_t WAIT_US = 6000;

// ===================== STATE =====================
volatile uint8_t my_id = 255;
volatile uint8_t next_id = 1;
volatile bool got_spi_id = false;
volatile bool discovery_phase = false;
volatile bool send_sensor = false;
volatile uint8_t source_face = 255;
absolute_time_t time_received;
volatile bool ready = false;

volatile uint8_t i2c_rx_buf[4];
volatile uint8_t i2c_rx_len = 0;

// ===================== SENSOR STUB =====================
uint8_t get_sensor_data(void) {
    // TODO: replace with real ultrasonic or sensor logic
    return 42;
}

// ===================== I2C INTERRUPT HANDLER =====================
void i2c0_irq_handler(void) {
    while (i2c_get_read_available(I2C_PORT)) {
        if (i2c_rx_len < sizeof(i2c_rx_buf)) {
            i2c_rx_buf[i2c_rx_len++] = i2c_read_byte_raw(I2C_PORT);
        } else {
            i2c_rx_len = 0; // overflow protection
        }
    }

    if (i2c_rx_len >= 2) {
        uint8_t cmd = i2c_rx_buf[0];
        if (cmd == CMD_B000 && i2c_rx_len >= 2) {
            uint8_t target_id = i2c_rx_buf[1];
            if (target_id == my_id) send_sensor = true;
        } else if (cmd == CMD_B001 && i2c_rx_len >= 2) {
            uint8_t recv_id = i2c_rx_buf[1];
            next_id = recv_id + 1;
            printf("[I2C] CMD_B001: next_id=%u\n", next_id);
        } else if (cmd == DISCOVERY_CMD && i2c_rx_len >= 3) {
            next_id = i2c_rx_buf[1] + 1;
            printf("[I2C] DISCOVERY: next_id=%u type=%u\n", i2c_rx_buf[1], i2c_rx_buf[2]);
        }
        i2c_rx_len = 0; // reset after processing
    }

    // Clear interrupt flags
    i2c_hw_t *hw = I2C_PORT == i2c0 ? i2c0_hw : i2c1_hw;
    hw->intr_mask = 0;
}

// ===================== SPI INTERRUPT HANDLER =====================
void handle_cs(uint gpio, uint32_t events) {
    uint8_t rx;
    spi_read_blocking(SPI_PORT, 0x00, &rx, 1);
    if (rx == 0 || rx == 0xFF) return;
    my_id = rx;
    next_id = my_id + 1;
    got_spi_id = true;
    discovery_phase = true;
    time_received = get_absolute_time();
    source_face = gpio;
}

// ===================== MAIN =====================
int main() {
    stdio_init_all();
    sleep_ms(200);
    printf("\n[MODULE] Booting...\n");

    // SPI init
    spi_init(SPI_PORT, SPI_SPEED);
    gpio_set_function(18, GPIO_FUNC_SPI); // SCK
    gpio_set_function(19, GPIO_FUNC_SPI); // MOSI
    gpio_set_function(16, GPIO_FUNC_SPI); // MISO

    for (int i = 0; i < NUM_OUTPUT_CS; i++) {
        gpio_init(cs_output_pins[i]);
        gpio_set_dir(cs_output_pins[i], GPIO_OUT);
        gpio_put(cs_output_pins[i], 1);
    }

    for (int i = 0; i < NUM_INPUT_CS; i++) {
        gpio_init(cs_input_pins[i]);
        gpio_set_dir(cs_input_pins[i], GPIO_IN);
        gpio_pull_up(cs_input_pins[i]);
        gpio_set_irq_enabled_with_callback(cs_input_pins[i], GPIO_IRQ_EDGE_FALL, true, &handle_cs);
    }

    // I2C init
    i2c_init(I2C_PORT, I2C_SPEED);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Enable I2C interrupt
    irq_set_exclusive_handler(I2C0_IRQ, i2c0_irq_handler);
    irq_set_enabled(I2C0_IRQ, true);

    ready = true;
    printf("[MODULE] Ready.\n");

    while (true) {
        // 1. Got new ID via SPI
        if (got_spi_id) {
            got_spi_id = false;
            uint8_t msg[3] = {DISCOVERY_CMD, my_id, MODULE_TYPE};
            i2c_write_blocking(I2C_PORT, I2C_BROADCAST_ADDR, msg, 3, false);
            printf("[I2C] Sent DISCOVERY_CMD ID=%d TYPE=%d\n", my_id, MODULE_TYPE);
        }

        // 2. Send sensor data if requested
        if (send_sensor) {
            send_sensor = false;
            uint8_t val = get_sensor_data();
            uint8_t msg[2] = {CMD_B001, val};
            i2c_write_blocking(I2C_PORT, I2C_BROADCAST_ADDR, msg, 2, false);
            printf("[I2C] Sent SENSOR=%u\n", val);
        }

        // 3. SPI relay logic (timed propagation)
        if (discovery_phase &&
            absolute_time_diff_us(time_received, get_absolute_time()) > WAIT_US * (my_id + 1)) {
            discovery_phase = false;
            for (int i = 0; i < NUM_OUTPUT_CS; i++) {
                gpio_put(cs_output_pins[i], 0);
                spi_write_blocking(SPI_PORT, &next_id, 1);
                gpio_put(cs_output_pins[i], 1);
                sleep_us(800);
            }
            printf("[SPI] Relayed next_id=%u\n", next_id);
        }

        tight_loop_contents();
    }
}
