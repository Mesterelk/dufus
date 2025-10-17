#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include <stdio.h>

#define NUM_FACES 6
const uint cs_pins[NUM_FACES] = {10, 11, 12, 13, 14, 15};

#define SPI_PORT spi0
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_BROADCAST_ADDR 0x6F

#define DISCOVERY_CMD 0xA0
#define CMD_B000 0x00
#define DISCOVERY_INTERVAL_MS 2000

volatile uint8_t next_id = 1;
volatile bool discovery_received = false;
uint8_t module_type[256];

// Simple I2C RX buffer for ISR
volatile uint8_t i2c_rx_buf[4];
volatile uint8_t i2c_rx_len = 0;

// ---------------- I2C Interrupt Handler ----------------
void i2c0_irq_handler(void) {
    // Read all available bytes
    while (i2c_get_read_available(I2C_PORT)) {
        if (i2c_rx_len < sizeof(i2c_rx_buf)) {
            i2c_rx_buf[i2c_rx_len++] = i2c_read_byte_raw(I2C_PORT);
        } else {
            i2c_rx_len = 0; // overflow protection
        }
    }

    // Process discovery frame
    if (i2c_rx_len >= 3 && i2c_rx_buf[0] == DISCOVERY_CMD) {
        uint8_t id = i2c_rx_buf[1];
        uint8_t type = i2c_rx_buf[2];
        next_id = id + 1;
        module_type[id] = type;
        discovery_received = true;
        i2c_rx_len = 0; // reset buffer
    }

    // Clear interrupt flags
    i2c_hw_t *hw = I2C_PORT == i2c0 ? i2c0_hw : i2c1_hw;
    hw->intr_mask = 0;
}

// ---------------- Setup ----------------
int main() {
    stdio_init_all();
    sleep_ms(200);
    printf("[BATTERY] Booting...\n");

    // --- SPI setup ---
    spi_init(SPI_PORT, 1000000);
    gpio_set_function(18, GPIO_FUNC_SPI); // SCK
    gpio_set_function(19, GPIO_FUNC_SPI); // MOSI
    gpio_set_function(16, GPIO_FUNC_SPI); // MISO

    for (int i = 0; i < NUM_FACES; i++) {
        gpio_init(cs_pins[i]);
        gpio_set_dir(cs_pins[i], GPIO_OUT);
        gpio_put(cs_pins[i], 1);
    }

    // --- I2C setup ---
    i2c_init(I2C_PORT, 100000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Enable I2C interrupt
    irq_set_exclusive_handler(I2C0_IRQ, i2c0_irq_handler);
    irq_set_enabled(I2C0_IRQ, true);

    absolute_time_t last_discovery = get_absolute_time();

    printf("[BATTERY] Ready.\n");

    while (true) {
        // Periodically send discovery over SPI
        if (absolute_time_diff_us(last_discovery, get_absolute_time()) / 1000 >= DISCOVERY_INTERVAL_MS) {
            last_discovery = get_absolute_time();
            printf("[SPI] Broadcasting ID=%d\n", next_id);

            for (int i = 0; i < NUM_FACES; i++) {
                gpio_put(cs_pins[i], 0);
                spi_write_blocking(SPI_PORT, &next_id, 1);
                gpio_put(cs_pins[i], 1);

                // small per-face delay (~1ms max)
                sleep_us(800);
                // Check if discovery was received via I2C interrupt
                if (discovery_received) {
                    discovery_received = false;
                    printf("[I2C] Got module ID=%d, type=%d\n", next_id - 1, module_type[next_id - 1]);
                }
            }
        }
        if (discovery_received) {
            discovery_received = false;
            printf("[I2C] Got module ID=%d, type=%d\n", next_id - 1, module_type[next_id - 1]);
        }
        tight_loop_contents(); // small idle
    }
}
