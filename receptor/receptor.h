#ifndef RECEPTOR // Previne múltiplas inclusões do cabeçalho
#define RECEPTOR

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include <string.h>

#include "lib/ssd1306.h"
#include "lib/font.h"

// ============================================================================
// == DISPLAY OLED1306 (i2c) ==================================================
// ============================================================================
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
ssd1306_t ssd;
bool cor = true;

// ============================================================================
// == Definições dos Pinos e SPI ==============================================
// ============================================================================
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19
#define PIN_RST 20

// ============================================================================
// == Definições dos Registradores LoRa (RFM95/SX1276) ========================
// ============================================================================
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_LNA 0x0C
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_FIFO_RX_BASE_ADDR 0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1A
#define REG_MODEM_CONFIG_1 0x1D
#define REG_MODEM_CONFIG_2 0x1E
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MAX_PAYLOAD_LENGTH 0x23
#define REG_DIO_MAPPING_1 0x40
#define REG_VERSION 0x42
#define REG_PA_DAC 0x4D

// ============================================================================
// == Modos de Operação (REG_OP_MODE) =========================================
// ============================================================================
#define MODE_SLEEP 0x80         // LoRa Mode + Sleep
#define MODE_STDBY 0x81         // LoRa Mode + Standby
#define MODE_TX 0x83            // LoRa Mode + TX
#define MODE_RX_CONTINUOUS 0x85 // LoRa Mode + RX Continuous

// ============================================================================
// == Flags de Interrupção (REG_IRQ_FLAGS) ====================================
// ============================================================================
#define IRQ_RX_DONE_MASK 0x40
#define IRQ_TX_DONE_MASK 0x08
#define IRQ_CAD_DONE_MASK 0x04
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20

// Frequência de operação (915 MHz para o Brasil)
#define LORA_FREQUENCY_HZ 915000000
#define RF_CRYSTAL_FREQ_HZ 32000000

void inicializar_display_i2c();
void rmf95_reset();
void rmf95_write_reg(uint8_t reg, uint8_t value);
uint8_t rmf95_read_reg(uint8_t reg);
void rmf95_read_fifo(uint8_t *buffer, uint8_t length);
void lora_set_frequency(long frequency);
void lora_init();
#endif // RECEPTOR