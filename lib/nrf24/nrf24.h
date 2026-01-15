#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <furi_hal_spi.h>
#include <furi_hal_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Command Mnemonics */
#define R_REGISTER          0x00
#define W_REGISTER          0x20
#define REGISTER_MASK       0x1F
#define ACTIVATE            0x50
#define R_RX_PL_WID         0x60
#define R_RX_PAYLOAD        0x61
#define W_TX_PAYLOAD        0xA0
#define W_TX_PAYLOAD_NOACK  0xB0
#define W_ACK_PAYLOAD       0xA8
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define REUSE_TX_PL         0xE3
#define RF24_NOP            0xFF

/* Register Address Mnemonics */
#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_RX_ADDR_P0  0x0A
#define REG_RX_ADDR_P1  0x0B
#define REG_RX_ADDR_P2  0x0C
#define REG_RX_ADDR_P3  0x0D
#define REG_RX_ADDR_P4  0x0E
#define REG_RX_ADDR_P5  0x0F
#define REG_TX_ADDR     0x10
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD       0x1C
#define REG_FEATURE     0x1D

#define RX_PW_P0        0x11
#define RX_DR           0x40
#define TX_DS           0x20
#define MAX_RT          0x10

#define NRF24_CONT_WAVE (1 << 7)
#define NRF24_PLL_LOCK  (1 << 4)
#define NRF24_EN_CRC    (1 << 3)

/* --- Custom Hardware Configuration --- */
#define nrf24_TIMEOUT   500
#define nrf24_CE_PIN    &gpio_ext_pc3 // Pin 7 (PC3)
#define nrf24_CS_PIN    &gpio_ext_pa6 // Pin 3 (PA6)
#define nrf24_HANDLE    ((FuriHalSpiBusHandle*)&furi_hal_spi_bus_handle_external)

/* Function Prototypes */
void nrf24_init();
void nrf24_deinit();
void nrf24_spi_trx(FuriHalSpiBusHandle* handle, uint8_t* tx, uint8_t* rx, uint8_t size, uint32_t timeout);
uint8_t nrf24_write_reg(FuriHalSpiBusHandle* handle, uint8_t reg, uint8_t data);
uint8_t nrf24_write_buf_reg(FuriHalSpiBusHandle* handle, uint8_t reg, uint8_t* data, uint8_t size);
uint8_t nrf24_read_reg(FuriHalSpiBusHandle* handle, uint8_t reg, uint8_t* data, uint8_t size);
uint8_t nrf24_status(FuriHalSpiBusHandle* handle);
uint8_t nrf24_set_rx_mode(FuriHalSpiBusHandle* handle);
uint8_t nrf24_set_tx_mode(FuriHalSpiBusHandle* handle);
uint8_t nrf24_set_idle(FuriHalSpiBusHandle* handle);
uint8_t nrf24_flush_rx(FuriHalSpiBusHandle* handle);
uint8_t nrf24_flush_tx(FuriHalSpiBusHandle* handle);
uint8_t nrf24_txpacket(FuriHalSpiBusHandle* handle, uint8_t* payload, uint8_t size, bool ack);
uint8_t nrf24_rxpacket(FuriHalSpiBusHandle* handle, uint8_t* packet, uint8_t* packetsize, bool full);
bool nrf24_check_connected(FuriHalSpiBusHandle* handle);

#ifdef __cplusplus
}
#endif