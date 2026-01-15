#include "nrf24.h"
#include <furi.h>
#include <furi_hal.h>
#include <string.h>



void nrf24_init() {
    // 1. Initialize CE Pin (PC3 / Pin 7)
    furi_hal_gpio_init(nrf24_CE_PIN, GpioModeOutputPushPull, GpioPullUp, GpioSpeedVeryHigh);
    furi_hal_gpio_write(nrf24_CE_PIN, false);

    // 2. Initialize CS Pin (PA6 / Pin 3)
    furi_hal_gpio_init(nrf24_CS_PIN, GpioModeOutputPushPull, GpioPullUp, GpioSpeedVeryHigh);
    furi_hal_gpio_write(nrf24_CS_PIN, true); // CS is Active Low

    // 3. Setup SPI
    furi_hal_spi_acquire(nrf24_HANDLE);
}

void nrf24_deinit() {
    furi_hal_spi_release(nrf24_HANDLE);
    
    // Cleanup pins
    furi_hal_gpio_write(nrf24_CE_PIN, false);
    furi_hal_gpio_write(nrf24_CS_PIN, true);
    furi_hal_gpio_init(nrf24_CE_PIN, GpioModeAnalog, GpioPullNo, GpioSpeedLow);
    furi_hal_gpio_init(nrf24_CS_PIN, GpioModeAnalog, GpioPullNo, GpioSpeedLow);
}

void nrf24_spi_trx(FuriHalSpiBusHandle* handle, uint8_t* tx, uint8_t* rx, uint8_t size, uint32_t timeout) {
    UNUSED(timeout);
    // Use manual toggle for Pin 3 (PA6)
    furi_hal_gpio_write(nrf24_CS_PIN, false);
    furi_hal_spi_bus_trx(handle, tx, rx, size, nrf24_TIMEOUT);
    furi_hal_gpio_write(nrf24_CS_PIN, true);
}

uint8_t nrf24_write_reg(FuriHalSpiBusHandle* handle, uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {W_REGISTER | (REGISTER_MASK & reg), data};
    uint8_t rx[2] = {0};
    nrf24_spi_trx(handle, tx, rx, 2, nrf24_TIMEOUT);
    return rx[0];
}

uint8_t nrf24_read_reg(FuriHalSpiBusHandle* handle, uint8_t reg, uint8_t* data, uint8_t size) {
    uint8_t tx[size + 1];
    uint8_t rx[size + 1];
    memset(tx, 0, size + 1);
    tx[0] = R_REGISTER | (REGISTER_MASK & reg);
    nrf24_spi_trx(handle, tx, rx, size + 1, nrf24_TIMEOUT);
    memcpy(data, &rx[1], size);
    return rx[0];
}

uint8_t nrf24_status(FuriHalSpiBusHandle* handle) {
    uint8_t tx = RF24_NOP;
    uint8_t rx = 0;
    nrf24_spi_trx(handle, &tx, &rx, 1, nrf24_TIMEOUT);
    return rx;
}

bool nrf24_check_connected(FuriHalSpiBusHandle* handle) {
    uint8_t status = nrf24_status(handle);
    return (status != 0x00 && status != 0xFF);
}

uint8_t nrf24_set_idle(FuriHalSpiBusHandle* handle) {
    uint8_t cfg = 0;
    nrf24_read_reg(handle, REG_CONFIG, &cfg, 1);
    cfg &= 0xFC; 
    furi_hal_gpio_write(nrf24_CE_PIN, false);
    return nrf24_write_reg(handle, REG_CONFIG, cfg);
}

uint8_t nrf24_flush_rx(FuriHalSpiBusHandle* handle) {
    uint8_t tx = FLUSH_RX;
    uint8_t rx = 0;
    nrf24_spi_trx(handle, &tx, &rx, 1, nrf24_TIMEOUT);
    return rx;
}

uint8_t nrf24_flush_tx(FuriHalSpiBusHandle* handle) {
    uint8_t tx = FLUSH_TX;
    uint8_t rx = 0;
    nrf24_spi_trx(handle, &tx, &rx, 1, nrf24_TIMEOUT);
    return rx;
}