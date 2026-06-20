/**
 * @file i2c_bus.c
 * @brief Implementacao do acesso I2C mestre (ver i2c_bus.h).
 *
 * Usa o driver I2C do IDF 4.4. O timeout curto evita que uma falha de um sensor
 * (fio solto, dispositivo travado) bloqueie o laco de leitura indefinidamente.
 */
#include "i2c_bus.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"

#define I2C_BUS_PORT       I2C_NUM_0
/* Timeout por transacao: limita o impacto de um sensor que nao responde. */
#define I2C_BUS_TIMEOUT_MS (20)

esp_err_t i2c_bus_init(int sda_gpio, int scl_gpio, uint32_t clock_hz)
{
    const i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = { .clk_speed = clock_hz }
    };

    esp_err_t err = i2c_param_config(I2C_BUS_PORT, &config);
    if (err == ESP_OK)
    {
        err = i2c_driver_install(I2C_BUS_PORT, I2C_MODE_MASTER, 0, 0, 0);
    }
    return err;
}

bool i2c_bus_write_reg(uint8_t device_addr, uint8_t reg, uint8_t value)
{
    const uint8_t payload[2] = { reg, value };
    const esp_err_t err = i2c_master_write_to_device(
        I2C_BUS_PORT, device_addr, payload, sizeof(payload),
        pdMS_TO_TICKS(I2C_BUS_TIMEOUT_MS));
    return (err == ESP_OK);
}

bool i2c_bus_read_regs(uint8_t device_addr, uint8_t reg, uint8_t *buffer, size_t length)
{
    bool ok = false;
    if ((buffer != NULL) && (length > 0U))
    {
        const esp_err_t err = i2c_master_write_read_device(
            I2C_BUS_PORT, device_addr, &reg, 1U, buffer, length,
            pdMS_TO_TICKS(I2C_BUS_TIMEOUT_MS));
        ok = (err == ESP_OK);
    }
    return ok;
}
