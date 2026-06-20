/**
 * @file i2c_bus.h
 * @brief Acesso I2C mestre compartilhado pelos sensores no barramento.
 *
 * MPU (IMU) e BMP (barometro) ficam no mesmo barramento I2C. Centralizar o
 * acesso aqui evita duplicar a sequencia de transacao em cada driver e garante
 * que ambos usem a mesma porta e o mesmo timeout.
 *
 * A leitura usa start-repetido (escreve o endereco do registrador e, sem
 * liberar o barramento, le os dados) -- exatamente o que os sensores esperam.
 */
#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/**
 * @brief Inicializa o barramento I2C mestre (porta I2C_NUM_0).
 *
 * Chamar uma vez antes de qualquer leitura/escrita (tipicamente em sensor_hub).
 *
 * @param sda_gpio  GPIO do SDA.
 * @param scl_gpio  GPIO do SCL.
 * @param clock_hz  Frequencia do barramento (ex.: 400000 para fast mode).
 * @return ESP_OK em sucesso; codigo de erro do driver I2C caso contrario.
 */
esp_err_t i2c_bus_init(int sda_gpio, int scl_gpio, uint32_t clock_hz);

/**
 * @brief Escreve um byte em um registrador de um dispositivo.
 *
 * @param device_addr Endereco I2C de 7 bits.
 * @param reg         Registrador de destino.
 * @param value       Byte a escrever.
 * @return true em sucesso; false em falha de comunicacao.
 *
 * @code
 * (void)i2c_bus_write_reg(0x68, PWR_MGMT_1, 0x01); // acorda a MPU
 * @endcode
 */
bool i2c_bus_write_reg(uint8_t device_addr, uint8_t reg, uint8_t value);

/**
 * @brief Le um bloco de registradores consecutivos.
 *
 * @param device_addr Endereco I2C de 7 bits.
 * @param reg         Primeiro registrador a ler.
 * @param buffer      Destino (>= length bytes). Ignorado se NULL.
 * @param length      Numero de bytes a ler (> 0).
 * @return true se todos os bytes foram lidos; false em falha/parametro invalido.
 *
 * @code
 * uint8_t raw[14];
 * if (i2c_bus_read_regs(0x68, ACCEL_XOUT_H, raw, sizeof(raw))) { ... }
 * @endcode
 */
bool i2c_bus_read_regs(uint8_t device_addr, uint8_t reg, uint8_t *buffer, size_t length);

#endif /* I2C_BUS_H */
