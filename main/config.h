/**
 * @file config.h
 * @brief Constantes de hardware, calibracao e controle da controladora de voo.
 *
 * Todos os valores aqui sao validados em voo. Sao o "contrato" do sistema:
 * pinos, faixa de PWM, ganhos de PID e limiares de seguranca. Cada grupo abaixo
 * explica por que o valor e o que ele controla.
 *
 * @warning Estes numeros sao resultado de calibracao e ensaio reais. Alterar
 * qualquer um muda o comportamento de voo e/ou a seguranca; nao mude sem motivo.
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ===== Motores / ESCs =====
 * A faixa 1000..2000 us e o padrao de ESCs de drone (1 ms = parado, 2 ms =
 * maximo). 250 Hz e uma taxa de atualizacao conservadora aceita pela maioria
 * dos ESCs e suficiente para o laco de controle de 20 ms (50 Hz). O arming
 * mantem o sinal minimo por alguns segundos para o ESC reconhecer "throttle
 * zero" e habilitar, evitando partida acidental. */
#define NUM_MOTORS              (4)
#define ESC_MIN_US              (1000)
#define ESC_MAX_US              (2000)
#define ESC_FREQUENCY_HZ        (250)
#define ESC_ARM_HOLD_MS         (5000U)

/* GPIOs dos ESCs M1..M4 na configuracao em X (frente-esq, frente-dir,
 * tras-dir, tras-esq). A ordem define a mixagem; nao reordene sem reavaliar. */
#define ESC_GPIO_M1             (25)
#define ESC_GPIO_M2             (26)
#define ESC_GPIO_M3             (27)
#define ESC_GPIO_M4             (14)

/* Calibracao por motor (defaults de fabrica), em microssegundos.
 * start = pulso minimo em que o motor gira de forma confiavel; max = topo util
 * da faixa; trim = ajuste fino somado a ambos. Compensam diferencas entre
 * motores/ESCs. Persistidos na NVS e ajustaveis pela interface. */
#define DEFAULT_MOTOR_START_US_INIT  { 1100, 1100, 1100, 1100 }
#define DEFAULT_MOTOR_MAX_US_INIT    { 2000, 2000, 2000, 2000 }
#define DEFAULT_MOTOR_TRIM_US_INIT   { 0, 0, 0, 0 }

/* Piso de operacao (us). Quando > 0, eleva o start efetivo para manter os
 * motores fora da regiao nao-linear de baixa rotacao. 0 = desativado; util em
 * bancada para enxergar o deadband real. */
#define DEFAULT_OPERATING_IDLE_US    (0)

/* Limites aceitos para o trim na calibracao (sanitizacao de entrada do usuario). */
#define MOTOR_TRIM_MIN_US       (-100)
#define MOTOR_TRIM_MAX_US       (100)

/* ===== Barramentos / perifericos =====
 * I2C a 400 kHz (fast mode) atende MPU+BMP com folga para o laco de 20 ms.
 * GPS NEO-6M opera em 9600 8N1 (padrao de fabrica do modulo). */
#define I2C_SDA_GPIO            (21)
#define I2C_SCL_GPIO            (22)
#define I2C_CLOCK_HZ            (400000)
#define GPS_RX_GPIO             (16)
#define GPS_TX_GPIO             (17)
#define GPS_BAUD                (9600U)

/* ===== Rede (Access Point) =====
 * O firmware sobe um AP proprio para a interface de controle. */
#define WIFI_AP_SSID            "EQUIPE4-AP"
#define WIFI_AP_PASSWORD        "12345678"

/* ===== Ganhos PID default (malha externa angulo->us, QuadPIDController) =====
 * Usados na mixagem simples (sem estabilizacao por IMU) e como Kp externo da
 * cascata. Yaw tem ganho menor por ser eixo menos critico para estabilidade. */
#define ROLL_GAIN_KP_DEFAULT    (4.0f)
#define ROLL_GAIN_KI_DEFAULT    (0.02f)
#define ROLL_GAIN_KD_DEFAULT    (0.9f)
#define PITCH_GAIN_KP_DEFAULT   (4.0f)
#define PITCH_GAIN_KI_DEFAULT   (0.02f)
#define PITCH_GAIN_KD_DEFAULT   (0.9f)
#define YAW_GAIN_KP_DEFAULT     (2.2f)
#define YAW_GAIN_KI_DEFAULT     (0.01f)
#define YAW_GAIN_KD_DEFAULT     (0.3f)

/* ===== Seguranca / temporizacao da malha de controle =====
 * MIN_THROTTLE: abaixo disso a malha mantem os motores em minimo (evita
 * corrigir atitude sem empuxo). COMMAND_TIMEOUT: sem comando da interface por
 * mais que isso, dispara failsafe (perda de link). MPU_MAX_AGE: dado de IMU
 * mais velho que isso e considerado invalido. MAX_SAFE_TILT: inclinacao acima
 * da qual o drone e considerado fora de controle e a malha desarma.
 * SWAP_ROLL_PITCH: corrige a orientacao fisica da IMU em relacao aos eixos do
 * frame. INTERVAL: periodo do laco de controle (50 Hz). */
#define CONTROL_MIN_THROTTLE_US     (1050)
#define FLIGHT_COMMAND_TIMEOUT_MS   (600U)
#define MPU_MAX_AGE_MS              (150U)
#define MAX_SAFE_TILT_DEG          (65.0f)
#define CONTROL_SWAP_ROLL_PITCH    (true)
#define FLIGHT_CONTROL_INTERVAL_MS (20U)

/* ===== Orientacao/montagem da IMU =====
 * Compensa a rotacao fisica do modulo. Medido nesta montagem: com o drone
 * nivelado, a gravidade cai no eixo -X (accel ~ [-1, 0, 0], pitch ~87 graus).
 * Aplica-se uma rotacao de +90 graus em torno de Y ao acelerometro e ao
 * giroscopio, de modo que "nivelado" passe a ler accel ~ [0, 0, +1]
 * (roll/pitch ~0):  novo_x = antigo_z ; novo_y = antigo_y ; novo_z = -antigo_x
 * Defina 0 se a IMU for remontada com a face para cima (eixo Z para cima). */
#define MPU_REMAP_ROTATE_Y_90  (1)

/* ===== Controle de velocidade vertical (climb-rate hold) =====
 * Funcionalidade adicional (ver secao 11 do guia do projeto): malha INTERNA de
 * velocidade vertical (PI) que substitui o empuxo base quando engatada. Fica
 * separada do caminho congelado e desligada por padrao; so atua se habilitada.
 *
 * HOVER_THROTTLE e o empuxo aproximado de pairar; depende de massa/bateria/helice
 * e deve ser calibrado. Os ganhos sao em microssegundos por (m/s) e precisam de
 * sintonia em bancada. Os limites saturam comando e ajuste. As constantes K1/K2
 * sao o corretor do filtro complementar (baro corrige o drift da integracao do
 * acelerometro). A faixa morta do stick define o ponto de "manter velocidade 0". */
#define VERT_HOVER_THROTTLE_US_DEFAULT (1500.0f)
#define VERT_MAX_VELOCITY_MS           (2.0f)
#define VERT_VEL_KP_US_PER_MS          (120.0f)
#define VERT_VEL_KI_US_PER_MS          (60.0f)
#define VERT_VEL_OUT_LIMIT_US          (400.0f)
#define VERT_EST_K1                    (2.1f)
#define VERT_EST_K2                    (2.25f)
#define VERT_TILT_COMP_MIN_COS         (0.5f)
#define VERT_THROTTLE_DEADBAND_LOW_US  (1450.0f)
#define VERT_THROTTLE_DEADBAND_HIGH_US (1550.0f)
/* Idade maxima da amostra do barometro (ms) para a trava de seguranca aceitar a
 * referencia como valida. Acima disso, o controle vertical desengata. */
#define VERT_BARO_MAX_AGE_MS           (1000U)
#define GRAVITY_MS2                    (9.80665f)
#define DEG_TO_RAD_F                   (0.0174532925f)

#endif /* CONFIG_H */
