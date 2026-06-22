# Controladora de Voo de Quadricoptero (ESP32 / ESP-IDF)

Firmware em **C** sobre **ESP-IDF 4.4** para uma controladora de voo de drone
baseada em **ESP32**. Gera o PWM dos 4 ESCs, le os sensores (IMU, barometro,
GPS), estabiliza o voo com controle PID em cascata e expoe um console web por
Wi-Fi (Access Point) para comando, telemetria e calibracao.

Escrito em C sobre ESP-IDF com foco em confiabilidade, robustez e conformidade
MISRA C. A logica de PWM e de controle e tratada como contrato congelado e
validada em voo.

---

## Recursos

- PWM de 4 ESCs via LEDC (1000–2000 us @ 250 Hz), com calibracao por motor.
- Fusao de atitude (acelerometro + giroscopio + magnetometro) na IMU MPU-925x,
  amostrada a ~200 Hz; `dt` real e deteccao de amostra repetida.
- Barometro BMP280 (altitude relativa filtrada) e GPS NEO-6M (NMEA).
- Controle de voo em cascata (atitude -> velocidade angular) a 50 Hz.
- Mixagem em X com desaturacao priorizada (roll/pitch > coletivo > yaw).
- Failsafes: perda de comando, IMU invalida/nao calibrada, inclinacao excessiva,
  contencao do controlador (`CONTROLLER_TIMEOUT`), parada de emergencia
  (com travamento de rearme).
- Autoteste de eixos (`/axisCheck`) e telemetria de diagnostico (timing/mixer).
- Calibracao persistida em NVS.
- Console web (Wi-Fi AP) com abas de teste, voo e calibracao + rotas JSON.

---

## Hardware

### Ligacoes (definidas em `main/config.h`)

| Funcao            | GPIO            |
|-------------------|-----------------|
| ESC M1 (frente-esq) | 25            |
| ESC M2 (frente-dir) | 26            |
| ESC M3 (tras-dir)   | 27            |
| ESC M4 (tras-esq)   | 14            |
| I2C SDA (MPU/BMP)   | 21            |
| I2C SCL (MPU/BMP)   | 22            |
| GPS RX (UART2)      | 16            |
| GPS TX (UART2)      | 17            |

- ESCs: protocolo servo padrao (1000–2000 us), 250 Hz.
- I2C a 400 kHz; GPS a 9600 8N1.

---

## Requisitos de software

- **ESP-IDF 4.4** instalado e exportado no ambiente.
- Toolchain do ESP32 (instalada junto com o IDF).
- Porta serial USB para o ESP32 (ex.: `/dev/ttyUSB0` no Linux, `COMx` no Windows).

Verifique a versao antes de comecar:

```bash
idf.py --version
```

> Se `idf.py` nao for encontrado, carregue o ambiente do IDF:
> `. $IDF_PATH/export.sh` (Linux/macOS) ou `%IDF_PATH%\export.bat` (Windows).

---

## Build

```bash
# 1) Carregar o ambiente ESP-IDF 4.4
. $IDF_PATH/export.sh

# 2) Definir o alvo (apenas na primeira vez)
idf.py set-target esp32

# 3) Compilar
idf.py build
```

A pagina web (`main/index.html`) e embutida no binario automaticamente
(`EMBED_TXTFILES`); nao ha sistema de arquivos a gravar.

Configuracao opcional (Kconfig/sdkconfig):

```bash
idf.py menuconfig
```

Os defaults relevantes ja vem em `sdkconfig.defaults` (tick de 1 kHz, CPU a
240 MHz, stack da tarefa principal).

---

## Flash e monitor

```bash
# Grava e abre o terminal serial (115200 baud)
idf.py -p /dev/ttyUSB0 flash monitor
```

- Troque `/dev/ttyUSB0` pela sua porta.
- Para sair do monitor: `Ctrl + ]`.
- Para apenas gravar: `idf.py -p /dev/ttyUSB0 flash`.
- Para apenas monitorar: `idf.py -p /dev/ttyUSB0 monitor`.

Se a gravacao falhar, segure **BOOT** ao iniciar o flash (algumas placas exigem)
ou reduza a velocidade: `idf.py -p /dev/ttyUSB0 -b 115200 flash`.

### Apagar a flash (reset de fabrica, inclui a calibracao na NVS)

```bash
idf.py -p /dev/ttyUSB0 erase-flash
```

---

## Primeiro uso

1. Apos o boot, o firmware arma os ESCs por 5 s (motores no minimo) e sobe o AP.
2. Conecte-se a rede Wi-Fi:
   - **SSID:** `EQUIPE4-AP`
   - **Senha:** `12345678`
3. Abra o console: **http://192.168.4.1**
4. Acompanhe o log pelo monitor serial (115200) durante a inicializacao.

> SSID e senha sao definidos em `main/config.h` (`WIFI_AP_SSID` /
> `WIFI_AP_PASSWORD`).

### Rotas HTTP

`/` `/status` `/health` `/sensors` `/setMotor` `/stopAll` `/setFlight`
`/flightStatus` `/axisCheck` `/resetPid` `/setVerticalHold` `/setVerticalGains`
`/calibration` `/setCalibration` `/resetCalibration` `/findDeadband`

Exemplos:

```bash
curl -s http://192.168.4.1/health
curl -s http://192.168.4.1/sensors | python3 -m json.tool
```

---

## Seguranca

> ⚠️ **Faça toda a verificacao inicial SEM HELICES.**

Antes de qualquer voo, siga o procedimento de bancada em
[`TESTE_BANCADA.md`](TESTE_BANCADA.md): boot, conectividade, deadband por motor,
sensores, malha de controle, failsafes e conferencia do pulso do ESC no
osciloscopio.

---

## Estrutura do projeto

```
CMakeLists.txt          Projeto ESP-IDF
sdkconfig.defaults      Defaults de build
Doxyfile                Configuracao da documentacao
MODELAGEM_E_CONTROLE.md Equacionamento (estimativa, PID, cascata, mixagem, vertical)
TESTE_BANCADA.md        Guia de teste em bancada (sem helices)
main/
  app_main.c            Inicializacao e criacao das tarefas
  config.h              Pinos, faixas de PWM, ganhos e limiares
  app_math.h            Saturacao (clamp) e mapeamento linear inteiro
  app_time.h            millis()
  app_state.c/.h        Flags globais (arming, telemetria)
  drone_pid.c/.h        PID, QuadPID e controle em cascata + mixagem X
  esc_pwm.c/.h          Geracao de PWM (LEDC) e calibracao dos motores
  i2c_bus.c/.h          Acesso I2C compartilhado
  calibration_store.c/.h  Persistencia da calibracao (NVS)
  mpu9259.c/.h          Driver da IMU + fusao de atitude
  bmp280.c/.h           Driver do barometro + altitude relativa
  neo6m_gps.c/.h        Driver/parser GPS NMEA
  sensor_hub.c/.h       Agregacao de sensores + telemetria JSON
  flight_control.c/.h   Tarefa de controle (20 ms) + failsafes
  wifi_ap.c/.h          Access Point Wi-Fi
  web_server.c/.h       Servidor HTTP e rotas
  index.html            Console web (embutido no binario)
```

---

## Documentacao tecnica e de API

Para a **modelagem e o equacionamento** (fusao de atitude, barometria, PID,
cascata atitude->taxa, mixagem em X, controle vertical e failsafes) com o
mapeamento para o codigo, veja [`MODELAGEM_E_CONTROLE.md`](MODELAGEM_E_CONTROLE.md).

O codigo e documentado em estilo Doxygen. Para gerar a referencia HTML:

```bash
sudo apt install doxygen graphviz   # graphviz e opcional (grafos)
doxygen Doxyfile
# abre: docs/doxygen/html/index.html
```

Para PDF: defina `GENERATE_LATEX = YES` no `Doxyfile`, rode `doxygen Doxyfile` e
depois `make -C docs/doxygen/latex` (requer LaTeX).

---

## Solucao de problemas

| Sintoma | Acao |
|--------|------|
| `idf.py: command not found` | Carregue o ambiente: `. $IDF_PATH/export.sh` |
| Falha ao gravar | Segure BOOT no inicio do flash; tente `-b 115200`; confira a porta |
| ESC apita sem parar | Calibre o range do ESC (1000–2000 us) ou ajuste `start/max` |
| Pagina nao carrega | Confirme conexao ao AP `EQUIPE4-AP` e o IP 192.168.4.1; veja `/health` |
| Motor gira ao contrario | Inverta 2 das 3 fases motor<->ESC |
| Mais diagnosticos | Veja a secao 11 de [`TESTE_BANCADA.md`](TESTE_BANCADA.md) |
