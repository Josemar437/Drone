# Guia de Teste em Bancada (SEM HÉLICES)

Procedimento de verificação do firmware ESP-IDF, **com os motores instalados mas
SEM HÉLICES**. Valida, passo a passo, PWM, sensores, malha de controle e
failsafes antes de qualquer tentativa de voo.

> ⚠️ **SEGURANÇA — leia antes de ligar**
> - **NUNCA** execute estes testes com hélices montadas. Um motor a 100% sem
>   hélice ainda é perigoso; com hélice é grave.
> - Fixe o frame à bancada (morsa/braçadeira). Motores produzem torque.
> - Use fonte com **limite de corrente** (ou bateria com o frame preso).
>   Mantenha um botão/chave para cortar a energia ao alcance da mão.
> - Verifique a ordem das fases motor↔ESC e o sentido de giro **antes** de
>   pensar em hélices.
> - Mantenha mãos, cabos e ferramentas longe dos rotores.

---

## 0. Pré-requisitos

- ESP-IDF **4.4** instalado e no PATH (`idf.py --version`).
- ESP32 conectado via USB; saber a porta (ex.: `/dev/ttyUSB0`).
- 4 ESCs + motores ligados aos GPIOs definidos em `main/config.h`:

| Motor | Posição (config X) | GPIO |
|------|---------------------|------|
| M1   | frente-esquerda     | 25   |
| M2   | frente-direita      | 26   |
| M3   | traseira-direita    | 27   |
| M4   | traseira-esquerda   | 14   |

- Sensores no I²C: **SDA=21, SCL=22** (400 kHz). GPS na UART2: **RX=16, TX=17**.
- Opcional, mas recomendado: **osciloscópio** (ou analisador lógico) para
  conferir o pulso dos ESCs (passo 9).
- Um celular/PC com Wi-Fi e/ou `curl` na mesma máquina de teste.

---

## 1. Build e gravação

```bash
. $IDF_PATH/export.sh
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor   # grava e abre o Serial a 115200
```

Mantenha o `monitor` aberto durante todos os testes — boa parte da verificação é
feita pelo log Serial.

---

## 2. Verificação de boot (Serial 115200)

Ao reiniciar, confira a sequência de logs (a ordem importa):

```
===== ESP32 MOTOR CONTROLLER (ESP-IDF) =====
Motor 1 anexado no GPIO 25, canal 0
Motor 2 anexado no GPIO 26, canal 1
Motor 3 anexado no GPIO 27, canal 2
Motor 4 anexado no GPIO 14, canal 3
Armando motores - mantendo sinal minimo por 5000 ms...
ESCs armados com sinal minimo!
MPU925x: conectado
Magnetometro: conectado            (ou "nao encontrado" se não houver AK8963)
BMP280: conectado
GPS NEO-6M: porta serial inicializada
Configurando Access Point: EQUIPE4-AP
AP ativo. IP: 192.168.4.1
[INIT] Mutex da malha de controle: OK
[HTTP] Servidor iniciado na porta 80.
[TASK] Telemetry: OK
[TASK] FlightControl: OK
===== SISTEMA PRONTO =====
Rede: EQUIPE4-AP | Senha: 12345678 | http://192.168.4.1
```

**Critérios de aprovação (passo 2):**
- [ ] Os 4 motores anexados sem `ERRO`.
- [ ] Durante os 5 s de arming, os ESCs **apitam o tom de armado** e os motores
      **não giram**.
- [ ] `MPU925x: conectado` e `BMP280: conectado` (se os sensores estiverem
      presentes). `GPS` é informativo.
- [ ] `AP ativo`, `Servidor iniciado`, ambas as tarefas `OK`.

> Se um ESC não armar (ficar apitando indefinidamente), confira alimentação do
> ESC e a calibração do range (passo 7) — pode ser necessário calibrar o ESC ao
> range 1000–2000 µs.

---

## 3. Conectividade Wi-Fi e página web

1. Conecte-se à rede **`EQUIPE4-AP`** (senha `12345678`).
2. Abra `http://192.168.4.1` — deve carregar o console (abas Teste / Voo /
   Calibração).
3. Teste de saúde por `curl` (de um PC conectado ao AP):

```bash
curl -s http://192.168.4.1/health
```
Esperado (valores variam):
```json
{"ok":true,"heap":NNNNN,"maxAllocHeap":NNNNN,"htmlBytes":70307,
 "commandTask":true,"telemetryTask":true,"flightTask":true,"motorsAtMinimum":true}
```

**Critérios:** `commandTask`, `telemetryTask`, `flightTask` = `true`;
`motorsAtMinimum` = `true`; `htmlBytes` > 0.

---

## 4. Caracterização de deadband por motor (`/findDeadband`)

Sobe o pulso de **um** motor gradualmente (os outros ficam em mínimo). Use para
descobrir o menor pulso em que cada motor começa a girar de forma confiável.
A varredura é **bloqueante** e escreve o pulso **bruto** (sem calibração).

Para o motor 1 (índice 0), de 1000 a 1250 µs, passo 2 µs, 400 ms por degrau:

```bash
curl -s "http://192.168.4.1/findDeadband?motor=0&from=1000&to=1250&step=2&dwell=400"
```

No Serial:
```
=== Varredura de deadband - Motor 1 (SEM HELICES) ===
  pulso 1000 us (0%)
  pulso 1002 us (0%)
  ...
=== Fim da varredura. Motor parado. ===
```

Anote o pulso em que o motor **começa a girar com firmeza**. Repita para
`motor=1`, `2`, `3`.

**Critérios:**
- [ ] Cada motor gira (sem hélice) acima de um certo pulso e **para** ao fim.
- [ ] Os 4 motores giram no **sentido correto** para a config X (inverta 2 fases
      do motor↔ESC se algum estiver invertido).
- [ ] Uma nova varredura é rejeitada enquanto outra roda:
      `curl` retorna **409** "Varredura ja em andamento".

> Use os valores anotados para ajustar `start` de cada motor no passo 7.

---

## 5. Teste de motores individuais (`/setMotor`)

`speed` é **percentual 0–100**. Internamente vira `1000 + pct*10` µs lógico e
passa pela calibração antes de ir ao ESC.

```bash
curl -s "http://192.168.4.1/setMotor?motor=0&speed=20"   # M1 a 20%
```
Serial:
```
[Motor 1] 1200 us logico -> 1218 us ESC (20%)
```
(o "us ESC" depende da calibração; com defaults `start=1100/max=2000`.)

Pare tudo:
```bash
curl -s http://192.168.4.1/stopAll      # -> "Todos parados"
```

**Critérios:**
- [ ] Cada motor responde ao seu índice (0→M1 … 3→M4); confira que o índice bate
      com a **posição física**.
- [ ] `/status` reflete o estado:
```bash
curl -s http://192.168.4.1/status
# {"speeds":[20,0,0,0],"outputs":[1218,1000,1000,1000],"arming":false,
#  "flightControlEnabled":false,"failsafe":"NONE"}
```
- [ ] `/stopAll` zera todos (speeds todos 0, outputs 1000).

---

## 6. Sensores (`/sensors`)

```bash
curl -s http://192.168.4.1/sensors | python3 -m json.tool
```

Verifique, com a placa **parada e nivelada**:
- **MPU**: `online:true`, `valid:true`. Deixe **imóvel** ~5 s até
  `calibrationComplete:true` (acompanhe `calibrationProgress` 0→100).
  Com a placa nivelada, `realAttitude.roll` e `.pitch` ≈ 0°. Incline a placa e
  veja roll/pitch acompanharem.
- **BMP**: `online:true`; após ~11 s parado, `referenceReady:true` e
  `altitudeM` ≈ 0 (sobe/desce ao mover a placa verticalmente).
- **GPS**: `valid` só fica `true` com céu aberto/antena; em bancada fechada é
  normal `signalLost:true`.

**Critérios:**
- [ ] MPU calibra (`calibrationComplete:true`) e a atitude segue a inclinação
      física **no sentido certo**.
- [ ] BMP atinge `referenceReady:true`.

> A malha estabilizada **só arma** com `calibrationComplete:true` (senão failsafe
> `MPU_NOT_CALIBRATED`).

### 6.1 Autoteste de eixos (`/axisCheck`) — validação da convenção de frame (CRÍTICO)

Confirma que a cadeia de sinais (IMU → swap roll/pitch → cascata → mixer) reage
**no sentido certo**. É **read-only**: não aciona motores nem altera o controlador
ativo. Faça isto **antes** de habilitar a malha estabilizada (8.2).

No console web, aba **Controle de Voo** → painel **"Autoteste de eixos"**: clique
**RODAR AUTOTESTE** ou ligue **AUTO** para ver `motorDelta` ao vivo enquanto
inclina o frame (verde = acelera, vermelho = desacelera). Equivalente por linha de
comando:

```bash
curl -s http://192.168.4.1/axisCheck | python3 -m json.tool
```

Saída (campos):
- `state` (roll/pitch/yaw medidos), `rate` (taxas),
- `correction` (o que a cascata aplicaria com setpoint nivelado),
- `motors` e `motorDelta` (delta de cada motor vs. hover; **>0 = aceleraria**),
- `motorLabels`: `["M1_FL","M2_FR","M3_RR","M4_RL"]`.

Com a placa **nivelada e parada**: `state` ≈ 0 e `motorDelta` ≈ `[0,0,0,0]`.

Incline **e segure** o frame e leia `motorDelta` — os motores do **lado baixo
devem acelerar** (delta > 0):

| Inclinação física | Lado baixo | `motorDelta` esperado (>0) |
|-------------------|-----------|-----------------------------|
| Frente para baixo (nariz desce) | frente | **M1_FL, M2_FR** |
| Trás para baixo | traseira | **M3_RR, M4_RL** |
| Direita para baixo | direita | **M2_FR, M3_RR** |
| Esquerda para baixo | esquerda | **M1_FL, M4_RL** |
| Girar (yaw) à direita | — | par de reação acelera (ver sinais de yaw) |

**Critérios:**
- [ ] Nivelado: `state` e `motorDelta` ≈ 0.
- [ ] Para **cada** inclinação, os motores do lado baixo aparecem com `motorDelta>0`
      e os do lado alto com `motorDelta<0` (conforme a tabela).
- [ ] Yaw: ao girar, o par de rotação correto acelera.

> ⚠️ Se **qualquer** sentido estiver invertido, **PARE**: há inversão de eixo/sinal
> (rever `MPU_REMAP_ROTATE_Y_90` / `CONTROL_SWAP_ROLL_PITCH` em `config.h` e a
> montagem física da IMU). Habilitar a estabilização nessa condição causa
> **realimentação positiva → tombamento na decolagem**. Não prossiga para 8.2.

---

## 7. Calibração dos motores (`/calibration`, `/setCalibration`, `/resetCalibration`)

Aplique os `start` medidos no passo 4 (e ajuste `max`/`trim` se necessário):

```bash
# Lê a calibração atual
curl -s http://192.168.4.1/calibration

# Define start/max/trim por motor (exemplo)
curl -s "http://192.168.4.1/setCalibration?start0=1120&max0=2000&trim0=0&start1=1110&max1=2000&trim1=0&start2=1120&max2=2000&trim2=0&start3=1115&max3=2000&trim3=0"

# Restaura defaults de fábrica
curl -s http://192.168.4.1/resetCalibration
```

**Critérios:**
- [ ] Após `setCalibration`, os valores voltam no JSON e **persistem após
      reboot** (gravados na NVS). Reinicie e confira com `/calibration`.
- [ ] Os limites são sanitizados (`start` em 1000–1999, `max > start`,
      `trim` em −100..100).

---

## 8. Malha de controle e failsafes (`/setFlight`, `/flightStatus`, `/stopAll`)

> Continua **SEM HÉLICES**. Os motores vão girar; mantenha o frame preso.

### 8.1 Mixagem manual (sem estabilização)
Calcula a mixagem X e aplica diretamente (não usa a IMU):
```bash
curl -s "http://192.168.4.1/setFlight?throttle=1300&rollSp=0&pitchSp=0&yawSp=0&apply=1&stabilize=0"
curl -s http://192.168.4.1/flightStatus | python3 -m json.tool
```
- [ ] `motors[]` ≈ todos iguais com roll/pitch/yaw = 0.
- [ ] Aumente `pitchSp`/`rollSp` e veja a assimetria correta entre M1–M4
      (config X: pitch+ sobe frente, etc.).

### 8.2 Malha estabilizada (usa a IMU)
Requer `calibrationComplete:true` (passo 6). Envie **repetidamente** (a cada
<600 ms) para não disparar timeout:
```bash
while true; do
  curl -s "http://192.168.4.1/setFlight?throttle=1300&rollSp=0&pitchSp=0&apply=1&stabilize=1" >/dev/null
  sleep 0.2
done
```
- [ ] `/flightStatus` mostra `controlEnabled:true`, `failsafe:"NONE"`.
- [ ] **Incline o frame** levemente (preso!): os motores do lado baixo devem
      **acelerar** e os do lado alto **desacelerar** para corrigir
      (veja `motors[]` e `correction`).

### 8.3 Testes de failsafe (um a um)
Com a malha estabilizada ativa (8.2):

| Teste | Como provocar | Esperado |
|------|----------------|----------|
| **Throttle baixo** | enviar `throttle=1000` | motores vão a mínimo (idle), sem desarmar |
| **Timeout de comando** | **parar** o loop de `setFlight` >600 ms | Serial: `[CONTROL] Desabilitado. Motivo: COMMAND_TIMEOUT \| parar motores: SIM`; `failsafeLatched:true` |
| **Inclinação excessiva** | inclinar > 65° | `EXCESSIVE_TILT`; motores a mínimo; latched |
| **MPU inválida** | desconectar/derrubar I²C (cuidado) | `MPU_INVALID`; latched |
| **Parada de emergência** | `curl /stopAll` | tudo a mínimo; **limpa** o latch |

- [ ] Após qualquer failsafe **latched**, uma nova tentativa de habilitar
      (`apply=1&stabilize=1`) é **rejeitada com 409** até executar `/stopAll`.
- [ ] `/stopAll` zera o latch e permite rearmar.

```bash
# Deve retornar 409 enquanto houver failsafe latched:
curl -s -o /dev/null -w "%{http_code}\n" "http://192.168.4.1/setFlight?throttle=1300&apply=1&stabilize=1"
```

### 8.4 Reset de PID
```bash
curl -s http://192.168.4.1/resetPid     # -> "PID reiniciado"
```

### 8.5 Desaturação e priorização do mixer (saturação)
Valida a priorização **roll/pitch > coletivo > yaw** quando o comando não cabe na
faixa do ESC. Só o caminho **estabilizado** (8.2) usa o mixer com desaturação
(`quad_pid_mix_x_desaturated`); a mixagem manual (8.1) **não** exercita esta lógica.

> ⚠️ **SEM HÉLICES** e frame **firmemente preso**: este teste usa throttle alto
> (motores girando rápido) combinado com inclinação para forçar a saturação.

Com a malha estabilizada ativa (loop do 8.2), use **throttle alto** e **incline o
frame** (preso!) de forma acentuada, porém **< 65°** (acima disso dispara
`EXCESSIVE_TILT`):
```bash
# Loop de comando com throttle alto (mantenha rodando, <600 ms entre envios)
while true; do
  curl -s "http://192.168.4.1/setFlight?throttle=1900&rollSp=0&pitchSp=0&apply=1&stabilize=1" >/dev/null
  sleep 0.2
done
# Em outro terminal, observe os blocos "mixer" e "timing":
curl -s http://192.168.4.1/flightStatus | python3 -m json.tool
```

Inclinando o frame (gerando correção grande) com throttle a 1900:
- [ ] `mixer.saturated` vai a **`true`** durante a saturação.
- [ ] `mixer.collectiveAdjustUs` fica **negativo** (o coletivo é **reduzido** —
      air-mode — para preservar o torque de roll/pitch).
- [ ] Em `motors[]`, o diferencial de roll/pitch é **mantido**: os motores do lado
      alto saturam perto de 2000, mas os do lado baixo ficam **claramente menores**
      (não colapsam todos no mesmo valor). `correction.roll`/`.pitch` continuam
      refletindo a correção comandada.

Agora some **yaw** ao mesmo cenário saturado e confirme que **o yaw cede primeiro**:
```bash
while true; do
  curl -s "http://192.168.4.1/setFlight?throttle=1900&yawSp=90&apply=1&stabilize=1" >/dev/null
  sleep 0.2
done
```
- [ ] Com roll/pitch já saturando, o diferencial de yaw entre os pares (M1,M3) e
      (M2,M4) aparece **reduzido/ausente** em `motors[]`, enquanto roll/pitch são
      preservados — o yaw é escalado para a folga restante.
- [ ] Sem saturação (ex.: `throttle=1300`, frame nivelado), `mixer.saturated` é
      **`false`** e `collectiveAdjustUs` ≈ 0 (saída idêntica à mixagem normal).

> **Diagnóstico de tempo real (bloco `timing`):** aproveite para conferir
> `controlDtMs` ≈ 20, `loopJitterMs` ≈ 0, `imuAgeMs` baixo e `repeatedImuSamples`
> **estável** (não crescente) — confirmam a sincronização IMU↔controle. Se a
> cascata não obtiver o mutex no prazo, o failsafe aparece como
> `failsafe:"CONTROLLER_TIMEOUT"` (distinto de `MPU_INVALID`).

---

## 9. Verificação do pulso do ESC no osciloscópio (CRÍTICO)

Esta é a checagem que garante que a saída em microssegundos gerada pelo LEDC
corresponde ao pulso comandado. Faça **antes de qualquer voo**.

1. Sonde o sinal num GPIO de ESC (ex.: GPIO25 = M1) e o GND.
2. Use a varredura bruta para pulsos conhecidos (sem calibração):
   ```bash
   curl -s "http://192.168.4.1/findDeadband?motor=0&from=1000&to=2000&step=500&dwell=2000"
   ```
3. Meça em cada patamar:

| Comando | Pulso esperado | Período esperado |
|--------|----------------|------------------|
| 1000 µs | **1.00 ms** ± alguns µs | **4.00 ms** (250 Hz) |
| 1500 µs | **1.50 ms** | 4.00 ms |
| 2000 µs | **2.00 ms** | 4.00 ms |

**Critérios:**
- [ ] Frequência = **250 Hz** (período 4 ms).
- [ ] Largura de pulso = valor comandado, com erro **< ~5 µs**.
- [ ] Sem glitches/duplos pulsos.

> Se o pulso divergir, revise `esc_us_to_duty()` em `esc_pwm.c` e a resolução do
> timer LEDC (16 bits).

---

## 10. Checklist final de aprovação (bancada)

- [ ] Boot completo, 4 ESCs armados, sem motor girando no arming.
- [ ] Web acessível; `/health` com todas as tarefas `true`.
- [ ] Cada motor mapeado para a posição física correta e sentido de giro certo.
- [ ] Deadbands medidos e calibração aplicada/persistida (NVS).
- [ ] MPU calibra e a atitude segue a inclinação no sentido correto; BMP pronto.
- [ ] `/axisCheck`: nivelado ≈ 0; inclinando, os motores do lado baixo aceleram
      (`motorDelta>0`) em todos os eixos (validação da convenção de frame).
- [ ] Malha estabilizada corrige inclinações na direção certa.
- [ ] Sob saturação (throttle alto + inclinação), `mixer.saturated:true`, coletivo
      reduzido (`collectiveAdjustUs<0`), roll/pitch preservados e yaw cedendo primeiro.
- [ ] `timing` saudável: `controlDtMs`≈20, `loopJitterMs`≈0, `repeatedImuSamples` estável.
- [ ] Todos os failsafes disparam e travam; `/stopAll` libera o rearme.
- [ ] Pulso do ESC validado no osciloscópio (250 Hz, larguras corretas).

Somente após **todos** os itens acima, considere testes com hélices em ambiente
controlado e contido.

---

## 11. Solução de problemas

| Sintoma | Possível causa / ação |
|--------|------------------------|
| ESC apita continuamente, motor não arma | Range do ESC não calibrado; ajuste `start/max` ou calibre o ESC ao range 1000–2000 µs |
| Motor gira ao contrário | Inverta 2 das 3 fases motor↔ESC |
| `MPU925x: nao encontrado` | Verifique fiação I²C (SDA21/SCL22), pull-ups, endereço 0x68/0x69 |
| Malha não arma (`MPU_NOT_CALIBRATED`) | Deixe a placa imóvel até `calibrationComplete:true` |
| `setFlight` estabilizado retorna 409 | Failsafe latched; execute `/stopAll` antes de reabilitar |
| `setFlight` retorna 503 | Tarefa de controle não criada; veja `[TASK] FlightControl` no boot |
| Página não carrega | Confirme conexão ao AP `EQUIPE4-AP` e IP 192.168.4.1; veja `/health` |
| Atitude no sentido invertido | Conferir `CONTROL_SWAP_ROLL_PITCH` e orientação física da IMU (config.h) |

---

## 12. Calibração de `hover_throttle` e sintonia do PI vertical

> ⚠️ A sintonia final exige **voo real com hélices** em área segura e contida.
> Só os passos 1 (estimador) são feitos sem hélices. `hover_throttle` e os ganhos
> PI **não podem ser calculados de bancada** — dependem de massa, bateria e hélice.

Os ganhos são ajustáveis em runtime e salvos na NVS:
```
# kp = us por (m/s) ; ki = us por (m/s)/s ; vmax = velocidade max do stick (m/s)
curl -s "http://192.168.4.1/setVerticalGains?kp=120&ki=60&vmax=1.5"
```
Acompanhe tudo por `http://192.168.4.1/flightStatus` → objeto `"vertical"`:
`velocityEstMs`, `vzSetpointMs`, `altitudeEstM`, `throttleUs`, `hoverUs`,
`saturated`, `kp`, `ki`, `vmaxMs`.

### Passo 1 — Validar o estimador (SEM hélices)
Segure o drone na mão e mova-o para cima/baixo:
- [ ] `velocityEstMs` fica **positivo ao subir** e negativo ao descer, e volta a
      ~0 quando parado.
- [ ] `altitudeEstM` acompanha o deslocamento e estabiliza parado.

Se o sinal estiver **invertido** ou divergir, **pare**: a remoção da gravidade
está errada (rever orientação da IMU). Não prossiga para voo.

### Passo 2 — Calibrar `hover_throttle` (em voo)
1. Mantenha o controle vertical **desligado** (`/setVerticalHold?enable=0`).
2. Decole e ajuste o throttle manualmente até o drone **pairar** (altitude estável).
3. Nesse instante, ligue: `/setVerticalHold?enable=1`. O firmware **captura o
   throttle atual como `hover_throttle`** (engate sem salto).
4. Confirme em `/flightStatus` → `vertical.hoverUs` (deve refletir o throttle de pairar).

### Passo 3 — Sintonizar o PI (em voo, controle vertical ligado)
Comece só com proporcional, depois adicione integral:
1. `setVerticalGains?kp=120&ki=0&vmax=1.0`. Com o stick no **centro** (manter 0 m/s),
   veja se a altitude se mantém. Aumente `kp` até a resposta ficar firme; se
   começar a **oscilar**, reduza `kp` ~20%.
2. Suba `ki` aos poucos (ex.: 40 → 60 → 80) para **eliminar a deriva lenta** de
   altitude. Pare antes de surgir oscilação lenta.
3. Teste **degraus**: comande subir/descer com o stick e verifique no
   `/flightStatus` que `velocityEstMs` segue `vzSetpointMs` sem overshoot grande.
4. Ajuste `vmax` ao conforto (1.0–1.5 m/s). Cada `setVerticalGains` **já salva** na NVS.

### Diagnóstico
- `saturated: true` frequente → reduza ganhos/`vmax` ou revise `hover_throttle`
  (se hover muito longe do real, o PI satura tentando compensar).
- `velocityEstMs` ruidoso → o estimador depende de accel/atitude; reduza vibração
  (balanceamento de hélices) antes de subir ganhos.

### Critérios de aprovação
- [ ] Com stick no centro, mantém altitude (deriva < ~0,2 m/s).
- [ ] Responde a comandos de subida/descida sem oscilar.
- [ ] Sem `saturated` constante em voo nivelado.

### Valores iniciais sugeridos
`kp` 100–150 · `ki` 40–80 · `vmax` 1,0–1,5 m/s (ponto de partida; ajuste em voo).
