# Modelagem, Equacionamento e Implementação da Controladora de Voo

Este documento descreve a **base matemática** da controladora de voo e **como
cada equação está implementada no firmware** (ESP32 / ESP-IDF). O objetivo é
servir de referência técnica: cada bloco apresenta o modelo, a forma discreta
efetivamente codificada, os parâmetros e o ponto exato do código onde reside.

> Convenção de notação: vetores e ângulos em negrito/itálico; subscritos `sp`
> indicam *setpoint* (valor desejado) e medições sem subscrito. As fórmulas
> refletem **o que o código faz**, não a teoria idealizada — quando há
> simplificação ou limite prático, ele está anotado.

Arquivos de referência:

| Bloco | Arquivo |
|-------|---------|
| Estimativa de atitude (IMU) | `main/mpu9259.c` |
| Estimativa de altitude (barômetro) | `main/bmp280.c` |
| Controladores PID, cascata e mixagem | `main/drone_pid.c` |
| Máquina de estados, coletivo, yaw híbrido | `main/flight_control.c` |
| Estimador e malha de velocidade vertical | `main/vertical_control.c` |
| Constantes (ganhos, limites, pinos) | `main/config.h` |

---

## 1. Convenções, referenciais e unidades

### 1.1 Referencial do corpo e configuração X

O quadricóptero usa configuração em **X**. Os quatro motores, na ordem usada
pela mixagem (`main/config.h`), são:

| Motor | Posição | Sentido de giro |
|-------|---------|-----------------|
| M1 | frente-esquerda | anti-horário |
| M2 | frente-direita  | horário |
| M3 | traseira-direita | anti-horário |
| M4 | traseira-esquerda | horário |

Eixos do corpo (convenção do firmware):

- **Roll** ($\phi$): rotação em torno do eixo longitudinal (frente). Positivo = inclina para a direita.
- **Pitch** ($\theta$): rotação em torno do eixo lateral. Positivo = nariz para cima.
- **Yaw** ($\psi$): rotação em torno do eixo vertical. Positivo = giro à direita.

### 1.2 Montagem física da IMU e remapeamento

A IMU está montada girada. Duas correções alinham os eixos do sensor aos eixos
do frame:

1. **Rotação de +90° em torno de Y** (`MPU_REMAP_ROTATE_Y_90`, função
   `apply_axis_remap` em `mpu9259.c`), aplicada **identicamente** ao acelerômetro
   e ao giroscópio:

$$
x' = z,\qquad y' = y,\qquad z' = -x
$$

   Com isso, "nivelado" passa a ler $\mathbf{a} \approx [0,\,0,\,+1]\,g$.

2. **Troca roll↔pitch** (`CONTROL_SWAP_ROLL_PITCH`, em
   `update_flight_state_from_mpu`, `flight_control.c`), feita já no domínio dos
   ângulos/taxas entregues ao controle:

$$
\phi \leftarrow \theta_\text{IMU},\quad \theta \leftarrow \phi_\text{IMU},\quad
\dot\phi \leftarrow \omega_{y},\quad \dot\theta \leftarrow \omega_{x}
$$

> ⚠️ Estes dois sinais dependem da montagem física. Antes de sintonizar ganhos,
> validar fisicamente (ver `TESTE_BANCADA.md`): inclinar a frente deve gerar
> pitch coerente; inclinar à direita, roll coerente.

### 1.3 Unidades e passo de tempo

- Ângulos em **graus**; taxas angulares em **graus/s (dps)**.
- Empuxo/coletivo em **microssegundos (µs)** de pulso (1000–2000 µs).
- Aceleração em **g** na IMU; velocidade/aceleração vertical em **m/s** e **m/s²**.
- O laço de controle roda a **50 Hz** (`FLIGHT_CONTROL_INTERVAL_MS = 20 ms`).
  A **IMU é amostrada a ~200 Hz** (`MPU_READ_INTERVAL_MS = 5 ms`), acima da taxa de
  controle, para que cada ciclo disponha de amostra fresca.
- O `dt` é **medido em tempo real**, não fixo: a fusão de atitude usa o intervalo
  entre amostras da IMU; a cascata usa `control_dt_from_mpu` (intervalo entre
  amostras processadas, ≈20 ms); o estimador vertical usa o período real do laço.
  Amostras repetidas da IMU são detectadas (`sample_seq`) e ignoradas.

---

## 2. Estimativa de atitude (fusão inercial) — `mpu9259.c`

A atitude $(\phi,\theta,\psi)$ é estimada por **filtro complementar adaptativo**:
o giroscópio domina o curto prazo (resposta rápida, sem ruído de vibração) e o
acelerômetro corrige a deriva no longo prazo (referência de gravidade). O yaw é
integrado do giroscópio e corrigido pelo magnetômetro.

### 2.1 Pré-filtragem do acelerômetro

**Filtro passa-baixa de 1ª ordem** com corte $f_c = 4\,\text{Hz}$
(`ACCEL_LPF_CUTOFF_HZ`), coeficiente dependente de `dt` (`low_pass_alpha`):

$$
\alpha = \frac{\Delta t}{RC + \Delta t},\qquad RC = \frac{1}{2\pi f_c}
$$

$$
\mathbf{a}_f \leftarrow \mathbf{a}_f + \alpha\,(\mathbf{a}_\text{lim} - \mathbf{a}_f)
$$

**Rejeição de picos:** antes do LPF, cada componente é limitada a um degrau
máximo de $0{,}35\,g$ (`MAX_ACCEL_STEP_G`) por amostra, descartando choques de
vibração:

$$
\mathbf{a}_\text{lim} = \text{clamp}\big(\mathbf{a}_\text{raw},\ \mathbf{a}_f - 0{,}35,\ \mathbf{a}_f + 0{,}35\big)
$$

### 2.2 Ângulos a partir do acelerômetro

Usando a aceleração filtrada normalizada $\hat{\mathbf{a}} = \mathbf{a}_f / \lVert \mathbf{a}_f \rVert$:

$$
\phi_\text{acc} = \operatorname{atan2}\!\big(\hat a_y,\ \hat a_z\big)
$$

$$
\theta_\text{acc} = \operatorname{atan2}\!\Big(-\hat a_x,\ \sqrt{\hat a_y^{\,2} + \hat a_z^{\,2}}\Big)
$$

O acelerômetro só é considerado confiável (`gravity_valid`) quando o módulo bruto
está na janela de gravidade e não houve pico:

$$
0{,}85\,g \le \lVert \mathbf{a}_\text{raw} \rVert \le 1{,}15\,g
$$

(`MIN_GRAVITY_G`, `MAX_GRAVITY_G`).

### 2.3 Integração do giroscópio e zona morta

O giroscópio passa por **zona morta** de $0{,}65$ dps (`remove_gyro_noise`,
`GYRO_NOISE_DEADBAND_DPS`) para não integrar ruído com o drone parado:

$$
\omega' = \begin{cases} 0, & |\omega| < 0{,}65\ \text{dps} \\ \omega, & \text{caso contrário} \end{cases}
$$

Predição por integração:

$$
\phi_\text{gyro} = \phi + \omega'_x\,\Delta t,\qquad
\theta_\text{gyro} = \theta + \omega'_y\,\Delta t
$$

### 2.4 Fusão complementar adaptativa

O peso do giroscópio $w$ é **adaptativo**: maior durante manobra (privilegia a
taxa), menor em repouso (deixa o acelerômetro corrigir mais rápido):

$$
w = \begin{cases}
0{,}995 & \text{se } |\omega'_x| \ge 4\ \text{ou}\ |\omega'_y| \ge 4\ \text{dps (manobra)} \\
0{,}980 & \text{caso contrário (estável)}
\end{cases}
$$

(`MOVING_GYRO_WEIGHT`, `STABLE_GYRO_WEIGHT`, `MOTION_RATE_THRESHOLD_DPS`).

Quando o acelerômetro é válido:

$$
\phi = w\,\phi_\text{gyro} + (1-w)\,\phi_\text{acc},\qquad
\theta = w\,\theta_\text{gyro} + (1-w)\,\theta_\text{acc}
$$

Sem gravidade confiável, mantém-se apenas a integração do giroscópio
($\phi=\phi_\text{gyro}$, $\theta=\theta_\text{gyro}$).

### 2.5 Guinada (yaw): integração + magnetômetro

Integração do giroscópio com normalização para $[-180°, 180°]$ (`wrap_angle_180`):

$$
\psi = \operatorname{wrap}_{180}\!\big(\psi + \omega'_z\,\Delta t\big)
$$

**Correção pelo magnetômetro com compensação de inclinação**
(`update_yaw_from_magnetometer`). Projeta-se o campo magnético no plano
horizontal usando $\phi,\theta$ atuais:

$$
h_x = m_x\cos\theta + m_z\sin\theta
$$

$$
h_y = m_x\sin\phi\sin\theta + m_y\cos\phi - m_z\sin\phi\cos\theta
$$

$$
\psi_\text{mag} = \operatorname{atan2}(h_y,\ h_x)
$$

A correção é aplicada suavemente (ganho $0{,}02$, `MAGNETOMETER_CORRECTION_WEIGHT`)
sobre o yaw integrado, limitando a deriva sem introduzir ruído:

$$
\psi \leftarrow \operatorname{wrap}_{180}\!\big(\psi + 0{,}02\cdot\operatorname{wrap}_{180}(\psi_\text{mag} - \psi)\big)
$$

### 2.6 Autocalibração

Com o drone parado, médias de N amostras estimam o **offset do giroscópio** (bias
por eixo) e a **escala do acelerômetro** ($1/\lVert \mathbf{a}\rVert$ médio). O
controle só engata após `calibration_complete = true`; caso contrário dispara o
failsafe `MPU_NOT_CALIBRATED`.

---

## 3. Estimativa de altitude — `bmp280.c` e `vertical_control.c`

### 3.1 Altitude barométrica

Fórmula barométrica internacional (`calculate_altitude`, `bmp280.c`):

$$
h = 44330\left[\,1 - \left(\frac{P}{P_0}\right)^{0{,}19029495}\right]\ \text{[m]},
\qquad 0{,}19029495 \approx \tfrac{1}{5{,}255}
$$

$P$ é a pressão medida e $P_0$ a referência. O firmware **aprende a referência**
$P_0$ pela média de N amostras na inicialização e reporta **altitude relativa**
$h_\text{rel} = h - h_\text{ref}$, filtrada. Só após estabilizar
(`reference_ready`) o dado é aceito como válido.

### 3.2 Estimador vertical inercial-barométrico

`vertical_estimator_update` funde aceleração vertical (inercial) com a altitude
do barômetro num **filtro complementar de 2ª ordem**.

**Projeção da força específica no eixo vertical do mundo** (Z para cima), a partir
da aceleração do corpo e dos ângulos:

$$
a_{z,\text{mundo}} = -\sin\theta\,a_x + \cos\theta\sin\phi\,a_y + \cos\theta\cos\phi\,a_z \quad [g]
$$

**Aceleração linear vertical** (remove 1 g e converte para m/s², $g=9{,}80665$):

$$
a_\text{vert} = (a_{z,\text{mundo}} - 1)\,g
$$

**Filtro complementar** (integra a aceleração, corrige com o barômetro). Sendo
$e = h_\text{baro} - \hat z$ o erro de altitude (zero se o baro não for válido —
*dead-reckoning*):

$$
\hat z \leftarrow \hat z + \big(\hat v + K_1\,e\big)\,\Delta t
$$

$$
\hat v \leftarrow \hat v + \big(a_\text{vert} + K_2\,e\big)\,\Delta t
$$

com $K_1 = 2{,}1$ (`VERT_EST_K1`) e $K_2 = 2{,}25$ (`VERT_EST_K2`). O estimador roda
**sempre** (para telemetria/validação), independentemente de a malha vertical
estar engatada.

---

## 4. Controlador PID base — `drone_pid.c`

Lei de controle contínua:

$$
u(t) = K_p\,e(t) + K_i\!\int_0^t e(\tau)\,d\tau + K_d\,\frac{de(t)}{dt}
$$

Forma discreta implementada em `pid_update` (passo $\Delta t$):

$$
e_k = \text{sp}_k - y_k
$$

$$
I_k = \text{clamp}\big(I_{k-1} + e_k\,\Delta t,\ -I_\text{lim},\ +I_\text{lim}\big) \quad\text{(anti-windup)}
$$

$$
D_k = \begin{cases} 0, & \text{primeira amostra} \\ \dfrac{e_k - e_{k-1}}{\Delta t}, & \text{caso contrário} \end{cases}
$$

$$
u_k = \text{clamp}\big(K_p e_k + K_i I_k + K_d D_k,\ u_\text{min},\ u_\text{max}\big)
$$

Características de robustez codificadas:

- **Anti-windup por saturação do integral** ($\pm I_\text{lim}$): impede acúmulo do
  termo integral além do útil.
- **Saturação da saída** ($[u_\text{min}, u_\text{max}]$): respeita a faixa física do atuador.
- **Derivada parte de zero** na primeira amostra (`has_previous_error`): evita
  pico de derivada no engate.
- **Passo inválido**: se $\Delta t \le 0$, retorna $u_k = 0$ (não atualiza estado).

---

## 5. Controle de atitude em cascata — `drone_pid.c` + `flight_control.c`

O modo estabilizado usa **cascata** (dois laços aninhados), mais robusta a
perturbações que um PID de ângulo puro, pois a malha interna atua sobre a **taxa
medida** pelo giroscópio.

### 5.1 Malha externa (atitude → taxa)

Termo proporcional converte erro de ângulo em **setpoint de velocidade angular**,
saturado a limites seguros (`attitude_rate_compute`):

$$
\dot\phi_\text{sp} = \text{clamp}\big(K_{p,\phi}(\phi_\text{sp}-\phi),\ \pm 180\big)\ \text{dps}
$$

$$
\dot\theta_\text{sp} = \text{clamp}\big(K_{p,\theta}(\theta_\text{sp}-\theta),\ \pm 180\big)\ \text{dps}
$$

$$
\dot\psi_\text{sp} = \text{clamp}\big(K_{p,\psi}\cdot\operatorname{wrap}_{180}(\psi_\text{sp}-\psi),\ \pm 120\big)\ \text{dps}
$$

(`MAX_ROLL_PITCH_RATE_DPS = 180`, `MAX_YAW_RATE_DPS = 120`). O **wrap** do erro de
yaw garante que a correção tome sempre o caminho angular mais curto.

### 5.2 Malha interna (taxa → correção em µs)

Cada eixo tem um PID que regula a taxa:

$$
\Delta_\text{eixo} = \text{PID}_\text{taxa}\big(\dot q_\text{sp},\ \dot q_\text{med}\big),\quad \text{eixo}\in\{\phi,\theta,\psi\}
$$

A saída $\Delta$ (em µs) alimenta a mixagem (Seção 6).

### 5.3 Ganhos default

| | Malha externa $K_p$ | Malha interna $K_p,K_i,K_d$ | Limite integral |
|---|---|---|---|
| Roll  | 4,0 | 1,20 / 0,02 / 0,015 | ±80 |
| Pitch | 4,0 | 1,20 / 0,02 / 0,015 | ±80 |
| Yaw   | 2,0 | 1,00 / 0,01 / 0,000 | ±60 |

Saída interna saturada em $\pm 220$ µs (`attitude_rate_init`). O eixo de yaw é
deliberadamente menos agressivo (menos crítico para estabilidade).

### 5.4 Modo de yaw híbrido (heading-hold ⟷ taxa) — `flight_control.c`

O yaw separa **duas físicas**: manter rumo absoluto quando não há comando, e
controlar taxa angular quando há comando de giro (`update_yaw_rate_command`).

O comando da interface é limitado, passa por zona morta e por *slew-rate*:

$$
c = \text{clamp}(\text{yawSp},\ \pm 90\ \text{dps}),\qquad c = 0\ \text{se}\ |c| < 2\ \text{dps}
$$

$$
\dot\psi_\text{ativo} \leftarrow \operatorname{slew}\big(\dot\psi_\text{ativo},\ c,\ 180\ \text{dps}^2,\ \Delta t\big)
$$

(`YAW_RATE_COMMAND_MAX_DPS = 90`, `YAW_RATE_DEADBAND_DPS = 2`,
`YAW_RATE_SLEW_DPS2 = 180`).

- **Com comando** ($\dot\psi_\text{ativo} \ne 0$): a cascata usa
  `attitude_rate_compute_yaw_rate`, injetando $\dot\psi_\text{sp} = \dot\psi_\text{ativo}$
  diretamente na malha interna (modo taxa). O setpoint de rumo acompanha o yaw
  medido para não saltar ao soltar o comando.
- **Sem comando** (retorno à zona morta): captura-se o **rumo atual**
  $\psi_\text{sp} \leftarrow \psi$ e volta-se ao **heading-hold** (Seção 5.1), sem
  salto de rumo.

A função `slew` limita a variação por passo a $\text{taxa}\cdot\Delta t$.

---

## 6. Mixagem em X — `drone_pid.c`

A saída por motor combina o coletivo $T$ com as correções $\Delta_\phi, \Delta_\theta,
\Delta_\psi$ conforme a posição e o sentido de giro (`quad_pid_mix_x`):

$$
\begin{aligned}
M_1 &= T + \Delta_\theta + \Delta_\phi - \Delta_\psi & \text{(frente-esq)}\\
M_2 &= T + \Delta_\theta - \Delta_\phi + \Delta_\psi & \text{(frente-dir)}\\
M_3 &= T - \Delta_\theta - \Delta_\phi - \Delta_\psi & \text{(tras-dir)}\\
M_4 &= T - \Delta_\theta + \Delta_\phi + \Delta_\psi & \text{(tras-esq)}
\end{aligned}
$$

Interpretação física:

- **Pitch** ($\Delta_\theta$): acelera a frente (M1,M2) e desacelera a traseira (M3,M4).
- **Roll** ($\Delta_\phi$): acelera a esquerda (M1,M4) e desacelera a direita (M2,M3).
- **Yaw** ($\Delta_\psi$): acelera o par horário (M2,M4) e desacelera o anti-horário
  (M1,M3), gerando torque de reação.

### 6.1 Desaturação priorizada (`quad_pid_mix_x_desaturated`)

Quando o comando não cabe em $[1000, 2000]\,\mu s$, em vez de saturar cada motor de
forma independente (o que distorce o torque resultante), o mixer aplica uma
desaturação com **prioridade roll/pitch > coletivo > yaw**:

1. **Limite físico de roll/pitch:** se o *span* diferencial de roll/pitch excede a
   faixa total, escala roll/pitch **juntos** (mantém a direção do vetor de torque):
   se $\;b_\text{max}-b_\text{min} > (\text{ESC}_\text{max}-\text{ESC}_\text{min})$,
   aplica fator $\;s = \dfrac{\text{ESC}_\text{max}-\text{ESC}_\text{min}}{b_\text{max}-b_\text{min}}$
   à parcela de roll/pitch em torno de $T$.
2. **Coletivo (air-mode):** desloca o empuxo comum $T$ para encaixar a parcela de
   roll/pitch na faixa, **sem** perder o diferencial (reportado em
   `collectiveAdjustUs`).
3. **Yaw por último:** adiciona o yaw escalado à folga restante de cada motor
   (`yaw_fit_scale`); se faltar faixa, o yaw é o primeiro a ceder.

O `clamp` final a $[1000, 2000]$ é apenas rede de segurança. A flag `mixerSaturated`
(em `/flightStatus`) sinaliza qualquer perda de autoridade. **No caso sem
saturação, a saída é idêntica à mixagem direta** acima.

---

## 7. Gestão do coletivo e sequência de decolagem — `flight_control.c`

O empuxo base não é aplicado diretamente: passa por `update_collective_throttle`,
que aplica *slew-rate* e governa a máquina de estados de voo, evitando degrau de
empuxo na transição.

Função de *slew* (limita a variação por passo):

$$
\operatorname{slew}(c, \text{alvo}, r, \Delta t):\quad \text{passo} = r\,\Delta t,\quad
c \leftarrow c + \text{clamp}(\text{alvo}-c,\ -\text{passo},\ +\text{passo})
$$

Lógica:

- $\text{alvo} \le 1050\,\mu s$ (`CONTROL_MIN_THROTTLE_US`): coletivo $= 1000$, fase = `IDLE`.
- Ao sair de `IDLE`/`DISARMED`: coletivo inicia em $1050$ e entra em `TAKEOFF_RAMP`.
- Taxa de subida: $180\ \mu s/s$ em `TAKEOFF_RAMP` (`TAKEOFF_RAMP_US_PER_S`),
  $260\ \mu s/s$ em voo normal (`THROTTLE_SLEW_US_PER_S`).

**Transições de fase** (`flight_phase_t`):

```
DISARMED → ARMING → IDLE → TAKEOFF_RAMP → HOVER ⇄ FLIGHT
                                  ↘ (qualquer condição perigosa) → FAILSAFE
```

A condição "tipo hover" (`setpoint_is_hover_like`) é satisfeita quando:

$$
|\phi_\text{sp}| \le 2°\ \wedge\ |\theta_\text{sp}| \le 2°\ \wedge\
|T - 1500| \le 80\,\mu s\ \wedge\ |\dot\psi_\text{ativo}| \le 2\ \text{dps}
$$

(`LEVEL_SETPOINT_BAND_DEG = 2`, `HOVER_STICK_BAND_US = 80`,
`VERT_HOVER_THROTTLE_US_DEFAULT = 1500`).

---

## 8. Controle de velocidade vertical (climb-rate hold) — `vertical_control.c`

Funcionalidade **opcional, desligada por padrão**. Quando engatada (e com a malha
estabilizada ativa), o stick de throttle deixa de comandar empuxo direto e passa
a comandar **velocidade vertical**.

### 8.1 Mapeamento stick → velocidade

Com zona morta entre 1450 e 1550 µs (`vertical_throttle_to_setpoint`):

$$
v_{z,\text{sp}} = \begin{cases}
\dfrac{T - 1550}{2000 - 1550}\,v_\text{max}, & T > 1550 \\[2ex]
\dfrac{T - 1450}{1450 - 1000}\,v_\text{max}, & T < 1450 \\[2ex]
0, & 1450 \le T \le 1550
\end{cases}
$$

saturado a $\pm v_\text{max}$ ($v_\text{max} = 2{,}0$ m/s, `VERT_MAX_VELOCITY_MS`).

### 8.2 Malha PI de velocidade com compensação de inclinação

Erro e controlador (`vertical_velocity_hold`), com $\hat v$ vindo do estimador
(Seção 3.2):

$$
e = v_{z,\text{sp}} - \hat v
$$

$$
\text{ajuste} = \text{clamp}\big(K_p\,e + I,\ \pm 400\big)\,\mu s,\qquad
T_\text{base} = T_\text{hover} + \text{ajuste}
$$

($K_p = 120$, $K_i = 60$ µs por m/s; `VERT_VEL_OUT_LIMIT_US = 400`).

**Compensação de inclinação** (mantém o empuxo vertical quando o drone inclina):

$$
\kappa = \max(\cos\phi\cos\theta,\ 0{,}5),\qquad
T = 1000 + \frac{T_\text{base} - 1000}{\kappa}
$$

(piso $0{,}5$ = `VERT_TILT_COMP_MIN_COS`, evita ganho excessivo perto de 90°), com
saturação final a $[1000, 2000]\,\mu s$.

**Anti-windup condicional**: o integral só acumula se a saída **não** está saturada
e o ajuste não está pressionando o limite:

$$
I \leftarrow \text{clamp}\big(I + K_i\,e\,\Delta t,\ \pm 400\big)\quad
\text{somente se não saturado}
$$

### 8.3 Engate *bumpless* e trava do barômetro

- Ao engatar, $T_\text{hover}$ recebe o empuxo corrente e $I=0$, de modo que a saída
  inicial iguala o empuxo aplicado (sem degrau, `vertical_control_engage`).
- **Trava de segurança** (`baro_guard`, ligada por padrão): a malha só atua com a
  referência do barômetro pronta e recente (idade $< 1000$ ms,
  `VERT_BARO_MAX_AGE_MS`); caso contrário desengata e devolve o throttle ao
  operador. O re-engate é *bumpless*.

---

## 9. Segurança, *failsafes* e temporização — `flight_control.c`

Toda iteração do laço (50 Hz) verifica, em ordem, condições de segurança. Qualquer
condição perigosa desabilita o controle, leva os motores ao mínimo e **trava o
rearme** (*latch*) até o operador comandar PARAR TUDO.

| Condição | Limite / parâmetro | Failsafe |
|----------|--------------------|----------|
| Perda de comando | $> 600$ ms (`FLIGHT_COMMAND_TIMEOUT_MS`) | `COMMAND_TIMEOUT` |
| Amostra de IMU velha | $> 150$ ms (`MPU_MAX_AGE_MS`) | `MPU_INVALID` |
| IMU não calibrada | `calibration_complete = false` | `MPU_NOT_CALIBRATED` |
| Inclinação excessiva | $\|\phi\| > 65°$ ou $\|\theta\| > 65°$ (`MAX_SAFE_TILT_DEG`) | `EXCESSIVE_TILT` |
| Contenção do controlador | mutex da cascata não obtido no prazo (5 ms) | `CONTROLLER_TIMEOUT` |
| Override manual / parada | comando do operador | `MANUAL_OVERRIDE` / `EMERGENCY_STOP` |

Abaixo do throttle mínimo de controle (1050 µs) a malha mantém os motores no
mínimo e reinicia a cascata (sem corrigir atitude sem empuxo).

Amostras repetidas da IMU (sem dado novo) são ignoradas pelo laço; se persistirem
além de `MPU_MAX_AGE_MS`, caem em `MPU_INVALID`. Diagnósticos de tempo real
(`controlDtMs`, `loopJitterMs`, `imuAgeMs`, `repeatedImuSamples`) e de mixer
(`mixerSaturated`, `collectiveAdjustUs`) são expostos em `/flightStatus`.

---

## 10. Tabela de símbolos e constantes principais

| Símbolo | Significado | Constante / valor |
|---------|-------------|-------------------|
| $\phi,\theta,\psi$ | roll, pitch, yaw (graus) | — |
| $\dot q$ | velocidade angular (dps) | — |
| $T$ | coletivo / empuxo base (µs) | 1000–2000 |
| $\Delta_\text{eixo}$ | correção por eixo (µs) | $\pm 220$ |
| $f_c$ | corte do LPF do accel | 4 Hz |
| $w$ | peso do giroscópio na fusão | 0,98 / 0,995 |
| $K_1, K_2$ | ganhos do estimador vertical | 2,1 / 2,25 |
| limite de taxa roll/pitch | malha externa | $\pm 180$ dps |
| limite de taxa yaw | malha externa | $\pm 120$ dps |
| comando de yaw | modo taxa | $\pm 90$ dps |
| $g$ | gravidade | 9,80665 m/s² |
| período do laço | controle | 20 ms (50 Hz) |
| taxa da IMU | leitura/fusão | ~200 Hz (5 ms) |

---

## 11. Pipeline por ciclo de controle (resumo)

```
[Telemetria]  sensor_hub_update  (IMU ~200 Hz, BMP ~5 Hz)
   └─ IMU: remap → LPF accel → ângulos accel → integra giro → fusão → yaw+mag
   └─ Barômetro: pressão → altitude relativa filtrada

[Controle, 50 Hz]  flight_control_task
   1. Estimador vertical (sempre): a_vert → filtro complementar (z, v)
   2. Lê snapshot IMU; rejeita amostra repetida/velha; aplica remap roll↔pitch
   3. Verifica failsafes (timeout, IMU, calibração, inclinação, mutex)
   4. Yaw híbrido: deadband + slew → modo taxa ou heading-hold
   5. Cascata: ângulo→taxa (P externo) → taxa→µs (PID interno), com dt real
   6. Coletivo: slew/rampa + máquina de estados (ou malha vertical se engatada)
   7. Mixagem X com desaturação priorizada (roll/pitch > coletivo > yaw) → ESCs
```

---

> A convenção de eixos (§1.1–1.2) pode ser verificada sem hélices pela rota
> read-only `/axisCheck` (autoteste de eixos): inclinar o frame e conferir que os
> motores do lado baixo aceleram. Procedimento em
> [`TESTE_BANCADA.md`](TESTE_BANCADA.md) §6.1.
>
> Para o procedimento de validação em bancada (sem hélices) destes blocos, ver
> [`TESTE_BANCADA.md`](TESTE_BANCADA.md). Para a referência de API gerada do
> código-fonte, ver a documentação Doxygen (`doxygen Doxyfile`).
