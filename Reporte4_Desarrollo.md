# DESARROLLO DEL MINI-RETO

A continuación se describen los programas clave desarrollados y/o modificados para lograr el funcionamiento requerido por el mini-reto: implementar un controlador PI de lazo cerrado sobre un motor DC real de 12 V, utilizando una ESP32 con Micro-ROS como nodo de hardware y ROS 2 Humble en la computadora host.

A diferencia del mini-reto anterior, en este caso no se utilizó un motor simulado en software, sino un motor DC físico controlado a través de un driver L298N. La ESP32 se encarga de la interfaz de bajo nivel con el hardware (PWM, encoder, driver), mientras que el controlador PI se ejecuta en la computadora como nodo de ROS 2 independiente, comunicándose con la ESP32 mediante el agente de Micro-ROS por puerto serial.

---

## Arquitectura del Sistema

El sistema está compuesto por cuatro nodos. El nodo de hardware (`motor_node`) corre directamente en la ESP32 mediante Micro-ROS y se encarga de aplicar señales PWM al driver L298N, leer el encoder cuadrático e inferir la velocidad en RPM. Los demás nodos corren en la computadora host bajo ROS 2 Humble. El diagrama de nodos y tópicos es el siguiente:

```
/sine_wave_publisher  ──(/cmd_vel_rpm)──▶  /pi_velocity_controller
                                                     │
                                            (/cmd_pwm)│◀── (/motor/rpm)
                                                     ▼
                                            /motor_node  (ESP32)
                                                     │
                                        publishes: /motor/rpm
                                                   /motor/encoder
                                                   /motor/state
```

| Tópico | Tipo | Dirección | Descripción |
|---|---|---|---|
| `/cmd_vel_rpm` | `Float32` | sp_gen → controlador | Referencia de velocidad [RPM] |
| `/motor/rpm` | `Float32` | ESP32 → controlador | RPM medida con encoder |
| `/motor/encoder` | `Int32` | ESP32 → monitor | Cuenta total de pulsos del encoder |
| `/motor/state` | `Int16` | ESP32 → monitor | 0 = detenido, 1 = en marcha |
| `/cmd_pwm` | `Int16` | controlador → ESP32 | Ciclo de trabajo PWM (−255 a +255) |
| `/pi/error` | `Float32` | controlador → monitor | Error de velocidad [% de RPM_max] |
| `/pi/u_pct` | `Float32` | controlador → monitor | Esfuerzo de control [%] |
| `/pi/ref_rpm` | `Float32` | controlador → monitor | Set-point activo [RPM] |

---

## Nodo de Hardware: Micro-ROS en ESP32

La ESP32 ejecuta el nodo `motor_node` escrito en C++ (Arduino + biblioteca `micro_ros_arduino`). Este nodo implementa una máquina de estados para gestionar la conexión con el agente Micro-ROS, lo que garantiza reconexión automática si el cable serial se desconecta.

### Configuración de hardware

```cpp
// Motor Driver Pins (L298N)
#define IN1_GPIO 26      // Motor direction control 1
#define IN2_GPIO 25      // Motor direction control 2
#define PWM_GPIO 27      // PWM signal for speed control (ENA)

// Encoder Pins
#define PHASEA_GPIO 18   // Encoder phase A (rising edge interrupt)
#define PHASEB_GPIO 19   // Encoder phase B (direction detection)

// PWM Configuration
#define PWM_FREQUENCY 1000   // Hz
#define PWM_RESOLUTION 8     // 8 bits (0-255)

// Encoder Configuration
#define PULSES_PER_REV 495.0f    // Pulsos por revolución
#define SAMPLE_TIME_MS 100       // Periodo de muestreo [ms]
#define RPM_MAX 110.0f           // RPM máximas del motor
#define CMD_TIMEOUT_MS 500       // Tiempo de espera antes de apagar el motor
```

### Máquina de estados de conexión

El nodo implementa cuatro estados para garantizar operación robusta:

```cpp
enum states {
  WAITING_AGENT,        // Esperando conexión del agente ROS 2
  AGENT_AVAILABLE,      // Agente detectado
  AGENT_CONNECTED,      // Conectado exitosamente
  AGENT_DISCONNECTED    // Conexión perdida
} state;
```

El `loop()` principal evalúa el estado cada iteración y hace ping periódico al agente para detectar desconexiones:

```cpp
void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                      ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;  
      if (state == WAITING_AGENT) { destroy_entities(); }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                      ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        apply_motor_command();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      currentPwmCommand = 0;
      apply_motor_command();   // Detiene el motor al perder conexión
      state = WAITING_AGENT;
      break;
  }
}
```

### Lectura del encoder y cálculo de RPM

Se usa una interrupción en el flanco de subida de la fase A del encoder cuadrático. La dirección se determina leyendo el estado de la fase B en ese instante:

```cpp
void IRAM_ATTR isrEncoderA() {
  if (digitalRead(PHASEB_GPIO)) {
    encoderCountTotal++;
  } else {
    encoderCountTotal--;
  }
}
```

El cálculo de RPM se realiza en el callback del timer de control (cada 100 ms):

```cpp
void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  unsigned long now = millis();
  float deltaSec = (now - previousMillis) * 0.001f;

  // Timeout de seguridad: apagar si no llegan comandos
  if (lastCmdReceivedMs > 0 && (now - lastCmdReceivedMs) > CMD_TIMEOUT_MS) {
    currentPwmCommand = 0;
  }

  // Lectura atómica del encoder
  long encoderCountNow = 0;
  noInterrupts();
  encoderCountNow = encoderCountTotal;
  interrupts();

  // Cálculo de RPM
  long pulsesInInterval = encoderCountNow - previousEncoderCount;
  float rpm = 0.0f;
  if (deltaSec > 0.0f) {
    rpm = (pulsesInInterval * 60.0f) / (PULSES_PER_REV * deltaSec);
  }

  previousEncoderCount = encoderCountNow;
  previousMillis = now;

  // Publicar telemetría
  rpm_msg.data = rpm;
  encoder_msg.data = (int32_t)encoderCountNow;
  state_msg.data = (currentPwmCommand != 0) ? 1 : 0;

  rcl_publish(&rpm_publisher, &rpm_msg, NULL);
  rcl_publish(&encoder_publisher, &encoder_msg, NULL);
  rcl_publish(&state_publisher, &state_msg, NULL);
}
```

La fórmula de RPM utilizada es:

$$\text{RPM} = \frac{\Delta\text{pulsos} \times 60}{\text{PPR} \times \Delta t}$$

donde PPR = 495 pulsos/revolución y Δt es el intervalo de muestreo en segundos.

### Aplicación del comando al motor (L298N)

```cpp
void apply_motor_command() {
  int16_t cmd = currentPwmCommand;
  uint8_t pwmDuty = 0;

  if (cmd == 0) {
    // Freno: ambos pines HIGH activa el freno dinámico del L298N
    digitalWrite(IN1_GPIO, HIGH);
    digitalWrite(IN2_GPIO, HIGH);
    pwmDuty = 0;
  } else if (cmd > 0) {
    // Avance
    digitalWrite(IN1_GPIO, HIGH);
    digitalWrite(IN2_GPIO, LOW);
    pwmDuty = (uint8_t)constrain((int32_t)cmd, 0, 255);
  } else {
    // Reversa
    digitalWrite(IN1_GPIO, LOW);
    digitalWrite(IN2_GPIO, HIGH);
    pwmDuty = (uint8_t)constrain((int32_t)(-cmd), 0, 255);
  }
  ledcWrite(PWM_CHANNEL, pwmDuty);
}
```

---

## Nodo Generador de Referencia Senoidal

Para las pruebas del controlador se desarrolló el nodo `sine_wave_publisher`, que genera una señal senoidal de velocidad en RPM y la publica en `/cmd_vel_rpm`. Sus parámetros son configurables al momento del lanzamiento:

| Parámetro | Valor por defecto | Descripción |
|---|---|---|
| `amplitude` | 30.0 RPM | Amplitud de la senoide |
| `frequency` | 0.1 Hz | Frecuencia de la senoide |
| `offset` | 30.0 RPM | Offset DC (mantiene la señal positiva) |
| `publish_rate` | 100 Hz | Frecuencia de publicación |

La ecuación de la señal generada es:

$$r(t) = A \cdot \sin(2\pi f t) + \text{offset}$$

Con los valores por defecto: $r(t) = 30\sin(2\pi \cdot 0.1 \cdot t) + 30$ [RPM], lo que produce una señal que oscila entre 0 y 60 RPM.

```python
def timer_callback(self):
    elapsed_time = time.time() - self.start_time
    sine_value = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time) + self.offset
    msg = Float32()
    msg.data = sine_value
    self.publisher.publish(msg)
```

---

## Controlador PI de Velocidad

### Ecuación del controlador

Para este reto se implementó un controlador PI en **forma incremental** (también llamada forma velocidad). Esta formulación tiene la ventaja de que el anti-windup del integrador es inherente: el término integral se acumula en la variable de salida `u(k)`, de modo que saturar la salida limita automáticamente el crecimiento del integrador.

La ecuación de diferencias implementada es:

$$u[k] = u[k-1] + K_p\bigl(e[k] - e[k-1]\bigr) + K_i \cdot T_s \cdot e[k]$$

donde el error se expresa como porcentaje del RPM máximo:

$$e[k] = \frac{|r[k]|}{RPM_{max}} \times 100 - \frac{|y[k]|}{RPM_{max}} \times 100 \quad [\%]$$

La salida $u[k]$ representa el ciclo de trabajo del PWM en porcentaje (0–100%), que se convierte a entero de 8 bits (1–255) para enviarse a la ESP32 en el tópico `/cmd_pwm`.

### Conversión de esfuerzo a PWM

$$\text{PWM\_duty} = \text{round}\!\left(\frac{u[k] \times 255}{100}\right), \quad \text{PWM\_duty} \in [1, 255]$$

El signo del comando enviado a la ESP32 codifica la dirección: valor positivo → avance, valor negativo → reversa.

### Parámetros del controlador

Los valores de ganancia seleccionados tras el proceso de ajuste son:

$$K_p = 0.55 \qquad K_i = 2.0$$

Con un periodo de muestreo de $T_s = 0.1\,\text{s}$ (100 ms) y un RPM máximo de referencia de 110 RPM.

### Implementación del loop de control

```python
def _control_loop(self):
    ref    = self._ref_rpm
    actual = self._actual_rpm
    ts     = self.sample_time

    direction_forward = (ref >= 0.0)

    # Normalizar a porcentaje de RPM_max
    control_pct = self._clamp(abs(ref)    * 100.0 / self.rpm_max, 0.0, 105.0)
    rpm_pct     = self._clamp(abs(actual) * 100.0 / self.rpm_max, 0.0, 130.0)

    # Referencia cero cuando su magnitud es menor al 0.2 % del máximo
    center_stop = control_pct < 0.2

    # Resetear integrador al cambiar de dirección (anti-windup)
    if self._dir_prev is not None and direction_forward != self._dir_prev:
        self._u_prev = 0.0
        self._e_prev = 0.0
    self._dir_prev = direction_forward

    cmd = Int16()

    if center_stop:
        cmd.data     = 0
        self._u_prev = 0.0
        self._e_prev = 0.0
        e_pct = 0.0
        u_pct = 0.0
    else:
        # Forma incremental: u(k) = u(k-1) + Kp*(e(k)-e(k-1)) + Ki*Ts*e(k)
        e_pct = control_pct - rpm_pct
        u_pct = (self._u_prev
                 + self.kp * (e_pct - self._e_prev)
                 + self.ki * ts * e_pct)
        u_pct = self._clamp(u_pct, 0.0, 100.0)

        # Convertir porcentaje a PWM de 8 bits [1..255]
        pwm_duty = int(u_pct * 255.0 / 100.0 + 0.5)
        pwm_duty = int(self._clamp(float(pwm_duty), 1.0, 255.0))

        cmd.data     = pwm_duty if direction_forward else -pwm_duty
        self._u_prev = u_pct
        self._e_prev = e_pct

    self._cmd_pwm_pub.publish(cmd)
```

### Proceso de ajuste de ganancias (tuning)

El ajuste se realizó de manera manual e iterativa:

1. Se comenzó aumentando lentamente $K_p$ con $K_i = 0$, hasta que la velocidad medida siguiera razonablemente la forma de la senoide de referencia sin oscilar.
2. Con $K_p$ fijo, se aumentó $K_i$ para eliminar el error en estado estacionario en la zona de velocidad constante, sin que el sistema se volviera inestable.
3. No se incluyó acción derivativa dado que la señal de RPM proveniente del encoder presenta ruido inherente, y el término integral resultó suficiente para alcanzar los requisitos de desempeño.

Los valores finales seleccionados fueron:

$$K_p = 0.55 \qquad K_i = 2.0 \qquad K_d = 0$$

---

## Proceso de Lanzamiento del Sistema

Para ejecutar el sistema completo se requieren tres terminales en paralelo:

**Terminal 1 – Agente Micro-ROS (puente serial con la ESP32):**
```bash
source ~/Documents/classes/IRS_6to/ROS/TE3001B_Actividades_del_Reto/ros2_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

**Terminal 2 – Controlador PI y monitor:**
```bash
source ~/Documents/classes/IRS_6to/ROS/TE3001B_Actividades_del_Reto/ros2_ws/install/local_setup.bash
ros2 launch motor_control pi_controller.launch.py kp:=0.55 ki:=2.0
```

**Terminal 3 – Publicador de referencia senoidal:**
```bash
source ~/Documents/classes/IRS_6to/ROS/TE3001B_Actividades_del_Reto/ros2_ws/install/local_setup.bash
ros2 run motor_control sine_wave_publisher
```

Para monitorear el desempeño del controlador en tiempo real con `rqt_plot`:
```bash
# Gráfica 1: seguimiento (set-point vs RPM medida)
ros2 run rqt_plot rqt_plot /pi/ref_rpm/data /motor/rpm/data

# Gráfica 2: esfuerzo de control
ros2 run rqt_plot rqt_plot /pi/u_pct/data /cmd_pwm/data
```

---

## Resultados y Demostración

**Capturas de Pantalla.**

*[Insertar aquí captura de rqt_plot mostrando /pi/ref_rpm/data (azul) vs /motor/rpm/data (rojo)]*

*[Insertar aquí captura de rqt_graph mostrando los nodos /sine_wave_publisher, /pi_velocity_controller, /motor_node y /motor_monitor con sus tópicos]*

El controlador PI implementado en forma incremental logra que la velocidad medida del motor siga de manera satisfactoria la referencia senoidal generada. El offset de 30 RPM en la señal de referencia mantiene siempre una dirección de giro positiva, evitando cambios de dirección durante la prueba. La acción integral elimina el error en estado estacionario en los tramos de velocidad relativamente constante, mientras que en los picos de la senoide el seguimiento es cercano pero no exacto, debido al tiempo de establecimiento limitado por la frecuencia de la señal (0.1 Hz).

---

## Anexo — Código completo

### pi_velocity_controller.py

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16


class PIVelocityController(Node):
    """Closed-loop PI velocity controller for a DC motor."""

    def __init__(self):
        super().__init__('pi_velocity_controller')

        self.declare_parameter('kp',          0.55)
        self.declare_parameter('ki',          2.0)
        self.declare_parameter('rpm_max',   110.0)
        self.declare_parameter('sample_time', 0.1)

        self.kp          = self.get_parameter('kp').value
        self.ki          = self.get_parameter('ki').value
        self.rpm_max     = self.get_parameter('rpm_max').value
        self.sample_time = self.get_parameter('sample_time').value

        self._ref_rpm    = 0.0
        self._actual_rpm = 0.0
        self._u_prev     = 0.0
        self._e_prev     = 0.0
        self._dir_prev   = None

        self.create_subscription(Float32, 'motor/rpm',   self._rpm_callback, 10)
        self.create_subscription(Float32, 'cmd_vel_rpm', self._ref_callback, 10)

        self._cmd_pwm_pub = self.create_publisher(Int16,   'cmd_pwm',    10)
        self._error_pub   = self.create_publisher(Float32, 'pi/error',   10)
        self._u_pct_pub   = self.create_publisher(Float32, 'pi/u_pct',   10)
        self._ref_rpm_pub = self.create_publisher(Float32, 'pi/ref_rpm', 10)
        self._timer = self.create_timer(self.sample_time, self._control_loop)

        self.get_logger().info(
            f'PI Velocity Controller started.  '
            f'Kp={self.kp}  Ki={self.ki}  RPM_max={self.rpm_max}  '
            f'Ts={self.sample_time*1000:.0f} ms')

    def _rpm_callback(self, msg: Float32):
        self._actual_rpm = msg.data

    def _ref_callback(self, msg: Float32):
        self._ref_rpm = float(msg.data)

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def _control_loop(self):
        ref    = self._ref_rpm
        actual = self._actual_rpm
        ts     = self.sample_time

        direction_forward = (ref >= 0.0)
        control_pct = self._clamp(abs(ref)    * 100.0 / self.rpm_max, 0.0, 105.0)
        rpm_pct     = self._clamp(abs(actual) * 100.0 / self.rpm_max, 0.0, 130.0)

        center_stop = control_pct < 0.2

        if self._dir_prev is not None and direction_forward != self._dir_prev:
            self._u_prev = 0.0
            self._e_prev = 0.0
        self._dir_prev = direction_forward

        cmd = Int16()

        if center_stop:
            cmd.data     = 0
            self._u_prev = 0.0
            self._e_prev = 0.0
            e_pct = 0.0
            u_pct = 0.0
        else:
            e_pct = control_pct - rpm_pct
            u_pct = (self._u_prev
                     + self.kp * (e_pct - self._e_prev)
                     + self.ki * ts * e_pct)
            u_pct = self._clamp(u_pct, 0.0, 100.0)

            pwm_duty = int(u_pct * 255.0 / 100.0 + 0.5)
            pwm_duty = int(self._clamp(float(pwm_duty), 1.0, 255.0))

            cmd.data     = pwm_duty if direction_forward else -pwm_duty
            self._u_prev = u_pct
            self._e_prev = e_pct

        self._cmd_pwm_pub.publish(cmd)

        error_msg = Float32(); error_msg.data = float(e_pct)
        u_msg     = Float32(); u_msg.data     = float(u_pct)
        ref_msg   = Float32(); ref_msg.data   = float(ref)

        self._error_pub.publish(error_msg)
        self._u_pct_pub.publish(u_msg)
        self._ref_rpm_pub.publish(ref_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PIVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### sine_wave_publisher.py

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time


class SineWavePublisher(Node):
    def __init__(self):
        super().__init__('sine_wave_publisher')

        self.declare_parameter('amplitude',    30.0)
        self.declare_parameter('frequency',     0.1)
        self.declare_parameter('offset',       30.0)
        self.declare_parameter('publish_rate', 100)

        self.amplitude    = self.get_parameter('amplitude').value
        self.frequency    = self.get_parameter('frequency').value
        self.offset       = self.get_parameter('offset').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.publisher  = self.create_publisher(Float32, '/cmd_vel_rpm', 10)
        self.timer      = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.start_time = time.time()

    def timer_callback(self):
        elapsed_time = time.time() - self.start_time
        sine_value   = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time) + self.offset
        msg      = Float32()
        msg.data = sine_value
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SineWavePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### motor_control_uros.ino (fragmentos clave)

Para el código completo del nodo de la ESP32, ver el archivo:
`ros2_ws/src/motor_control/arduino/motor_control_uros/motor_control_uros.ino`
