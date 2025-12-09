# 📚 PROYECTO FINAL - ELEMENTOS PROGRAMABLES II
> El programa implementa una plataforma *Stewart* controlada por gestos de la mano mediante *visión por computadora. Utiliza **MediaPipe* para detectar los landmarks de la mano (muñeca, dedo medio y pulgar) y calcula dos parámetros principales: el *pitch (inclinación arriba/abajo)* basado en la profundidad Z entre muñeca y dedo medio, y el *roll (inclinación izquierda/derecha)* determinado por la posición vertical del pulgar. Estos valores se procesan con un filtro exponencial para eliminar tembladera y se envían vía *Bluetooth Classic a un ESP32, que controla 3 servomotores **MG90S* en configuración triangular (pines 4, 5 y 15). El ESP32 recibe comandos en formato *A1:x,A2:y,A3:z* y genera señales *PWM a 50Hz* con resolución de *12 bits* para posicionar cada servo. Refuerza conceptos de procesamiento de imagen en tiempo real, comunicación serial inalámbrica, control de actuadores y cinemática inversa básica para plataformas de movimiento.
---

## 1) Resumén  📌

- *Equipo / Autor(es):* Emmanuel Angeles Barreto
- *Curso / Asignatura:* Elementos Programables II  
- *Fecha:* 07/12/25  

---

## 2) Objetivos

- *General:* _ Desarrollar un sistema de control para una plataforma Stewart de 3 grados de libertad mediante el reconocimiento de gestos de la mano utilizando visión por computadora y comunicación inalámbrica Bluetooth._
- *Específicos:*
  - OP1… Implementar la detección de la mano en tiempo real utilizando la librería MediaPipe para obtener los landmarks de muñeca, dedo medio y pulgar
  - OP2… Calcular los parámetros de inclinación (pitch y roll) a partir de las posiciones relativas de los puntos de la mano y aplicar filtros para suavizar el movimiento.
  - OP3… Establecer comunicación Bluetooth entre Python y el ESP32 para transmitir los ángulos calculados de forma inalámbrica.
  - OP4… Controlar 3 servomotores MG90S mediante señales PWM generadas por el ESP32 para inclinar la plataforma según los gestos detectados.

---
## 2) Instalación

bash

# 1) Seguir el siguiente link, para poder realizar el código:
[doc-ref]: https://iberopuebla.sharepoint.com/:p:/s/Section_11192A-O25/Eagdi_tzhWZMgKc8luT4Fi0BwHxjPm1VrXFUaZsVAG4fsw?e=aVqLTr "Instalación pybluez2"


---
## 2) Requisitos

*HARDWARE*
- ESP32-D
- SERVOMOTORES MG90S
- PROTOBOARD
- CABLES DUPONT (macho-macho, macho-hembra)
- FUENTE DE ALIMENTACIÓN (6v para los servos)
- CÁMARA WEB O CAMARA INTEGRADA DE LAPTOP
- ESTRUCTURA DE PLATAFORMA STEWART IMPRESA EN 3D

*SOFTWARE)*
- ARDUINO IDE (para programar ESP32-D)
- VISUAL STUDIO CODE (python)
- LIBRERIAS DE PYTHON

*CONOCIMIENTOS PREVIOS*
- PROGRAMACIÓN EN Python
- PROGRAMACIÓN EN C++(arduino)
- COMUNICACIÓN SERIAL Y BLUETOOTH
- CONTROL DE SERVOMOTORES CON PWM
- _ELECTRÓNICA BÁSICA _


---
## 2) PIEZAS A UTILIZAR ⌨

<img src="../recursos/imgs/ibero.jpeg" alt="ESTRUCTURA PIRNCIPAL DE LA BASE" width="420">

---
## 3) Código ⌨️
## CÓDIGO PYTHON (Visual Studio)
bash
# ============================================================================
# PLATAFORMA STEWART - CONTROL POR GESTOS DE MANO
# ============================================================================
# Este programa utiliza visión por computadora para detectar la posición
# y orientación de la mano del usuario, calculando ángulos que se envían
# a un ESP32 vía Bluetooth para controlar 3 servomotores en una plataforma
# Stewart de 3 grados de libertad.
# ============================================================================

# ----- IMPORTACIÓN DE LIBRERÍAS -----
import cv2              # OpenCV: procesamiento de imágenes y captura de video
import mediapipe as mp  # MediaPipe: detección de landmarks de la mano
import time             # Control de tiempos y delays
import bluetooth        # PyBluez: comunicación Bluetooth con ESP32

# ============================================================================
# CONFIGURACIÓN DE CONEXIÓN BLUETOOTH
# ============================================================================
# El ESP32 actúa como servidor Bluetooth Serial (SPP - Serial Port Profile)
# La comunicación se realiza en el puerto 1, que es el estándar para SPP

PORT = 1                          # Puerto Bluetooth estándar para SPP
ESP32_MAC = "14:33:5C:02:4D:2A"   # Dirección MAC del ESP32 (CAMBIAR según tu dispositivo)

# Creación del socket Bluetooth tipo RFCOMM (comunicación serial)
sock = bluetooth.BluetoothSocket()
sock.settimeout(20)  # Timeout de 20 segundos para evitar bloqueos indefinidos

# ----- BUCLE DE CONEXIÓN -----
# Intenta conectar continuamente hasta lograr la conexión
# Esto es útil cuando el ESP32 tarda en estar disponible
print("Intentando conectar al ESP32...")
while True:
    try:
        sock.connect((ESP32_MAC, PORT))  # Intenta establecer conexión
        print("¡Conectado al ESP32!")
        break  # Sale del bucle si la conexión es exitosa
    except Exception as e:
        print("Error en conexión... reintentando:", e)
        time.sleep(1)  # Espera 1 segundo antes de reintentar

# ----- FUNCIÓN DE ENVÍO DE DATOS -----
def send_bt(message: str):
    """
    Envía un mensaje codificado en UTF-8 al ESP32 vía Bluetooth.
    
    Parámetros:
        message (str): Cadena de texto a enviar (ej: "ANG:90,90,90\n")
    
    El mensaje se codifica a bytes antes de enviarse ya que Bluetooth
    trabaja con datos binarios, no strings directamente.
    """
    try:
        sock.send(message.encode())       # Codifica string a bytes y envía
        print("Enviado:", message.strip()) # Muestra en consola (sin salto de línea)
    except:
        print("Error enviando datos")      # Manejo básico de errores

# ============================================================================
# CONFIGURACIÓN DE MEDIAPIPE HANDS
# ============================================================================
# MediaPipe Hands es un modelo de ML que detecta 21 puntos (landmarks)
# en la mano, permitiendo rastrear su posición y orientación en tiempo real

mp_hands = mp.solutions.hands  # Módulo de detección de manos

# Inicialización del detector de manos con parámetros optimizados
hands = mp_hands.Hands(
    max_num_hands=1,              # Solo detectar 1 mano (mejor rendimiento)
    min_detection_confidence=0.6, # Confianza mínima para detectar mano (60%)
    min_tracking_confidence=0.5   # Confianza mínima para rastrear (50%)
)

# Utilidades para dibujar los landmarks en la imagen
mp_draw = mp.solutions.drawing_utils

# ----- CAPTURA DE VIDEO -----
cap = cv2.VideoCapture(0)  # Abre la cámara por defecto (índice 0)

# ============================================================================
# VARIABLES DE FILTRADO Y CONTROL
# ============================================================================
# Los filtros suavizan el movimiento, eliminando tembladera y ruido
# Se utiliza un filtro exponencial de primer orden (EMA)

pitch_filtrado = 0  # Valor filtrado de inclinación arriba/abajo
roll_filtrado = 0   # Valor filtrado de inclinación izquierda/derecha
alpha = 0.25        # Factor de suavizado (0.1=muy suave, 1.0=sin filtro)

# Control de frecuencia de envío
ultimo_envio = time.time()  # Timestamp del último envío
intervalo_envio = 0.05      # Enviar cada 50ms (20 Hz)

# ============================================================================
# POSICIONES HOME (NEUTRAS)
# ============================================================================
# Cuando la mano está en posición neutral, los servos van a estos ángulos
# 90° es el centro del rango de movimiento de un servo (0°-180°)

HOME_IZQ = 90     # Posición home del servo izquierdo (Pin 15)
HOME_ARRIBA = 90  # Posición home del servo central (Pin 33)
HOME_DER = 90     # Posición home del servo derecho (Pin 25)

# ============================================================================
# GANANCIAS DE CONTROL
# ============================================================================
# Estas constantes determinan qué tan sensible es cada movimiento
# Valores más altos = movimientos más amplios de los servos

K_pitch = 30.0       # Sensibilidad para inclinación arriba/abajo
K_roll = 0.05        # Sensibilidad para detección de roll (pulgar)
K_lat = 85.0         # Sensibilidad para movimiento lateral (izq/der)
K_mid_acompa = 20.0  # Cuánto acompaña el servo central al movimiento lateral

# ============================================================================
# BUCLE PRINCIPAL DE PROCESAMIENTO
# ============================================================================
# Este bucle se ejecuta continuamente mientras la cámara esté abierta,
# capturando frames, procesándolos y enviando comandos al ESP32

while cap.isOpened():
    # ----- CAPTURA DE FRAME -----
    ret, img = cap.read()  # Lee un frame de la cámara
    if not ret:
        break  # Sale si no puede leer (cámara desconectada)

    # Espeja la imagen horizontalmente para efecto espejo natural
    # Esto hace que el movimiento de la mano corresponda intuitivamente
    img = cv2.flip(img, 1)
    
    # Convierte de BGR (formato OpenCV) a RGB (formato MediaPipe)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    # ----- DETECCIÓN DE MANO -----
    results = hands.process(rgb)  # Procesa el frame con MediaPipe
    
    h, w, _ = img.shape  # Obtiene dimensiones del frame (alto, ancho)
    detectada = False    # Flag para saber si se detectó mano

    # ----- PROCESAMIENTO SI HAY MANO DETECTADA -----
    if results.multi_hand_landmarks:
        for hand in results.multi_hand_landmarks:
            detectada = True
            
            # Dibuja los 21 landmarks y sus conexiones en la imagen
            mp_draw.draw_landmarks(img, hand, mp_hands.HAND_CONNECTIONS)

            # ========== EXTRACCIÓN DE LANDMARKS CLAVE ==========
            # MediaPipe detecta 21 puntos en la mano, numerados del 0 al 20
            # Usamos solo 3 puntos estratégicos:
            
            muneca = hand.landmark[0]   # Punto 0: Muñeca (base de la mano)
            medio = hand.landmark[12]   # Punto 12: Punta del dedo medio
            pulgar = hand.landmark[4]   # Punto 4: Punta del pulgar

            # Convierte coordenadas normalizadas (0-1) a píxeles
            # Las coordenadas de MediaPipe están normalizadas al tamaño de imagen
            wx, wy = int(muneca.x * w), int(muneca.y * h)  # Muñeca en píxeles
            mx, my = int(medio.x * w), int(medio.y * h)    # Dedo medio en píxeles
            px, py = int(pulgar.x * w), int(pulgar.y * h)  # Pulgar en píxeles

            # Dibuja círculos en los puntos clave para visualización
            cv2.circle(img, (wx, wy), 10, (255, 0, 0), -1)  # Muñeca: Azul
            cv2.circle(img, (mx, my), 10, (0, 255, 0), -1)  # Medio: Verde
            cv2.circle(img, (px, py), 10, (0, 0, 255), -1)  # Pulgar: Rojo

            # ========== CÁLCULO DE PITCH (ARRIBA/ABAJO) ==========
            # El pitch se calcula usando la coordenada Z (profundidad)
            # Cuando la mano se inclina hacia arriba, el dedo medio se acerca
            # a la cámara (Z menor) respecto a la muñeca
            
            pitch = (muneca.z - medio.z) * 1.8  # Diferencia de profundidad escalada
            pitch = max(-1, min(1, pitch))       # Limita al rango [-1, 1]

            # ========== CÁLCULO DE ROLL (IZQUIERDA/DERECHA) ==========
            # El roll se detecta por la posición vertical del pulgar
            # Pulgar ARRIBA (py < wy) = Roll positivo = Mover a DERECHA
            # Pulgar ABAJO (py > wy) = Roll negativo = Mover a IZQUIERDA
            
            dy = wy - py              # Diferencia vertical (muñeca - pulgar)
            roll = dy * K_roll        # Escala la diferencia
            roll = max(-1, min(1, roll))  # Limita al rango [-1, 1]

            # ========== APLICACIÓN DE FILTRO EXPONENCIAL ==========
            # El filtro EMA (Exponential Moving Average) suaviza los valores
            # Fórmula: valor_nuevo = (1-alpha)*valor_anterior + alpha*valor_actual
            # Alpha bajo = más suave pero más lento
            # Alpha alto = más rápido pero más ruidoso
            
            pitch_filtrado = (1 - alpha) * pitch_filtrado + alpha * pitch
            roll_filtrado = (1 - alpha) * roll_filtrado + alpha * roll

            # Zona muerta: ignora valores muy pequeños para evitar temblor
            if abs(pitch_filtrado) < 0.05:
                pitch_filtrado = 0
            if abs(roll_filtrado) < 0.05:
                roll_filtrado = 0

            # ========== CÁLCULO DE ÁNGULOS DE SERVOS ==========
            # Cada servo se calcula basándose en los valores filtrados
            # La lógica implementa el comportamiento de la plataforma Stewart
            
            # ----- SERVO CENTRAL (ARRIBA - Pin 33) -----
            # Sube con pitch positivo (mano hacia arriba)
            # También sube ligeramente cuando hay movimiento lateral
            a_arriba = HOME_ARRIBA + K_pitch * pitch_filtrado + K_mid_acompa * abs(roll_filtrado)

            # ----- SERVOS LATERALES (IZQ Pin 15, DER Pin 25) -----
            # Calculamos el delta lateral basado en el roll
            delta_lat = K_lat * roll_filtrado
            
            # Izquierdo y derecho se mueven en direcciones opuestas
            # Roll positivo (pulgar arriba/derecha): izq baja, der sube
            # Roll negativo (pulgar abajo/izquierda): izq sube, der baja
            a_izq = HOME_IZQ - delta_lat
            a_der = HOME_DER + delta_lat

            # Acompañamiento de pitch en servos laterales
            # Ambos bajan cuando la mano sube (pitch positivo)
            a_izq += (K_pitch * 0.25) * pitch_filtrado
            a_der += (K_pitch * 0.25) * pitch_filtrado

            # Limita los ángulos al rango válido de los servos (0°-180°)
            a_izq = int(max(0, min(180, a_izq)))
            a_arriba = int(max(0, min(180, a_arriba)))
            a_der = int(max(0, min(180, a_der)))

            # ========== ENVÍO DE DATOS AL ESP32 ==========
            # Solo envía si ha pasado suficiente tiempo desde el último envío
            # Esto evita saturar la comunicación Bluetooth
            
            if time.time() - ultimo_envio >= intervalo_envio:
                # Formato del mensaje: "ANG:izq,arriba,der\n"
                msg = f"ANG:{a_izq},{a_arriba},{a_der}\n"
                send_bt(msg)
                ultimo_envio = time.time()

    # ----- MENSAJE CUANDO NO SE DETECTA MANO -----
    if not detectada:
        cv2.putText(img, "No se detecta mano", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # ----- MOSTRAR IMAGEN EN VENTANA -----
    cv2.imshow("STEWART CONTROL PULGAR", img)

    # ----- CONTROL DE TECLADO -----
    k = cv2.waitKey(1)  # Espera 1ms por tecla
    
    if k == ord('q'):      # Tecla 'q': Salir del programa
        break
    if k == ord('z') or k == ord('c'):  # Tecla 'z' o 'c': Enviar comando ZERO
        send_bt("ZERO\n")   # Lleva todos los servos a posición home

# ============================================================================
# LIMPIEZA Y CIERRE
# ============================================================================
# Libera todos los recursos antes de terminar

sock.close()              # Cierra la conexión Bluetooth
cap.release()             # Libera la cámara
cv2.destroyAllWindows()   # Cierra todas las ventanas de OpenCV
print("Programa terminado")


## CÓDIGO C++ (ARDUINO)
bash
// ============================================================================
// PLATAFORMA STEWART - FIRMWARE ESP32
// ============================================================================

#include <Arduino.h>           // Librería base de Arduino
#include "BluetoothSerial.h"   // Librería para Bluetooth Classic (SPP)

// Instancia del objeto Bluetooth Serial
BluetoothSerial SerialBT;

// Almacena caracteres recibidos hasta formar un comando completo (terminado en '\n')
String btBuffer;

#define SERVO_IZQ    15   // GPIO 15: Servo izquierdo
#define SERVO_ARRIBA 33   // GPIO 33: Servo central/vertical
#define SERVO_DER    25   // GPIO 25: Servo derecho

// ============================================================================
// ÁNGULOS HOME (POSICIÓN NEUTRAL)
// ============================================================================
// Cuando no hay comandos o se pierde la mano, los servos van a estos ángulos
// 90° representa el centro del rango de movimiento

const int HOME_IZQ    = 90;   // Home servo izquierdo
const int HOME_ARRIBA = 90;   // Home servo central
const int HOME_DER    = 90;   // Home servo derecho

// ============================================================================
// CONFIGURACIÓN PWM PARA SERVOS
// ============================================================================
// Los servos MG90S operan con señales PWM de 50Hz
// El ancho del pulso determina la posición:
// - ~1ms (5%) = 0°
// - ~1.5ms (7.5%) = 90°
// - ~2ms (10%) = 180°

const uint32_t FREQ_HZ   = 50;    // Frecuencia PWM: 50 Hz (período de 20ms)
const uint8_t  RES_BITS  = 12;    // Resolución: 12 bits (0-4095)
const uint16_t DUTY_MIN  = 205;   // Duty cycle para 0° (~1.0 ms)
const uint16_t DUTY_MAX  = 410;   // Duty cycle para 180° (~2.0 ms)

// ============================================================================
// FUNCIONES DE CONVERSIÓN DE ÁNGULOS
// ============================================================================

/**
 * Convierte grados (0-180) a valor de duty cycle para PWM
 * 
 * @param deg Ángulo en grados (0-180)
 * @return Valor de duty cycle (205-410)
 * 
 * La función map() hace una interpolación lineal:
 * 0° -> 205, 180° -> 410
 */
uint16_t dutyFromDeg(int deg) {
  deg = constrain(deg, 0, 180);                    // Limita al rango válido
  return map(deg, 0, 180, DUTY_MIN, DUTY_MAX);     // Mapea a duty cycle
}

/**
 * Convierte ángulo lógico a físico (inversión)
 * 
 * @param logicalDeg Ángulo lógico (0-180)
 * @return Ángulo físico invertido (180-0)
 * 
 * Esto es necesario porque los servos pueden estar montados
 * en dirección opuesta a la esperada. Invertir el ángulo
 * hace que "subir" en el código signifique "subir" físicamente.
 */
int logicalToPhysical(int logicalDeg) {
  logicalDeg = constrain(logicalDeg, 0, 180);
  return 180 - logicalDeg;  // Inversión: 0->180, 90->90, 180->0
}

/**
 * Escribe un ángulo lógico a un servo
 * 
 * @param pin Pin GPIO del servo
 * @param logicalDeg Ángulo lógico deseado (0-180)
 * 
 * Convierte el ángulo lógico a físico y luego a duty cycle
 * antes de escribir al pin PWM.
 */
void writeServoLogical(int pin, int logicalDeg) {
  int fisico = logicalToPhysical(logicalDeg);  // Convierte lógico a físico
  ledcWrite(pin, dutyFromDeg(fisico));         // Escribe duty cycle al PWM
}

/**
 * Configura un pin GPIO como salida PWM para servo
 * 
 * @param pin Pin GPIO a configurar
 * @param initialLogical Ángulo inicial en grados lógicos
 * 
 * En ESP32 Arduino Core 3.x, el pin actúa como identificador del canal.
 * ledcAttach() configura automáticamente el canal PWM.
 */
void configServo(int pin, int initialLogical) {
  pinMode(pin, OUTPUT);                        // Configura pin como salida
  ledcAttach(pin, FREQ_HZ, RES_BITS);          // Configura PWM: 50Hz, 12 bits
  writeServoLogical(pin, initialLogical);      // Posiciona servo inicialmente
}

// ============================================================================
// CONFIGURACIÓN DE RAMPA (MOVIMIENTO SUAVE)
// ============================================================================

const int  LIM_MIN     = 0;       // Límite mínimo de ángulo
const int  LIM_MAX     = 180;     // Límite máximo de ángulo
const int  PASO_RAMPA  = 5;       // Incremento por paso (grados)
const uint32_t DT_RAMP = 10;      // Intervalo entre pasos (ms)
const uint32_t TIMEOUT_MS = 700;  // Timeout para volver a home (ms)

// ============================================================================
// VARIABLES DE ESTADO
// ============================================================================

// Posiciones actuales de los servos (valores lógicos)
int posIzq    = HOME_IZQ;
int posArriba = HOME_ARRIBA;
int posDer    = HOME_DER;

// Posiciones objetivo (hacia donde deben moverse)
int tgtIzq    = HOME_IZQ;
int tgtArriba = HOME_ARRIBA;
int tgtDer    = HOME_DER;

// Control de tiempo
uint32_t tPrevRamp = 0;   // Timestamp del último paso de rampa
uint32_t tLastCmd  = 0;   // Timestamp del último comando recibido

// ============================================================================
// FUNCIÓN DE RAMPA SUAVE
// ============================================================================
/**
 * Aplica movimiento gradual hacia las posiciones objetivo
 * 
 * Se ejecuta cada DT_RAMP milisegundos y mueve cada servo
 * un máximo de PASO_RAMPA grados hacia su objetivo.
 * Esto evita movimientos bruscos que podrían dañar la mecánica
 * o causar oscilaciones.
 */
void aplicarRampa() {
  uint32_t now = millis();
  
  // Solo ejecuta si ha pasado suficiente tiempo
  if (now - tPrevRamp < DT_RAMP) return;
  tPrevRamp = now;

  // Lambda function para calcular siguiente posición
  // Mueve hacia el target en pasos de PASO_RAMPA
  auto go = [&](int actual, int target) {
    if (actual < target) return min(actual + PASO_RAMPA, target);  // Subir
    if (actual > target) return max(actual - PASO_RAMPA, target);  // Bajar
    return actual;  // Ya está en posición
  };

  // Actualiza posiciones actuales
  posIzq    = go(posIzq,    tgtIzq);
  posArriba = go(posArriba, tgtArriba);
  posDer    = go(posDer,    tgtDer);

  // Escribe las nuevas posiciones a los servos
  writeServoLogical(SERVO_IZQ,    posIzq);
  writeServoLogical(SERVO_ARRIBA, posArriba);
  writeServoLogical(SERVO_DER,    posDer);
}

// ============================================================================
// PARSER DE COMANDOS
// ============================================================================
/**
 * Parsea un mensaje de ángulos en formato "ANG:x,y,z"
 * 
 * @param msg Mensaje recibido (ej: "ANG:45,90,135")
 * @param aIzq Variable donde se guarda ángulo izquierdo
 * @param aArriba Variable donde se guarda ángulo central
 * @param aDer Variable donde se guarda ángulo derecho
 * @return true si el parseo fue exitoso, false si el formato es inválido
 */
bool parseAngulos(const String &msg, int &aIzq, int &aArriba, int &aDer) {
  // Verifica que el mensaje comience con "ANG:"
  if (!msg.startsWith("ANG:")) return false;

  // Extrae la parte de datos (después de "ANG:")
  String data = msg.substring(4);
  
  // Busca las comas que separan los valores
  int c1 = data.indexOf(',');          // Primera coma
  int c2 = data.indexOf(',', c1 + 1);  // Segunda coma
  
  // Verifica que existan ambas comas
  if (c1 < 0 || c2 < 0) return false;

  // Extrae y convierte cada valor
  aIzq    = data.substring(0, c1).toInt();      // Primer número
  aArriba = data.substring(c1 + 1, c2).toInt(); // Segundo número
  aDer    = data.substring(c2 + 1).toInt();     // Tercer número

  return true;
}

// ============================================================================
// FUNCIÓN SETUP - INICIALIZACIÓN
// ============================================================================
/**
 * Se ejecuta una vez al encender/reiniciar el ESP32
 * Configura la comunicación serial, Bluetooth y los servos
 */
void setup() {
  // Inicializa comunicación serial para debug (USB)
  Serial.begin(115200);
  
  // Inicializa Bluetooth con nombre visible "ESP32-Stewart"
  SerialBT.begin("ESP32-Stewart");

  // Configura los 3 servos con sus posiciones home
  configServo(SERVO_IZQ,    posIzq);
  configServo(SERVO_ARRIBA, posArriba);
  configServo(SERVO_DER,    posDer);

  // Mensajes de información en Serial Monitor
  Serial.println("ESP32 listo - Plataforma Stewart");
  Serial.println("Pines: 15 = Izq, 33 = Arriba, 25 = Der");

  // Inicializa timestamp del último comando
  tLastCmd = millis();
}

// ============================================================================
// FUNCIÓN LOOP - BUCLE PRINCIPAL
// ============================================================================
/**
 * Se ejecuta continuamente después del setup
 * Lee comandos Bluetooth, los procesa y actualiza los servos
 */
void loop() {

  // ========== LECTURA DE BLUETOOTH ==========
  // Lee caracteres disponibles y los acumula en el buffer
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();  // Lee un caracter

    if (c == '\n') {
      // ----- FIN DE LÍNEA: PROCESAR COMANDO -----
      String msg = btBuffer;  // Copia el buffer
      btBuffer = "";          // Limpia el buffer para el próximo mensaje
      msg.trim();             // Elimina espacios y saltos de línea

      if (msg.length() > 0) {
        tLastCmd = millis();  // Actualiza timestamp del último comando

        // ----- COMANDO ZERO O LOST -----
        // Enviados cuando se pierde la mano o se presiona 'z'
        if (msg == "ZERO" || msg == "LOST") {
          tgtIzq    = HOME_IZQ;
          tgtArriba = HOME_ARRIBA;
          tgtDer    = HOME_DER;
          Serial.println("HOME ejecutado (ZERO/LOST)");
        }
        // ----- COMANDO DE ÁNGULOS -----
        else {
          int aI, aA, aD;
          if (parseAngulos(msg, aI, aA, aD)) {
            // Parseo exitoso: actualiza objetivos con límites
            tgtIzq    = constrain(aI, LIM_MIN, LIM_MAX);
            tgtArriba = constrain(aA, LIM_MIN, LIM_MAX);
            tgtDer    = constrain(aD, LIM_MIN, LIM_MAX);
            Serial.printf("ANG -> %d, %d, %d\n", tgtIzq, tgtArriba, tgtDer);
          }
          else {
            // Comando no reconocido
            Serial.print("Comando desconocido: ");
            Serial.println(msg);
          }
        }
      }
    }
    else if (c != '\r') {
      // Acumula caracteres (ignora retorno de carro)
      btBuffer += c;
    }
  }

  // ========== TIMEOUT: RETORNO A HOME ==========
  // Si no se reciben comandos por TIMEOUT_MS, vuelve a posición home
  // Esto es una medida de seguridad por si se pierde la conexión
  if (millis() - tLastCmd > TIMEOUT_MS) {
    tgtIzq    = HOME_IZQ;
    tgtArriba = HOME_ARRIBA;
    tgtDer    = HOME_DER;
  }

  // ========== APLICAR RAMPA ==========
  // Mueve los servos suavemente hacia sus objetivos
  aplicarRampa();
  // Pequeño delay para estabilidad
  delay(1);
}
