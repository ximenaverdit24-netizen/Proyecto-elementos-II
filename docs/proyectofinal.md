# Proyecto: Plataforma controlada por gestos de la mano

---

## 1) Resumen

- **Equipo / Autor(es):** Ximena Guadalupe Verdi Toledo  
- **Curso / Asignatura:** Elementos Programables II  
- **Fecha:** 07/12/2025  

**Descripción breve:**  
Este proyecto implementa una plataforma tipo **Stewart** de 3 grados de libertad controlada mediante **gestos de la mano**.  
El sistema utiliza **visión por computadora con MediaPipe** para detectar los landmarks de la mano (muñeca, dedo medio y pulgar) y, a partir de ellos, calcular los ángulos de **pitch** (inclinación arriba/abajo) y **roll** (inclinación izquierda/derecha).

Los valores calculados se filtran con un **filtro exponencial** para suavizar el movimiento y, posteriormente, se envían por **Bluetooth Classic** a un **ESP32**, el cual controla **tres servomotores MG90S** en configuración triangular mediante **PWM a 50 Hz**.  
El firmware del ESP32 recibe comandos del tipo `ANG:x,y,z`, aplica rampas de movimiento y posiciona cada servo para inclinar la plataforma de acuerdo con los gestos del usuario.

---

## 2) Objetivos

### 🎯 Objetivo general
Desarrollar un sistema de control para una plataforma Stewart de 3 grados de libertad, utilizando **reconocimiento de gestos de la mano** con visión por computadora y **comunicación inalámbrica Bluetooth** hacia un ESP32 que gobierna los servomotores.

### 🎯 Objetivos específicos

- **OP1.** Implementar la detección de la mano en tiempo real con **MediaPipe**, obteniendo los landmarks de muñeca, dedo medio y pulgar.  
- **OP2.** Calcular los parámetros de inclinación (**pitch** y **roll**) a partir de las posiciones relativas de estos puntos y aplicar filtros para reducir ruido y temblor.  
- **OP3.** Establecer comunicación Bluetooth entre el programa en **Python** y el **ESP32**, transmitiendo periódicamente los ángulos calculados.  
- **OP4.** Controlar 3 servomotores **MG90S** mediante señales **PWM** generadas por el ESP32, de forma que la plataforma reproduzca de manera estable los gestos del usuario.

---

## 3) Alcance y exclusiones

### ✅ Alcance
- Diseño e impresión 3D de la **estructura de la plataforma Stewart** (base, brazos y soportes).  
- Implementación de un script en **Python** con OpenCV + MediaPipe para:  
  - Captura de video.  
  - Detección de mano.  
  - Cálculo de pitch y roll.  
  - Filtrado exponencial de las señales.  
- Implementación de firmware en **ESP32** para:
  - Recepción de comandos vía Bluetooth (`ANG:x,y,z`).  
  - Conversión a **PWM de 12 bits, 50 Hz**.  
  - Movimiento suave de los servos mediante rampa y límites de seguridad.  

### 🚫 Exclusiones / restricciones
- No se utiliza realimentación de posición de los servos (no hay encoders).  
- No se implementa un controlador PID formal; el control se basa en mapeos directos de los gestos y filtrado EMA.  
- La detección de la mano asume **buena iluminación** y una sola mano en cuadro.  
- No se implementa seguimiento automático de pelota, solo control manual por gestos.

---

## 4) Resultados

Al ejecutar el sistema completo:

1. La cámara capta la imagen de la mano del usuario en tiempo real.  
2. **MediaPipe** detecta automáticamente los landmarks de muñeca, dedo medio y pulgar.  
3. Con estas posiciones se calculan:
   - El **pitch**, a partir de la diferencia en profundidad (eje Z).  
   - El **roll**, a partir de la diferencia vertical entre muñeca y pulgar.  
4. Ambos valores se filtran con un **promedio exponencial** para reducir vibraciones.  
5. Los ángulos resultantes se codifican como `ANG:izq,arriba,der` y se envían al ESP32 vía **Bluetooth**.  
6. El **ESP32** interpreta los datos, aplica una rampa de movimiento y genera las señales PWM necesarias para los 3 servos **MG90S**, inclinando la plataforma.

Se obtuvo una respuesta **suave, estable y en tiempo real**, lo que demuestra que se puede implementar control de plataformas robóticas de forma intuitiva utilizando visión por computadora y actuadores económicos.

---

### 📌 5.1 Script Python – Control por gestos

```python
import cv2
import mediapipe as mp
import time
import bluetooth

# ================== CONEXIÓN BLUETOOTH ==================

PORT = 1
ESP32_MAC = "14:33:5C:02:4D:2A"   # CAMBIA a la MAC de tu ESP32

sock = bluetooth.BluetoothSocket()
sock.settimeout(20)

print("Intentando conectar al ESP32...")
while True:
    try:
        sock.connect((ESP32_MAC, PORT))
        print("¡Conectado al ESP32!")
        break
    except Exception as e:
        print("Error en conexión... reintentando:", e)
        time.sleep(1)

def send_bt(message: str):
    try:
        sock.send(message.encode())
        print("Enviado:", message.strip())
    except:
        print("Error enviando datos")


# ================== MEDIAPIPE ==================

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1,
                       min_detection_confidence=0.6,
                       min_tracking_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

# ===== FILTROS =====
pitch_filtrado = 0
roll_filtrado = 0
alpha = 0.25

ultimo_envio = time.time()
intervalo_envio = 0.05

# ===== HOME =====
HOME_IZQ = 90
HOME_ARRIBA = 90
HOME_DER = 90

# ===== GANANCIAS =====
K_pitch = 30.0
K_roll  = 0.05
K_lat   = 85.0
K_mid_acompa = 20.0


while cap.isOpened():

    ret, img = cap.read()
    if not ret:
        break

    img = cv2.flip(img, 1)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)
    h, w, _ = img.shape

    detectada = False

    if results.multi_hand_landmarks:
        for hand in results.multi_hand_landmarks:
            detectada = True
            mp_draw.draw_landmarks(img, hand, mp_hands.HAND_CONNECTIONS)

            muneca = hand.landmark[0]
            medio  = hand.landmark[12]
            pulgar = hand.landmark[4]

            wx, wy = int(muneca.x*w), int(muneca.y*h)
            mx, my = int(medio.x*w),  int(medio.y*h)
            px, py = int(pulgar.x*w), int(pulgar.y*h)

            cv2.circle(img, (wx,wy), 10, (255,0,0), -1)
            cv2.circle(img, (mx,my), 10, (0,255,0), -1)
            cv2.circle(img, (px,py), 10, (0,0,255), -1)

            pitch = (muneca.z - medio.z) * 1.8
            pitch = max(-1, min(1, pitch))

            dy = wy - py
            roll = dy * K_roll
            roll = max(-1, min(1, roll))

            pitch_filtrado = (1-alpha)*pitch_filtrado + alpha*pitch
            roll_filtrado  = (1-alpha)*roll_filtrado  + alpha*roll

            if abs(pitch_filtrado) < 0.05:
                pitch_filtrado = 0
            if abs(roll_filtrado) < 0.05:
                roll_filtrado = 0

            a_arriba = HOME_ARRIBA + K_pitch*pitch_filtrado + K_mid_acompa*abs(roll_filtrado)

            delta_lat = K_lat * roll_filtrado
            a_izq = HOME_IZQ - delta_lat
            a_der = HOME_DER + delta_lat

            a_izq += (K_pitch*0.25)*pitch_filtrado
            a_der += (K_pitch*0.25)*pitch_filtrado

            a_izq = int(max(0, min(180, a_izq)))
            a_arriba = int(max(0, min(180, a_arriba)))
            a_der = int(max(0, min(180, a_der)))

            if time.time() - ultimo_envio >= intervalo_envio:
                msg = f"ANG:{a_izq},{a_arriba},{a_der}\n"
                send_bt(msg)
                ultimo_envio = time.time()

    if not detectada:
        cv2.putText(img, "No se detecta mano", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    cv2.imshow("STEWART CONTROL PULGAR", img)

    k = cv2.waitKey(1)
    if k == ord('q'):
        break
    if k == ord('z') or k == ord('c'):
        send_bt("ZERO\n")

sock.close()
cap.release()
cv2.destroyAllWindows()
print("Programa terminado")
```

## 6) Conclusión

El proyecto logró integrar de forma práctica varias áreas vistas en la materia **Elementos Programables II**:

- Procesamiento de imagen en tiempo real con **OpenCV + MediaPipe**.  
- Comunicación inalámbrica mediante **Bluetooth Classic** entre una PC y un microcontrolador.  
- Generación y control de señales **PWM** para servomotores con un **ESP32**.  
- Diseño y fabricación de una estructura mecánica mediante **impresión 3D**.

La plataforma Stewart controlada por gestos de la mano muestra cómo, con componentes accesibles y herramientas de software libres, es posible construir un sistema interactivo que combina visión por computadora y control de movimiento.  
Como trabajo futuro se podrían integrar modos adicionales de control (por ejemplo, seguimiento automático de pelota, control PID completo o interfaz gráfica) y optimizar la estructura mecánica para mejorar la precisión y la velocidad de respuesta.
