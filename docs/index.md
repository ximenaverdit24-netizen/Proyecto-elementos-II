# Proyecto: Plataforma controlada por gestos de la mano

---

## 1) Resumen

- **Equipo / Autor(es):** Ximena Guadalupe Verdi Toledo 
- **Curso / Asignatura:** Elementos Programables II  
- **Fecha:** 07/12/2025  

**Descripción breve:**  
Este proyecto implementa una plataforma tipo **Stewart** de 3 grados de libertad controlada mediante **gestos de la mano**.  
El sistema utiliza **visión por computadora con MediaPipe** para detectar los landmarks de la mano (muñeca, dedo medio y pulgar) y, a partir de ellos, calcular los ángulos de **pitch** (inclinación arriba/abajo) y **roll** (inclinación izquierda/derecha).  

Los valores calculados se filtran con un **filtro exponencial** para suavizar el movimiento y, posteriormente, se envían por **Bluetooth Classic** a un **ESP32**, el cual controla **tres servomotores MG90S** en configuración triangular mediante **PWM a 50 Hz**. El firmware del ESP32 recibe comandos del tipo `ANG:x,y,z`, aplica rampas de movimiento y posiciona cada servo para inclinar la plataforma de acuerdo con los gestos del usuario.

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
- No se implementa en esta versión un modo automático de seguimiento de pelota, solo control directo por gestos.

---

## 4) Resultados

Al ejecutar el sistema completo:

1. La cámara capta la imagen de la mano del usuario en tiempo real.  
2. **MediaPipe** detecta automáticamente los landmarks de muñeca, dedo medio y pulgar.  
3. Con estas posiciones se calculan:
   - El **pitch**, a partir de la diferencia en profundidad (eje Z) entre muñeca y dedo medio.  
   - El **roll**, a partir de la diferencia vertical entre muñeca y pulgar.  
4. Ambos valores se filtran con un **promedio exponencial** para reducir vibraciones y cambios bruscos.  
5. Los ángulos resultantes se codifican en un mensaje `ANG:izq,arriba,der` y se envían por **Bluetooth** al ESP32.  
6. El **ESP32** interpreta los datos, aplica rampas de movimiento y genera las señales PWM adecuadas para los tres servos **MG90S**, inclinando la plataforma.  

En las pruebas realizadas se logró que la plataforma respondiera de manera fluida a los gestos de la mano, permitiendo inclinar la base hacia adelante/atrás y hacia los lados con buena estabilidad y sin movimientos abruptos. El sistema demostró ser una forma efectiva de **controlar una plataforma Stewart de manera intuitiva**, reforzando los conceptos de visión por computadora, comunicación inalámbrica y control de actuadores.

---

## 5) Archivos adjuntos / Código

En el repositorio se incluyen:

- **Script de Python** para el control por gestos de mano (captura de cámara, MediaPipe, cálculo de ángulos y envío Bluetooth).  
- **Firmware ESP32** en C++ (Arduino) para recepción de comandos `ANG:x,y,z`, generación de PWM y control de los 3 servos con rampado y límites.  
- Archivos de diseño **3D** de la estructura de la plataforma (base, brazos y soportes).  

> Desde esta sección se pueden enlazar los archivos específicos del repo (por ejemplo, el `.py`, el `.ino`/`.cpp` y los `.stl`), así como imágenes o videos de demostración del sistema funcionando.

---

## 6) Conclusión

El proyecto logró integrar de forma práctica varias áreas vistas en la materia **Elementos Programables II**:

- Procesamiento de imagen en tiempo real con **OpenCV + MediaPipe**.  
- Comunicación inalámbrica mediante **Bluetooth Classic** entre una PC y un microcontrolador.  
- Generación y control de señales **PWM** para servomotores con un **ESP32**.  
- Diseño y fabricación de una estructura mecánica mediante **impresión 3D**.

La plataforma Stewart controlada por gestos de la mano muestra cómo, con componentes accesibles y herramientas de software libres, es posible construir un sistema interactivo que combina visión por computadora y control de movimiento.  
Como trabajo futuro se podrían integrar modos adicionales de control (por ejemplo, seguimiento automático de pelota, control PID completo o interfaz gráfica) y optimizar la estructura mecánica para mejorar la precisión y la velocidad de respuesta.
