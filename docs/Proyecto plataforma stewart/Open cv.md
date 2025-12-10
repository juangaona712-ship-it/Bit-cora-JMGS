---
title: Vision artificial con Open cv
summary: Sistema completo de seguimiento visual que controla 3 servos mediante PID (X,Y,Área) con explicación detallada del código.
hide:
  - navigation
---

# Balanza PID 3D - OpenCV + Arduino

## Objetivo
Crear una plataforma autónoma que siga un objeto de color específico moviendo 3 servos usando visión por computadora y control PID avanzado.

## 1. Inicialización Cámara (cv2.VideoCapture)

### ¿Por qué VideoCapture(0)?

```python
cap = cv2.VideoCapture(0) # 0 = cámara predeterminada (webcam)
```

- `0` = primera cámara detectada (webcam USB/interna)
- `1` = segunda cámara, etc.
- **IMPORTANTE**: Siempre cerrar con `cap.release()` para liberar el dispositivo

### Ciclo de vida cámara

```python
✅ CORRECTO
cap = cv2.VideoCapture(0)
while True:
ret, frame = cap.read() # ret=True si lee bien
if not ret: break

...
cap.release() # Libera cámara para otras apps
cv2.destroyAllWindows() # Cierra ventanas OpenCV

❌ ERROR: Cámara queda "bloqueada"
cap = cv2.VideoCapture(0)
while True:
ret, frame = cap.read()
# Sin release() → otras apps no acceden cámara
```

## 2. Comunicación Serial Completa

### Inicialización robusta

```
PUERTO_SERIAL = 'COM11'
BAUDRATE = 11520
TIMEOUT = 0.01

try:
arduino = serial.Serial(PUERTO_SERIAL, BAUDRATE, timeout=TIMEOUT)
time.sleep(2) # Arduino necesita 2s para reset+boot
print("✓ Serial conectado")
except:
print("✗ Puerto no disponible")
exit()
```

**time.sleep(2)**: Arduino hace reset al conectar USB, necesita tiempo para cargar sketch.

### Envío de comandos

```python
comando = f"X:{int(outputX)},Y:{int(outputY)},Z:{int(outputArea)}\n"
arduino.write(comando.encode()) # String → bytes
```

- `\n` = terminador necesario para `readStringUntil('\n')` en Arduino
- `encode()` convierte string Python a bytes seriales

### Cierre seguro

```python
Antes de salir
arduino.write(b"X:0,Y:0,Z:0\n") # Servos a posición segura
time.sleep(0.5) # Servo llega a posición
arduino.close() # Libera puerto serial
```


## 3. Clase PID - Explicación Matemática

```python
class PID:
def update(self, measurement):
error = self.setpoint - measurement # Error actual
```

    self.integral += error * self.dt     # ∑error·Δt (acumula)
    self.integral = np.clip(self.integral, -50, 50)  # Anti-windup
    
    derivative = (error - self.prev_error) / self.dt  # Δerror/Δt
    self.output = (self.Kp*error + 
                  self.Ki*self.integral + 
                  self.Kd*derivative)
    self.output = np.clip(self.output, -90, 90)  # Límite servo
    
    self.prev_error = error
    return self.output


**np.clip()**: Evita que servos vayan más allá de límites físicos.

## 4. Detección HSV - ¿Por qué dos rangos para rojo?

```python
Rojo envuelve H=0° y H=180° (círculo HSV)
redbajo1 = np.array() # Rojo inicial (0-8)​
redalto1 = np.array()
redbajo2 = np.array() # Rojo final (170-179)​
redalto2 = np.array()
mask = cv2.add(mask1, mask2) # Combina ambos
```


**S=150, V=50**: Filtra rojos saturados y brillantes, elimina sombras/grises.

## 5. Procesamiento Morfológico

```python
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # Elimina ruido
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # Une huecos
```

- **OPEN**: Erosión → Dilatación = elimina puntos sueltos
- **CLOSE**: Dilatación → Erosión = une regiones separadas
- **ELLIPSE**: Más suave que rectángulo

## 6. Selección Mejor Contorno

```python
for c in contornos:
area = cv2.contourArea(c)
if 100 < area < 8000: # Filtra ruido/objetos grandes
M = cv2.moments(c)
if M["m00"] != 0: # Área válida
cx = int(M["m10"] / M["m00"]) # Centro X
cy = int(M["m01"] / M["m00"]) # Centro Y
if area > mejor_area: # Mayor = objetivo principal
mejor_contorno = c
```


**m00=0**: Contorno con área cero (inválido).

## 7. Filtro Exponencial (Estabilidad)

```python
alpha = 0.6 # 60% nuevo, 40% anterior
errorX = alpha * errorX_raw + (1-alpha) * prev_errorX
```

**Ejemplo**: Error raw: 10→50→-20 → Filtrado: 10→28→16 (suave).

## 8. Bucle Principal Explicado

```python
while True:
ret, frame = cap.read()
if not ret: break # Cámara desconectada
```

frame = cv2.flip(frame, 1)  # Espejo (intuitivo)
centrox = frame.shape // 2  # Centro ancho[2]
centroy = frame.shape // 2  # Centro alto

# [Detección HSV → PID → Serial → Visualizar]

if cv2.waitKey(1) & 0xFF == ord('q'):  # ESC o 'q'
    break


**waitKey(1)**: 1ms delay, ~1000 FPS máximo (limitado por cámara).

## 9. Visualización Informativa

```python
cv2.circle(frame, (centrox, centroy), 8, (0,255,0), -1) # Centro VERDE
cv2.circle(frame, (mejor_cx, mejor_cy), 7, (0,0,255), -1) # Target ROJO
cv2.drawContours(frame, [hull], 0, (255,255,0), 2) # Contorno CYAN
cv2.putText(frame, f'X11:{int(outputX)}', ...) # Valores PID
```

## 10. Limpieza Final (CRÍTICA)

```python
1. Servos a posición segura
arduino.write(b"X:0,Y:0,Z:0\n")
time.sleep(0.5) # Servo se mueve

2. Liberar recursos
cap.release() # Cámara libre para otras apps
arduino.close() # Puerto serial libre
cv2.destroyAllWindows() # Cierra todas ventanas OpenCV
```


**Sin release()**: Cámara queda bloqueada, serial ocupado.

## Código Arduino Completo

```c++
#include <Servo.h>
Servo servoX(11), servoY(6), servoZ(5);

void setup() {
Serial.begin(11520);
servoX.write(90); servoY.write(90); servoZ.write(90); // Neutral
}

void loop() {
if (Serial.available()) {
String cmd = Serial.readStringUntil('\n');
// Parsing robusto con índices
int x = cmd.substring(cmd.indexOf("X:")+2, cmd.indexOf(",Y")).toInt();
servoX.write(map(x, -90,90,0,180));
}
}
```

int x = cmd.substring(idxX, cmd.indexOf(",", idxX)).toInt();
int y = cmd.substring(idxY, cmd.indexOf(",", idxY)).toInt();
int z = cmd.substring(idxZ).toInt();

servoX.write(map(x, -90, 90, 0, 180));
servoY.write(map(y, -90, 90, 0, 180));
servoZ.write(map(z, -90, 90, 0, 180));

---


## Tabla HSV por Color Objetivo

| Color     | H_bajo1 | H_alto1 | H_bajo2 | H_alto2 | S_min | V_min |
|-----------|---------|---------|---------|---------|-------|-------|
| **Rojo**  | 0       | 8       | 170     | 179     | 150   | 50    |
| Verde     | 50      | 75      | -       | -       | 100   | 50    |
| Azul      | 100     | 130     | -       | -       | 100   | 50    |
| Amarillo  | 20      | 35      | -       | -       | 150   | 100   |
| Naranja   | 10      | 25      | -       | -       | 150   | 100   |
| Púrpura   | 140     | 160     | -       | -       | 100   | 50    |
| Rosa      | 160     | 175     | -       | -       | 100   | 100   |
| Cian      | 80      | 100     | -       | -       | 100   | 100   |

**Uso**: Reemplazar rangos rojo por el color deseado en el código.

## Código Python Completo

```python
import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time

PUERTO_SERIAL = 'COM11'
BAUDRATE = 11520
TIMEOUT = 0.01

try:
arduino = serial.Serial(PUERTO_SERIAL, BAUDRATE, timeout=TIMEOUT)
time.sleep(2)
print("✓ Serial conectado")
except:
print("✗ ERROR puerto")
exit()

class PID:
def init(self, Kp, Ki, Kd, setpoint=0, dt=0.03):
self.Kp = Kp; self.Ki = Ki; self.Kd = Kd
self.setpoint = setpoint; self.dt = dt
self.prev_error = 0; self.integral = 0; self.output = 0

def update(self, measurement):
    error = self.setpoint - measurement
    self.integral += error * self.dt
    self.integral = np.clip(self.integral, -50, 50)
    derivative = (error - self.prev_error) / self.dt
    self.output = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
    self.output = np.clip(self.output, -90, 90)
    self.prev_error = error
    return self.output

pid_x = PID(0.8, 0.05, 0.15)
pid_y = PID(0.8, 0.05, 0.15)
pid_area = PID(0.3, 0.02, 0.08, setpoint=2000)

cap = cv2.VideoCapture(0)
alpha = 0.6
prev_errorX, prev_errorY = 0, 0

print("🎯 BALANZA PID 3D")
while True:
ret, frame = cap.read()
if not ret: break

frame = cv2.flip(frame, 1)
centrox = frame.shape // 2[2]
centroy = frame.shape // 2

hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
redbajo1 = np.array(); redalto1 = np.array()[1]
redbajo2 = np.array(); redalto2 = np.array()[1]

mask1 = cv2.inRange(hsv, redbajo1, redalto1)
mask2 = cv2.inRange(hsv, redbajo2, redalto2)
mask = cv2.add(mask1, mask2)

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

mejor_contorno = None; mejor_cx, mejor_cy, mejor_area = None, None, 0

for c in contornos:
    area = cv2.contourArea(c)
    if 100 < area < 8000:
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            if area > mejor_area:
                mejor_contorno = c
                mejor_cx, mejor_cy = cx, cy
                mejor_area = area

if mejor_contorno is not None:
    errorX_raw = mejor_cx - centrox
    errorY_raw = mejor_cy - centroy
    errorX = alpha * errorX_raw + (1-alpha) * prev_errorX
    errorY = alpha * errorY_raw + (1-alpha) * prev_errorY
    prev_errorX, prev_errorY = errorX, errorY
    
    outputX = pid_x.update(errorX)
    outputY = pid_y.update(errorY)
    outputArea = pid_area.update(mejor_area)
    
    comando = f"X:{int(outputX)},Y:{int(outputY)},Z:{int(outputArea)}\n"
    arduino.write(comando.encode())
    
    cv2.circle(frame, (int(mejor_cx), int(mejor_cy)), 7, (0,0,255), -1)
    hull = cv2.convexHull(mejor_contorno)
    cv2.drawContours(frame, [hull], 0, (255,255,0), 2)
else:
    arduino.write(b"X:0,Y:0,Z:0\n")

cv2.circle(frame, (centrox, centroy), 8, (0,255,0), -1)
cv2.imshow('BALANZA PID 3D', frame)

if cv2.waitKey(1) & 0xFF == ord('q'): break

arduino.write(b"X:0,Y:0,Z:0\n")
time.sleep(0.5)
cap.release()
arduino.close()
cv2.destroyAllWindows()
print("✓ Sistema cerrado")

```
---

## Evidencias
<img src="../recursos/imgs/Tercero/Deteccion.png" alt="Interfaz con centros y valores PID" width="420">

<img src="../recursos/imgs/Tercero/Rango_HSV.png_" alt="Interfaz con centros y valores PID" width="420">
