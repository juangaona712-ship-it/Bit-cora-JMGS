---
title: Prueba de 6 Cuadrantes Polares
summary: Sistema de seguimiento visual con detección polar en 6 sectores de 60° + zonas centro, slicers interactivos y control PID para 3 servos.
hide:
  - navigation
---

# Balanza PID 3D - 6 Cuadrantes Polares

## Objetivo
Crear sistema de seguimiento que divide la imagen en 6 sectores polares de 60° más zonas centro superior/inferior, con slicers interactivos para filtrar área y control PID que envía comandos a Arduino.

## Arquitectura Dual Ventana
VENTANA 1 'Camera': Video en vivo + 6 radios + detección polar + PID
VENTANA 2 'SLICER': Controles interactivos min/max área

## 1. División Polar 6 Cuadrantes

```python
def obtener_cuadrante_6(cx, cy, centrox, centroy):
    dx = cx - centrox; dy = cy - centroy
    distancia = math.sqrt(dx**2 + dy**2)
    
    if distancia < 50:  # Zona CENTRO
        return "CENTRO_SUP/INF", 0.0, 0.0, color_amarillo/cyan
    
    angulo = math.atan2(dy, dx) * 180 / math.pi  # 0°=derecha
    if angulo < 0: angulo += 360
    sector = int(angulo // 60)  # 6 sectores 60°
```

Sectores: 0°(Der-Rojo), 60°(Naranja), 120°(Magenta), 180°(Cian), 240°(Rosa), 300°(Verde)

## 2. Errores Normalizados Polares

```python
error_magnitud = np.clip(distancia / 250, 0, 1.0)  # 0=centro, 1=borde
errorX_norm = (dx / distancia) * error_magnitud
errorY_norm = (dy / distancia) * error_magnitud
```

Vector unitario × magnitud: Error direccional ponderado por distancia.

## 3. Comunicación Serial Arduino

```python
PUERTO_SERIAL = 'COM11'; BAUDRATE = 11520
arduino = serial.Serial(PUERTO_SERIAL, BAUDRATE, timeout=0.01)
time.sleep(2)  # Reset Arduino
```

Comando: "X:45,Y:-12,Z:78\n" → Servos pines 11,6,5

## 4. Controladores PID (3 ejes)

```python
pid_x = PID(Kp=0.8, Ki=0.05, Kd=0.15)     # Horizontal
pid_y = PID(Kp=0.8, Ki=0.05, Kd=0.15)     # Vertical
pid_area = PID(Kp=0.3, Ki=0.02, Kd=0.08, setpoint=2000)  # Distancia
```

Escalado: errorX_norm * 100 → salida ±90° servo

## 5. Slicers Interactivos (min/max área)

```python
def dibujar_slicer(frame, y_pos, valor, label, min_val, max_val, color):
    cv2.rectangle(frame, (10, y_pos), (310, y_pos+30), (50,50,50), -1)  # Fondo
    proporcion = (valor - min_val) / (max_val - min_val)
    cv2.rectangle(frame, (10, y_pos), (10+int(proporcion*300), y_pos+30), color, -1)
```

Controles: Rueda mouse ↑↓ | A/Z:min±100 | S/X:max±200

## 6. Detección Visual Optimizada

```python
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
mask1 = cv2.inRange(hsv, [0,150,50], [8,255,255])     # Rojo1
mask2 = cv2.inRange(hsv, [170,150,50], [179,255,255]) # Rojo2
mask = cv2.add(mask1, mask2)
```

Filtro área: min_area(100)-max_area(8000) elimina ruido/objetos grandes

## 7. Visualización 6 Radios Polares

```python
radio_lineas = 150
for angulo in range(0, 360, 60):  # Cada 60°
    x_fin = centrox + radio_lineas * math.cos(math.radians(angulo))
    y_fin = centroy + radio_lineas * math.sin(math.radians(angulo))
    cv2.line(frame, (centrox, centroy), (x_fin, y_fin), (0,255,0), 2)
```

6 líneas verdes: Dividen imagen en sectores polares precisos.

## 8. Flujo Principal Dual Ventana

```python
while True:
    frame = cap.read() → flip(1)
    # 1. HSV → Morfología → Contornos filtrados
    # 2. Mejor contorno → cuadrante_6() → errores normalizados
    # 3. PID → outputX/Y/Z → Serial Arduino
    # 4. Dibujar: centro+radio50+6lineas+target+cuadrante
    # 5. frame_slicer → 2 slicers + cursor mouse
    cv2.imshow('Camera', frame)
    cv2.imshow('SLICER', frame_slicer)
```

## 9. Controles Interactivos Completos

TECLADO:
A/Z: min_area ±100
S/X: max_area ±200
Q: Salir

RATÓN (ventana SLICER):
↑↓Rueda sobre slicer1: min_area ±50
↑↓Rueda sobre slicer2: max_area ±100

## 10. Información en Pantalla Tiempo Real

CENTRO: Círculo verde r=10 + borde r=50 (zona CENTRO)
TARGET: Círculo coloreado por cuadrante r=8
RADIO: 6 líneas verdes 150px cada 60°
TEXTO: Cuadrante | X/Y_norm | Área | PID_XYZ
SLICER: min_area(verde) | max_area(naranja)

## 11. Limpieza Segura

```python
arduino.write(b"X:0,Y:0,Z:0\n")  # Servos neutral
time.sleep(0.5)  # Movimiento completo
cap.release()
arduino.close()
cv2.destroyAllWindows()
```

## Valores Iniciales Recomendados

min_area = 100    # Elimina ruido pequeño
max_area = 8000   # Limita objetos grandes
setpoint_area = 2000  # Tamaño objetivo pelota
radio_centro = 50     # Zona muerta central
radio_lineas = 150    # Visualización sectores

## Problemas Comunes y Soluciones

| Problema | Causa | Solución |
|----------|-------|----------|
| No detecta | Área fuera rango | Ajustar slicers A/Z/S/X |
| Cuadrante erróneo | Pelota en borde | ↑radio_centro (60-80) |
| PID inestable | Error muy pequeño | errorX_norm * 150 |
| Servos no responden | Serial | Verificar COM11/11520 |

## Evidencias
<img src="../recursos/imgs/Tercero/Test_cuadrantes.jpeg" alt="6 sectores polares con líneas radiales" width="420">
