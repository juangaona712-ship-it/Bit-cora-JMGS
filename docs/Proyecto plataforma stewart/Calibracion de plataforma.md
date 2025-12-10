---
title: Calibrador Interactivo HoughCircles
summary: Herramienta gráfica para optimizar parámetros de detección de círculos (HoughCircles) en tiempo real con slicers y controles por rueda/teclado.
hide:
  - navigation
---

# Calibrador HoughCircles - Plato Circular

## Objetivo
Crear una interfaz interactiva que permita ajustar en tiempo real los 6 parámetros de cv2.HoughCircles para detectar platos circulares de forma óptima, guardando valores finales para código principal.

## Parámetros HoughCircles Explicados

| Parámetro | Función | Rango típico | Efecto visual |
|-----------|---------|--------------|---------------|
| dp | Resolución acumulador (1=imagen, 2=mitad) | 0.8-2.0 | Precisión vs velocidad |
| minDist | Distancia mínima círculos | 100-400px | Evita detección múltiple |
| param1 | Umbral Canny (bordes) | 50-200 | Sensibilidad bordes |
| param2 | Umbral acumulador círculo | 15-60 | Confianza detección |
| minRadius | Radio mínimo círculo | 80-200px | Filtra círculos pequeños |
| maxRadius | Radio máximo círculo | 200-400px | Filtra círculos grandes |

## 1. Inicialización y Ventana Interactiva

```python
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

ventana_calibrador = 'HOUGH CIRCLES CALIBRADOR'
cv2.namedWindow(ventana_calibrador)
cv2.setMouseCallback(ventana_calibrador, mouse_callback)
```

- 640x480: Resolución fija para slicers consistentes
- mouse_callback: Detecta rueda del mouse sobre slicers

## 2. Frame Slicers (Panel de Control)

```python
frame_slicers = np.zeros((480, 640, 3), dtype=np.uint8)
frame_slicers.fill(30)  # Fondo gris oscuro
```

Crea panel negro separado (mismo tamaño cámara) para 6 slicers horizontales.

## 3. Función dibujar_slicer()

```python
def dibujar_slicer(frame, y_pos, valor, label, min_val, max_val, color):
    # Barra fondo gris
    cv2.rectangle(frame, (20, y_pos), (620, y_pos+35), (60,60,60), -1)
    # Barra progreso coloreada
    proporcion = (valor - min_val) / (max_val - min_val)
    ancho_activo = int(proporcion * 600)
    cv2.rectangle(frame, (20, y_pos+5), (20+ancho_activo, y_pos+30), color, -1)
    # Etiqueta con valor actual
    cv2.putText(frame, f'{label}: {valor:.1f}', (20, y_pos-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
```

Cada slicer: Barra gris + progreso coloreado + texto valor/rango.

## 4. Detección HoughCircles en Tiempo Real

```python
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)  # Suavizado esencial

circles = cv2.HoughCircles(
    blurred, cv2.HOUGH_GRADIENT, 
    dp=float(dp), minDist=int(minDist),
    param1=int(param1), param2=int(param2),
    minRadius=int(minRadius), maxRadius=int(maxRadius)
)
```

GaussianBlur(5,5): Elimina ruido antes de Hough (crítico).

## 5. Selección Mejor Círculo

```python
if circles is not None:
    circles = np.round(circles[0, :]).astype("int")
    areas = [r*r for _,_,r in circles]  # Área = πr² ≈ r²
    mejor_idx = np.argmax(areas)  # MAYOR = más confiable
    mejor_plato = circles[mejor_idx]
```

Criterio: El círculo de mayor radio es el plato principal.

## 6. Visualización Inteligente

# TODOS los círculos (transparencia por confianza)

```python
for i, (x, y, r) in enumerate(circles):
    alpha = 0.3 + 0.7 * (r*r / max(areas))  # Mayor área = más visible
    color = (0, int(255*alpha), 0)
    cv2.circle(frame, (x, y), r, color, 2)

# MEJOR PLATO (verde grueso)
cv2.circle(frame, (x, y), r, (0,255,0), 3)
cv2.circle(frame, (x, y), 6, (0,255,0), -1)
```

Alpha dinámico: Círculos pequeños semitransparentes, grandes=opacos.

## 7. Controles Interactivos

### Rueda del Mouse (sobre slicers)

```python
if event == cv2.EVENT_MOUSEWHEEL:
    delta = 0.05 if event > 0 else -0.05
    if y_slicer <= y <= y_slicer+33:  # Zona DP
        dp = np.clip(dp + delta, 0.8, 2.0)
```

### Teclado rápido
Q/A: dp ±0.1    D/E: minDist ±20
Z/X: param1 ±10  C/V: param2 ±5
B/N: minR ±10    ,/.: maxR ±20
ESPACIO: Guardar  ESC: Salir

## 8. Interfaz Combinada

```python
frame_combinado = np.hstack([frame, frame_slicers])  # Izquierda=Derecha
cv2.imshow(ventana_calibrador, frame_combinado)
```

hstack: Cámara (640px) + Slicers (640px) = 1280px ancho.

## 9. Guardado Valores Óptimos

```python
if tecla == 32:  # ESPACIO
    valores_optimos = {'dp': dp, 'minDist': minDist, ...}
    print("✅ VALORES ÓPTIMOS GUARDADOS:")
```

Salida lista para copiar a código principal HoughCircles.

## 10. Valores Iniciales Recomendados

dp = 1.2        # Resolución media
minDist = 200   # Platos ~20cm separados
param1 = 100    # Bordes medios
param2 = 30     # Confianza media
minRadius = 120 # Platos ~12cm diámetro
maxRadius = 300 # Hasta ~30cm

## Uso Práctico

1. Ejecutar calibrador → apuntar cámara a platos
2. Ajustar slicers con rueda/teclado hasta detectar 1 círculo verde grueso
3. ESPACIO → copiar valores impresos
4. Pegar en código principal:

```python
circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 
    dp=1.2, minDist=200, param1=100, param2=30, 
    minRadius=120, maxRadius=300)
```

## Problemas Comunes

| Síntoma              | Solución                  |
|----------------------|---------------------------|
| Demasiados círculos  | ↑minDist, ↓param2         |
| No detecta plato     | ↓param1, ↑param2, ↓dp     |
| Círculos muy pequeños| ↑minRadius                |
| Demasiado lento      | ↑dp (1.5-2.0)             |
| Bordes ruidosos      | GaussianBlur(7,7)         |

## Evidencias
<img src="../recursos/imgs/Tercero/Calibracion_plataforma.jpeg" alt="Interfaz calibrador con slicers" width="420">
