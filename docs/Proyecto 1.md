# 📚 Proyecto 1


## 1) Resumen

- **Nombre del proyecto:** _Conexion de ESP32_  
- **Equipo / Autor(es):** _Juan Manuel Gaona Serrano_  
- **Curso / Asignatura:** _Introducción a la mecatronica_  
- **Fecha:** _13/09/2025_  
- **Descripción breve:** _En este proyecto utilizamos un ESP32 para cargarle diferentes codigos y prender un led de diferentes formas, todo con el fin de aprender a utilizar la placa._

---

## 2) Objetivos

- **General:** _Conectar la placa con un celular usando una señal de bluetooth._
- **Específicos:**
  - _Codigo de conexión bluetooth_
  - _Uso de la placa ESP32_
  - _Cargar codigo a placa_

## 3) Alcance y Exclusiones

- **Incluye:** _Codigo para conectar la placa al celular._
- **No incluye:** _La conexión del led en la protoboard._

---

## 4) Requisitos

**Software**
- _Phyton_
- _Serial Bluetooth Terminal._

**Hardware (si aplica)**
- _ESP32_
- _Foco led_
- _Protoboard_
- _Resistencia de 220_

**Conocimientos previos**
- _Programación básica_
- _Electrónica básica_

---

## 5) Instalación

```bash

ich_sep12a ino
const int led - 13;
const int btn = 12;
void setup() {
//inicio la concat
Serial.begin(115200);
pinMode(led, OUTPUT);
pinMode(btn, INPUT);
void loop() {
int estado digitalRead(btn);
if (estado 1) (
digitalWrite(led, 1);
Serial.println("ON");
} else {
digitalWrite(led, 0);
Serial.println("OFF");


```
Se cargo este código a la placa ESP32 donde colocamos un botón para prender el led usando un botón.

[Video Uso del programa][(https://iberopuebla.sharepoint.com/:v:/r/sites/Section_11192A-O25/Student%20Work/Submitted%20files/GAONA%20SERRANO%20JUAN%20MANUEL/MCU%20101/VID_20250912_103243_800.mp4?csf=1&web=1&e=nxHvu2&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJ0ZWFtcyIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJwb3N0cm9sbC1jb3B5bGluayIsInJlZmVycmFsUGxheWJhY2tTZXNzaW9uSWQiOiIwN2ZjODI0My1iOWQ5LTQ4MTgtYWM0NC0wZDE2YjQxYjExYmQifX0%3D)]

