# 📚 Proyecto 2


## 1) Resumen

- **Nombre del proyecto:** _Uso de motores y servomotres_  
- **Equipo / Autor(es):** _Juan Manuel Gaona Serrano_  
- **Curso / Asignatura:** _Introducción a la mecatronica_  
- **Fecha:** _13/09/2025_  
- **Descripción breve:** _En este proyecto utilizamos motores y servomotores para poder controlarlos._

---

## 2) Objetivos

- **General:** _Aprender a usar los motores y servomoters._
- **Específicos:**
  - _Conexiones de motores y servomotores_
  - _Uso de la placa ESP32_
  - _Cargar codigo a placa_

## 3) Alcance y Exclusiones

- **Incluye:** _Codigo y conexión para utilizar los motores y servomotores._
- **No incluye:** _El uso de fuente de poder._

---

## 4) Requisitos

**Software**
- _Phyton_

**Hardware (si aplica)**
- _ESP32_
- _motor pololu_
- _servomotor_

**Conocimientos previos**
- _Programación básica_
- _Electrónica básica_

---

## 5) Instalación

```bash

#define in1 18

#define in1 19

void setup() {

// put your setup code here, to run once:

pinMode(in1, OUTPUT);

pinMode(in2, OUTPUT);

}

void loop() {

/*

1.-Motor avance en una direccion durante 4 segs

2.-Pare 2segs

3.-Motor avanza en direccion opuesta

*/

//Avance cw

digitalWrite(in1, 1);

digitalWrite(in2, 0);

delay(4000);

digitalWrite(in1, 0);

digitalWrite(in2, 0);

delay(2000);

digitalWrite(in1, 0);

digitalWrite(in2, 1);

delay(4000);


}


```
Con este código se logra mover un motor a un lado durante 4 segundos y después se mueve al otro lado durante otros 4 segundos.

[Video Uso del programa](https://iberopuebla.sharepoint.com/:v:/r/sites/Section_11192A-O25/Student%20Work/Submitted%20files/GAONA%20SERRANO%20JUAN%20MANUEL/VIdeos%20de%20Actuadores/VID_20250919_103129_846.mp4?csf=1&web=1&e=eJUDmM&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJ0ZWFtcyIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJ2aWRlb2FjdGlvbnMtc2hhcmUiLCJyZWZlcnJhbFBsYXliYWNrU2Vzc2lvbklkIjoiNTQ3NTQ3ZDctYzcwNi00OTM1LWJlMWItODgwNGU1YzQ5ZWNkIn19)

```bash

#define in1 18

#define in2 19

#define pum 21

void setup() (

// put your setup code here, to run once

pinMode(in1, OUTPUT);

pinMode(in2, OUTPUT);

ledcAttach(pum, 1000,8);

void loop() {

// put your main code here, to run repeatedly:

for(int i=0; 1 <= 255; 1++){

}

ledcwrite(pwm,1);;

digitalwrite(in1,1);

digitalWrite(in2,0);

deley(400);

digitalWrite(in1,0);

digitalwrite(in1,0);

delay(200);

digitalwrite(in1,0);

digitalWrite(in1,1);

delay (400)
}

```
Con este código se logra acelerar y desacelerar de un lado a otro usando un motor.

[Video Uso del programa](https://iberopuebla.sharepoint.com/:v:/r/sites/Section_11192A-O25/Student%20Work/Submitted%20files/GAONA%20SERRANO%20JUAN%20MANUEL/VIdeos%20de%20Actuadores/VID_20250926_100814_770%203.mp4?csf=1&web=1&e=KVkg29&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJ0ZWFtcyIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJ2aWRlb2FjdGlvbnMtc2hhcmUiLCJyZWZlcnJhbFBsYXliYWNrU2Vzc2lvbklkIjoiNzUzZGFlYTAtYWY3OS00ODU4LWE4NDMtYTNhZTY5NzUxZWE1In19)
