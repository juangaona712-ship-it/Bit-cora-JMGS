# 📚  Proyecto plataforma stewart


## 1) Resumen

- **Nombre del proyecto:** _Plataforma niveladora con servomotores_  
- **Equipo / Autor(es):** _Carlos Alberto Vasquez Perasa, Luis Hernesto Tamez Velazquez, Erik Andre Zepeda Tapia, Sebastian Gomez Rodrigues, Juan Manuel Gaona Serrano_  
- **Curso / Asignatura:** _Introducción a la mecatronica_  
- **Fecha:** _05/11/2025_  
- **Descripción breve:** _Este proyecto consiste en el desarrollo e implementación de una plataforma de autonivelación basada en la Plataforma Stewart. Este tipo de plataforma se utiliza en diversas industrias, como en simuladores de vuelo y posicionamiento de precisión._

---

## 2) Objetivos

- **General:** _El objetivo principal es mantener un objeto, específicamente una pelota, en una posición fija sobre la superficie de la plataforma, evitando su caída._
- **Específicos:**
  - _Visión por Computadora: Una cámara captura la posición y el color (rojo) de la pelota en tiempo real._
  - _Control: La información de la cámara es procesada por un programa en phyton que manda información a un programa c++ que se encarga del control de un SP32, que a su ves controla los servomotores._
  - _Actuación: La nivelación se ejecuta mediante el control preciso de tres servomotores ubicados con una separación de un angulo de 120°, los cuales ajustan los actuadores de la plataforma para inclinar y desplazar el plano y así mantener la pelota en el centro._

## 3) Alcance y Exclusiones

- **Incluye:** _Piezas de solidWorks y en .STL, planos para corte laser en .DXF, Codigo de la camara en phyton, codigo para SP32 en c++_
- **No incluye:** _La conexión de la parte mecanica, ni las medidad de los tornillos utilizados_

---

## 4) Requisitos

**Software**
- _solidWorks._
- _Serial Bluetooth Terminal._
- _Visual Studio Code_
- _Arduino IDE_

**Hardware (si aplica)**
- _ESP32_
- _Camara_
- _Tripie_
- _Servomotores_
- _Tornillos_
- _Partes mecanicas impresas en 3d_

**Conocimientos previos**
- _Programación básica_
- _Electrónica básica_
- _Diseño por computadora basico_

---

## 5) Instalación

### Codigo de phyton

### Codigo de c++

### Piezas .STL
