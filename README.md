# Wall-eees

##  Descripción

Este repositorio contiene la estructura modular de software para el robot **WALL-EES**, considerando las dos pistas del reto:

-   **Pista A:** detección de colores, obstáculos, casilla final roja y detección de ArUco como bonus
-   **Pista B:** búsqueda y transporte de una pelota, cruce evitando líneas blancas y navegación mediante instrucciones basadas en colores

La arquitectura separa las responsabilidades entre:

-   **Arduino:** control físico y tareas de tiempo real
-   **Raspberry Pi:** visión, toma de decisiones, navegación y lógica de alto nivel

El objetivo es evitar duplicar código y permitir que los módulos comunes sean reutilizados en ambas pistas

------------------------------------------------------------------------

# Estructura del proyecto

``` text
ROBOT WALL-EEES/
│
├── ARDUINO/
│   |
│   ├── PISTA_A/
│   │   └── pistaA.ino
│   │
│   └── PISTA_B/
│       └── pistaB.ino
│
└── RASPBERRY/
    │
    ├── PISTA_A/
    │   └── pista_a.py
    │
    └── PISTA_B/
        └── pista_b.py
```
------------------------------------------------------------------------

# Orden de pruebas

1. Compilación: verifica que cada archivo .ino compile por separado.
2. Encoders: levanta las ruedas y comprueba que ambos motores generen pulsos.
3. Giros: prueba 90° a la derecha y a la izquierda.
4. Avance: calibra una casilla de 30 cm.
5. Sensores: verifica el ultrasónico, los colores y la tira NeoPixel.
6. Pista completa: prueba primero la exploración de Pista A y después la captura de pelota de Pista B.
