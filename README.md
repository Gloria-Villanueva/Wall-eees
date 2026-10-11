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
    |   ├── ArduinoCommunication.py
    |   ├── ArucoDetector.py
    |   ├── ColorDetector.py
    |   ├──
    │
    └── PISTA_B/
        └── pista_b.py
```

------------------------------------------------------------------------

# Pendientes

Los siguientes puntos todavía deben definirse mediante pruebas y calibración:

-   Calcular el avance exacto por casilla
-   Calibrar los encoders
-   Determinar la relación entre pulsos y grados de giro
-   Definir los colores oficiales de la Pista B
-   Calibrar sensores de línea
-   Definir el protocolo final de comunicación Raspberry ↔ Arduino
-   Integrar y probar los módulos individualmente antes de integrarlos