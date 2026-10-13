"""
Pista B - Wall-eees
Base de prueba: comprobar la cámara de la Raspberry Pi.

Esta versión NO controla motores ni envía comandos a Arduino.
Las funciones de pelota, color y navegación se completarán después.
"""

import cv2
from picamera2 import Picamera2


def iniciar_camara():
    "Configura e inicia la cámara CSI."
    camara = Picamera2()
    configuracion = camara.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    camara.configure(configuracion)
    camara.start()
    return camara


def detectar_pelota(imagen):
    """Pendiente: definir cómo se reconocerá la pelota."""
    return None


def detectar_color(imagen):
    "Pendiente: definir los colores y calibrar con la iluminación real."
    return None


def traducir_color_a_direccion(color):
    "Pendiente: convertir el color reconocido en una dirección."
    return None


def explorar():
    "Pendiente: lógica de exploración de la Pista B."
    pass


def main():
    camara = iniciar_camara()
    print("Pista B: cámara iniciada. Pulsa Q para cerrar la prueba.")

    try:
        while True:
            imagen_rgb = camara.capture_array()
            imagen_bgr = cv2.cvtColor(imagen_rgb, cv2.COLOR_RGB2BGR)

            cv2.imshow("Wall-eees - Prueba de camara Pista B", imagen_bgr)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Prueba detenida.")

    finally:
        camara.stop()
        cv2.destroyAllWindows()
        print("Camara detenida.")


if __name__ == "__main__":
    main()
