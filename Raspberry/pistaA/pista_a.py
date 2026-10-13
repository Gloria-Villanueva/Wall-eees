"""Pista A: detectar ArUco y enviar su ID al Arduino."""

import time
import cv2
import serial
from picamera2 import Picamera2

# Cambia este puerto si tu Arduino aparece con otro nombre.
PUERTO_ARDUINO = "/dev/ttyUSB0"
BAUDIOS = 115200
DICCIONARIO = cv2.aruco.DICT_4X4_50


def crear_detector():
    diccionario = cv2.aruco.getPredefinedDictionary(DICCIONARIO)
    parametros = cv2.aruco.DetectorParameters_create()
    return diccionario, parametros


def main():
    arduino = serial.Serial(PUERTO_ARDUINO, BAUDIOS, timeout=0.01)
    time.sleep(2)  # Da tiempo a que se estabilice la conexión serial.

    camara = Picamera2()
    camara.configure(camara.create_video_configuration(
        main={"size": (640, 480), "format": "BGR888"}
    ))
    camara.start()

    diccionario, parametros = crear_detector()
    modo_aruco = False
    id_enviado = False

    print("Pista A lista. Esperando MODE:ARUCO del Arduino...")

    try:
        while True:
            # Leer mensajes enviados por el Arduino.
            while arduino.in_waiting:
                mensaje = arduino.readline().decode("utf-8", errors="ignore").strip()
                if mensaje == "MODE:ARUCO":
                    modo_aruco = True
                    print("Modo ArUco activado.")

            imagen = camara.capture_array()
            gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
            esquinas, ids, _ = cv2.aruco.detectMarkers(
                gris, diccionario, parameters=parametros
            )

            if modo_aruco and not id_enviado and ids is not None:
                id_detectado = int(ids.flatten()[0])
                if 0 <= id_detectado < 50:
                    arduino.write(f"ARUCO:{id_detectado}\n".encode("utf-8"))
                    print(f"ArUco detectado y enviado: {id_detectado}")
                    id_enviado = True

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Programa detenido.")
    finally:
        camara.stop()
        arduino.close()


if __name__ == "__main__":
    main()
