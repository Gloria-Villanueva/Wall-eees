"""
Coordinador de vision para Pista A
Raspberry Pi:
- Captura imagenes con Picamera2.
- Detecta colores.
- Detecta marcadores ArUco.
- Envia un ID estable a Arduino.

Arduino conserva el control de motores y la navegacion.
"""

import time
import cv2

from picamera2 import Picamera2

from ColorDetector import ColorDetector
from ArucoDetector import ArucoDetector
from ArduinoCommunication import ArduinoCommunication


class PistaA:
    TIEMPO_ARUCO_SEGUNDOS = 3.0

    def __init__(
        self,
        mostrar_ventana=True,
        conectar_arduino=True
    ):
        self.mostrar_ventana = mostrar_ventana
        self.conectar_arduino = conectar_arduino
        self.ejecutando = False
        self.camara_iniciada = False

        self.detector_colores = ColorDetector()
        self.detector_aruco = ArucoDetector()
        self.arduino = ArduinoCommunication()

        self.ultimo_color = None

        # Seguimiento del ID que permanece visible.
        self.id_candidato = None
        self.tiempo_inicio_id = None
        self.id_confirmado = None

        self.camara = Picamera2()

        configuracion = self.camara.create_preview_configuration(
            main={
                "size": (640, 480),
                "format": "RGB888"
            }
        )

        self.camara.configure(configuracion)

    def iniciar(self):
        self.camara.start()
        self.camara_iniciada = True
        time.sleep(2)
        print("[Pista A] Cámara iniciada.")

        if self.conectar_arduino:
            if not self.arduino.conectar():
                print(
                    "[Pista A] Continuaré en modo de visión "
                    "sin comunicación serial."
                )

    def actualizar_aruco(self, marcadores):
        """
        Confirma un ID después de 3 segundos continuos.
        Envía únicamente el primer ID confirmado de la sesión.
        """
        if self.id_confirmado is not None:
            return

        # Si hay varios marcadores, selecciona el primero.
        # Más adelante podemos priorizar el de mayor tamaño.
        id_actual = (
            marcadores[0]["id"] if marcadores else None
        )

        if id_actual is None:
            self.id_candidato = None
            self.tiempo_inicio_id = None
            return

        if id_actual != self.id_candidato:
            self.id_candidato = id_actual
            self.tiempo_inicio_id = time.monotonic()

            print(
                f"[ArUco] ID {id_actual} detectado; "
                "comprobando estabilidad..."
            )
            return

        tiempo_visible = (
            time.monotonic() - self.tiempo_inicio_id
        )

        if tiempo_visible >= self.TIEMPO_ARUCO_SEGUNDOS:
            self.id_confirmado = id_actual

            print(
                f"[ArUco] ID {id_actual} confirmado "
                f"durante {tiempo_visible:.1f} segundos."
            )

            if self.arduino.conectado():
                enviado = self.arduino.enviar_id_aruco(
                    id_actual
                )

                if enviado:
                    print("[Pista A] ID enviado a Arduino.")
                else:
                    print("[Pista A] No se pudo enviar el ID.")
            else:
                print(
                    "[Pista A] ID confirmado, pero Arduino "
                    "no está conectado."
                )

    def procesar_frame(self, frame_rgb):
        if frame_rgb is None:
            return None

        # Picamera2 RGB888 -> BGR para los detectores OpenCV.
        frame_bgr = cv2.cvtColor(
            frame_rgb,
            cv2.COLOR_RGB2BGR
        )

        # Detección de color.
        resultado_color = self.detector_colores.analizar(
            frame_bgr
        )
        color = resultado_color["color"]

        if color != self.ultimo_color:
            if color is not None:
                print(f"[Color] {color}")

            self.ultimo_color = color

        # Detección ArUco.
        marcadores = self.detector_aruco.detectar(frame_bgr)
        self.actualizar_aruco(marcadores)

        # Imagen de depuración.
        vista = frame_bgr.copy()
        self.detector_aruco.dibujar(vista, marcadores)

        cv2.putText(
            vista,
            f"Color: {color or 'NINGUNO'}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        estado_aruco = (
            f"ArUco confirmado: {self.id_confirmado}"
            if self.id_confirmado is not None
            else f"ArUco candidato: {self.id_candidato}"
        )

        cv2.putText(
            vista,
            estado_aruco,
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2
        )

        return vista

    def ejecutar(self):
        try:
            self.iniciar()
            self.ejecutando = True

            print("[Pista A] Visión iniciada. Pulsa Q para salir.")

            while self.ejecutando:
                frame_rgb = self.camara.capture_array()
                vista = self.procesar_frame(frame_rgb)

                if self.mostrar_ventana and vista is not None:
                    cv2.imshow("Wall-eees - Pista A", vista)

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

        except KeyboardInterrupt:
            print("\n[Pista A] Interrupción solicitada.")

        finally:
            self.detener()

    def detener(self):
        self.ejecutando = False

        if self.camara_iniciada:
            self.camara.stop()
            self.camara_iniciada = False

        self.arduino.desconectar()
        cv2.destroyAllWindows()

        print("[Pista A] Sistema detenido.")


if __name__ == "__main__":
    robot = PistaA(
        mostrar_ventana=True,
        conectar_arduino=True
    )
    robot.ejecutar()
