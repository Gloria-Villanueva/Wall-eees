"""
Detecta colores del suelo usando OpenCV y HSV.
Recibe una imagen BGR; no abre la camara.

Colores iniciales:
    ROJO, VERDE, CIAN, AMARILLO, NARANJA, ROSA

Los rangos HSV son provisionales y deben calibrarse
con los colores reales de la pista y la iluminacion.
"""

import cv2
import numpy as np


class ColorDetector:
    def __init__(self, area_minima=500):
        self.area_minima = area_minima

        # HSV de OpenCV:
        # H: 0-179, S: 0-255, V: 0-255.
        # Estos rangos son valores iniciales, no definitivos.
        self.rangos = {
            "ROJO": [
                ((0, 100, 60), (5, 255, 255)),
                ((170, 100, 60), (179, 255, 255)),
            ],
            "NARANJA": [
                ((6, 100, 70), (19, 255, 255)),
            ],
            "AMARILLO": [
                ((20, 90, 80), (34, 255, 255)),
            ],
            "VERDE": [
                ((35, 70, 50), (79, 255, 255)),
            ],
            "CIAN": [
                ((80, 70, 50), (100, 255, 255)),
            ],
            "ROSA": [
                ((145, 50, 70), (169, 255, 255)),
            ],
        }

        self.kernel = np.ones((5, 5), np.uint8)

    def _crear_mascara(self, hsv, rangos):
        """Combina los rangos HSV de un color."""
        mascara = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for inferior, superior in rangos:
            inferior = np.array(inferior, dtype=np.uint8)
            superior = np.array(superior, dtype=np.uint8)

            parcial = cv2.inRange(hsv, inferior, superior)
            mascara = cv2.bitwise_or(mascara, parcial)

        # Elimina puntos aislados y rellena huecos pequenos.
        mascara = cv2.morphologyEx(
            mascara, cv2.MORPH_OPEN, self.kernel
        )
        mascara = cv2.morphologyEx(
            mascara, cv2.MORPH_CLOSE, self.kernel
        )

        return mascara

    def analizar(self, frame, roi=None):
        """
        Analiza una imagen BGR.

        roi opcional:
            (x1, y1, x2, y2), coordenadas en pixeles.

        Devuelve:
            {
                "color": nombre o None,
                "area": area del contorno mayor,
                "confianza": proporcion de pixeles del color,
                "mascara": mascara del color detectado
            }
        """
        resultado_vacio = {
            "color": None,
            "area": 0.0,
            "confianza": 0.0,
            "mascara": None,
        }

        if frame is None or not isinstance(frame, np.ndarray):
            return resultado_vacio

        if frame.size == 0 or frame.ndim != 3:
            return resultado_vacio

        imagen = frame

        if roi is not None:
            x1, y1, x2, y2 = roi
            alto, ancho = frame.shape[:2]

            x1 = max(0, min(int(x1), ancho))
            x2 = max(0, min(int(x2), ancho))
            y1 = max(0, min(int(y1), alto))
            y2 = max(0, min(int(y2), alto))

            if x2 <= x1 or y2 <= y1:
                return resultado_vacio

            imagen = frame[y1:y2, x1:x2]

        hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)

        mejor_color = None
        mejor_area = 0.0
        mejor_mascara = None
        mejor_confianza = 0.0

        for color, rangos in self.rangos.items():
            mascara = self._crear_mascara(hsv, rangos)

            contornos, _ = cv2.findContours(
                mascara,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            if not contornos:
                continue

            contorno = max(contornos, key=cv2.contourArea)
            area = float(cv2.contourArea(contorno))

            if area < self.area_minima:
                continue

            # Proporcion de pixeles que coinciden con el color.
            confianza = (
                cv2.countNonZero(mascara) / mascara.size
            )

            # Elegimos el color con el contorno mas grande.
            if area > mejor_area:
                mejor_color = color
                mejor_area = area
                mejor_mascara = mascara
                mejor_confianza = confianza

        if mejor_color is None:
            return resultado_vacio

        return {
            "color": mejor_color,
            "area": mejor_area,
            "confianza": mejor_confianza,
            "mascara": mejor_mascara,
        }

    def detectar_color(self, frame, roi=None):
        """Devuelve solo el nombre del color o None."""
        return self.analizar(frame, roi)["color"]


# Compatibilidad con codigo que utilice detectar_color(frame).
_detector = ColorDetector()


def detectar_color(frame):
    return _detector.detectar_color(frame)


if __name__ == "__main__":
    print("ColorDetector listo.")
    print("Este modulo recibe imagenes; no abre la camara.")
