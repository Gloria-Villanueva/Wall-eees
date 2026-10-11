import cv2


class ArucoDetector:
    """Detector de marcadores ArUco 4x4_50 para Pista A."""

    def __init__(self):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "OpenCV ArUco no está disponible. "
                "Instala una versión de opencv-contrib-python compatible."
            )

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        # Compatibilidad con distintas versiones de OpenCV.
        if hasattr(cv2.aruco, "ArucoDetector"):
            parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary, parameters
            )
            self.legacy = False
        else:
            self.parameters = (
                cv2.aruco.DetectorParameters_create()
            )
            self.detector = None
            self.legacy = True

    def detectar(self, frame):
        """
        Recibe un frame BGR de OpenCV.

        Retorna una lista de diccionarios:
        [
            {
                "id": 13,
                "centro": (320, 240),
                "esquinas": [...]
            }
        ]
        """
        if frame is None:
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.legacy:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.parameters
            )
        else:
            corners, ids, rejected = (
                self.detector.detectMarkers(gray)
            )

        resultados = []

        if ids is None:
            return resultados

        for corner, marker_id in zip(corners, ids.flatten()):
            points = corner.reshape(4, 2)

            center_x = int(points[:, 0].mean())
            center_y = int(points[:, 1].mean())

            resultados.append({
                "id": int(marker_id),
                "centro": (center_x, center_y),
                "esquinas": points.tolist()
            })

        return resultados

    def dibujar(self, frame, marcadores):
        """Dibuja los contornos, centros e IDs detectados."""
        for marker in marcadores:
            points = marker["esquinas"]

            for i in range(4):
                p1 = tuple(map(int, points[i]))
                p2 = tuple(map(int, points[(i + 1) % 4]))

                cv2.line(
                    frame, p1, p2, (0, 255, 0), 2
                )

            x, y = marker["centro"]

            cv2.circle(
                frame, (x, y), 5, (0, 0, 255), -1
            )

            cv2.putText(
                frame,
                f'ArUco ID: {marker["id"]}',
                (x + 10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        return frame
