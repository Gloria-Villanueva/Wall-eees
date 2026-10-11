"""
Comunicacion Raspberry Pi <-> Arduino.

Protocolo compatible con el firmware actual:
    Raspberry -> Arduino: ARUCO:<id>\\n

El firmware Arduino reconoce IDs entre 0 y 49.
Esta clase no controla motores ni lee distancias.
"""

import time
import serial
from serial.tools import list_ports


class ArduinoCommunication:
    def __init__(
        self,
        port=None,
        baudrate=115200,
        timeout=1.0
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.ultimo_id_enviado = None

    def _buscar_puerto(self):
        """Busca puertos seriales comunes si no se especificó uno."""
        puertos = [
            p.device for p in list_ports.comports()
        ]

        if self.port and self.port in puertos:
            return self.port

        if self.port:
            return self.port

        preferidos = ("/dev/ttyACM", "/dev/ttyUSB")

        for prefijo in preferidos:
            for puerto in puertos:
                if puerto.startswith(prefijo):
                    return puerto

        return None

    def conectar(self):
        """Abre el puerto serial."""
        if self.conectado():
            return True

        puerto = self._buscar_puerto()

        if puerto is None:
            print("[Arduino] No se encontró un puerto serial.")
            return False

        try:
            self.serial = serial.Serial(
                port=puerto,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )

            # Algunos microcontroladores se reinician al abrir el puerto.
            time.sleep(2)
            self.serial.reset_input_buffer()

            self.port = puerto
            self.ultimo_id_enviado = None

            print(f"[Arduino] Conectado a {puerto}")
            return True

        except serial.SerialException as error:
            self.serial = None
            print(f"[Arduino] Error de conexión: {error}")
            return False

    def conectado(self):
        return (
            self.serial is not None
            and self.serial.is_open
        )

    def enviar_id_aruco(self, marker_id):
        """
        Envía ARUCO:<id> una sola vez por sesión.

        El firmware actual conserva el primer ID válido recibido.
        Retorna True si se escribió el mensaje.
        """
        if not isinstance(marker_id, int):
            raise TypeError("El ID ArUco debe ser un entero.")

        if not 0 <= marker_id < 50:
            raise ValueError("El ID debe estar entre 0 y 49.")

        if not self.conectado():
            print("[Arduino] No conectado; ID no enviado.")
            return False

        if self.ultimo_id_enviado is not None:
            if marker_id == self.ultimo_id_enviado:
                return True

            print(
                "[Arduino] Ya se envió un ID en esta sesión. "
                "El firmware actual conserva el primer ID."
            )
            return False

        mensaje = f"ARUCO:{marker_id}\n"

        try:
            self.serial.write(mensaje.encode("utf-8"))
            self.serial.flush()

            self.ultimo_id_enviado = marker_id
            print(f"[Raspberry -> Arduino] {mensaje.strip()}")
            return True

        except serial.SerialException as error:
            print(f"[Arduino] Error al enviar ID: {error}")
            return False

    def desconectar(self):
        """Cierra la conexión serial."""
        if self.serial is not None:
            try:
                if self.serial.is_open:
                    self.serial.close()
            finally:
                self.serial = None

        print("[Arduino] Desconectado.")


if __name__ == "__main__":
    arduino = ArduinoCommunication()

    try:
        if arduino.conectar():
            print("Conexión lista. No se enviaron comandos.")
    finally:
        arduino.desconectar()
