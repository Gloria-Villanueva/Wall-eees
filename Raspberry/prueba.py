
import os
import shutil
import subprocess
import sys

ARCHIVO = os.path.expanduser("~/foto_prueba.jpg")

def main():
    print("=== PRUEBA DE CAMARA RASPBERRY PI ===")

    # Buscar la herramienta de captura instalada
    camara = shutil.which("rpicam-still")

    if camara is None:
        camara = shutil.which("libcamera-still")

    if camara is None:
        print("ERROR: No se encontro una herramienta de captura.")
        print("Revisa la instalacion de rpicam-apps o libcamera.")
        sys.exit(1)

    print("Herramienta:", camara)
    print("Capturando fotografia...")

    # Tomar una fotografia
    resultado = subprocess.run(
        [
            camara,
            "--timeout", "3000",
            "--output", ARCHIVO
        ],
        capture_output=True,
        text=True
    )

    if resultado.returncode != 0:
        print("ERROR: No se pudo tomar la fotografia.")
        print(resultado.stderr)
        sys.exit(resultado.returncode)

    # Verificar el archivo generado
    if os.path.isfile(ARCHIVO) and os.path.getsize(ARCHIVO) > 0:
        print("\nEXITO: Fotografia guardada.")
        print("Ruta:", ARCHIVO)
        print("Tamano:", os.path.getsize(ARCHIVO), "bytes")
    else:
        print("ERROR: No se genero una fotografia valida.")
        sys.exit(1)

if __name__ == "__main__":
    main()
