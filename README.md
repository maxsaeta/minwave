1. Preparación del Sistema Operativo (OS)
No uses la versión de escritorio si buscas rendimiento. Necesitas cada ciclo de CPU para el procesamiento de imagen.

Imagen: Instala Raspberry Pi OS Lite (64-bit) usando Raspberry Pi Imager.

Configuración: Activa SSH y configura tu Wi-Fi desde el Imager.

Interfaz: Una vez dentro, ejecuta sudo raspi-config y asegúrate de que la Interface de Cámara (Legacy o Libcamera) esté habilitada según lo que pida tu código de GitHub.

2. Actualización y Dependencias Base
No instales librerías de Python con sudo pip. Eso rompe el sistema. Primero, los binarios:

sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-venv libopencv-dev python3-opencv git

3. El Entorno de Aislamiento (Crucial)
Para evitar conflictos entre dt_apriltags, OpenCV y las librerías de los GPIO:

Crea el directorio del proyecto: mkdir robot_car && cd robot_car

Crea el entorno virtual: python3 -m venv venv

Actívalo: source venv/bin/activate

4. Clonación y Despliegue de Código
Aquí es donde traemos tu trabajo de maxsaeta:

git clone https://github.com/maxsaeta/minwave.git .
# Instala las dependencias declaradas
pip install -r requirements.txt
# Si no tienes un requirements.txt, instala lo básico:
pip install dt-apriltags numpy pyserial

