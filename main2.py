import cv2
import time
import numpy as np
from dt_apriltags import Detector
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device

# Driver de pines para Trixie
Device.pin_factory = LGPIOFactory()

# --- HARDWARE: DIRECCIÓN (Ya configurado) ---
dir_pwm = PWMOutputDevice(12)
dir_in1 = DigitalOutputDevice(20)
dir_in2 = DigitalOutputDevice(21)

# --- HARDWARE: TRACCIÓN (NUEVO) ---
trac_pwm = PWMOutputDevice(13)     # Pin PWM para velocidad
trac_in1 = DigitalOutputDevice(19)  # Sentido avance
trac_in2 = DigitalOutputDevice(26)  # Sentido retroceso

# --- CONFIGURACIÓN ---
FRAME_WIDTH = 320
CENTER_X = FRAME_WIDTH // 2
KP_STEER = 0.005  
DEADZONE = 10     

# Configuración de distancia (basada en pixeles del tag)
TAG_TARGET_WIDTH = 80  # El robot se detendrá cuando el tag mida esto de ancho (aprox. 50cm)
TRAC_SPEED = 0.4       # Velocidad de crucero (0.0 a 1.0)

at_detector = Detector(searchpath=['apriltags'], families='tag36h11', nthreads=4)

def controlar_direccion(error_x):
    if abs(error_x) < DEADZONE:
        dir_pwm.value = 0
        dir_in1.off()
        dir_in2.off()
        return

    potencia = min(abs(error_x * KP_STEER), 0.8)
    if potencia < 0.3: potencia = 0.3

    if error_x < 0: # El tag está a la izquierda
        dir_in1.off()
        dir_in2.on()
    else:           # El tag está a la derecha
        dir_in1.on()
        dir_in2.off()
    dir_pwm.value = potencia

def controlar_traccion(tag_width):
    """
    Si el tag es pequeño (está lejos), avanza.
    Si el tag es grande (está cerca), se detiene.
    """
    if tag_width == 0: # No hay tag
        trac_pwm.value = 0
        trac_in1.off()
        trac_in2.off()
        return

    if tag_width < TAG_TARGET_WIDTH:
        # Avanzar
        trac_in1.on()
        trac_in2.off()
        trac_pwm.value = TRAC_SPEED
    else:
        # Detenerse (Llegamos al objetivo)
        trac_pwm.value = 0
        trac_in1.off()
        trac_in2.off()

def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    print("Robot en línea. Buscando AprilTag para seguir...")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = at_detector.detect(gray)

            if len(detections) > 0:
                tag = detections[0]
                
                # 1. Cálculo de Dirección (Eje X)
                error_x = tag.center[0] - CENTER_X
                
                # 2. Cálculo de Distancia (Ancho del tag en pixeles)
                # Calculamos la distancia entre las esquinas superiores del tag
                tag_width = np.linalg.norm(tag.corners[0] - tag.corners[1])

                print(f"Error X: {error_x:.1f} | Tag Width: {tag_width:.1f}")

                controlar_direccion(error_x)
                controlar_traccion(tag_width)
            else:
                # Si se pierde el tag, el robot se detiene por seguridad
                controlar_direccion(0)
                controlar_traccion(0)

    except KeyboardInterrupt:
        print("\nDeteniendo robot...")
    finally:
        controlar_direccion(0)
        controlar_traccion(0)
        cap.release()

if __name__ == "__main__":
    main()
