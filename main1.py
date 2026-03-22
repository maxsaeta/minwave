import cv2
import time
from dt_apriltags import Detector
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device

# Driver de pines para Trixie
Device.pin_factory = LGPIOFactory()

# --- HARDWARE: DIRECCIÓN ---
dir_pwm = PWMOutputDevice(12)
dir_in1 = DigitalOutputDevice(20) # Supongamos: ON para Derecha
dir_in2 = DigitalOutputDevice(21) # Supongamos: ON para Izquierda

# --- CONFIGURACIÓN DE VISIÓN ---
FRAME_WIDTH = 320
CENTER_X = FRAME_WIDTH // 2
KP_STEER = 0.005  # Ganancia Proporcional: Ajustar según la velocidad de tu motor
DEADZONE = 10     # Píxeles de tolerancia para evitar que el motor vibre en el centro

at_detector = Detector(searchpath=['apriltags'], families='tag36h11', nthreads=4)

def controlar_direccion(error_x):
    """
    Controla el motor de dirección basado en el error de píxeles.
    """
    if abs(error_x) < DEADZONE:
        # Estamos centrados, soltamos dirección
        dir_pwm.value = 0
        dir_in1.off()
        dir_in2.off()
        return

    # Calcular potencia (0.0 a 1.0)
    potencia = min(abs(error_x * KP_STEER), 0.8) # Capamos a 0.8 por seguridad
    
    # Asegurar una potencia mínima para que el motor se mueva (vencer inercia)
    if potencia < 0.3: potencia = 0.3

    if error_x < 0:
        # El tag está a la derecha del centro, girar a la derecha
        dir_in1.on();
        dir_in2.off()
    else:
        # El tag está a la izquierda del centro, girar a la izquierda
        dir_in1.off()
        dir_in2.on()

    dir_pwm.value = potencia

def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    print("Iniciando prueba de dirección... Busca el AprilTag.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = at_detector.detect(gray)

            if len(detections) > 0:
                tag = detections[0]
                # El error es la distancia del centro del tag al centro de la imagen
                # Si es positivo, el tag está a la derecha. Si es negativo, a la izquierda.
                error_x = tag.center[0] - CENTER_X

                print(f"Error X: {error_x:.2f} px | Corrigiendo...")
                controlar_direccion(error_x)
            else:
                # Si no hay tag, detener dirección por seguridad
                controlar_direccion(0)

    except KeyboardInterrupt:
        print("\nPrueba finalizada.")
    finally:
        controlar_direccion(0)
        cap.release()

if __name__ == "__main__":
    main()
