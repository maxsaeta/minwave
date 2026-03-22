import cv2
import time
from dt_apriltags import Detector
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device

# SOLUCIÓN PARA TRIXIE: Forzar el uso de lgpio
Device.pin_factory = LGPIOFactory()

# --- HARDWARE CONFIG ---
trac_pwm = PWMOutputDevice(13)
trac_in3 = DigitalOutputDevice(19)
trac_in4 = DigitalOutputDevice(16)
dir_pwm = PWMOutputDevice(12)
dir_in1 = DigitalOutputDevice(20)
dir_in2 = DigitalOutputDevice(21)

# --- DETECTOR CONFIG ---
at_detector = Detector(searchpath=['apriltags'], families='tag36h11', nthreads=4)

def main():
    cap = cv2.VideoCapture(0)
    # Importante: No usar cv2.imshow en Lite
    print("Iniciando navegación autónoma en OS 13...")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = at_detector.detect(gray)

            if len(detections) > 0:
                tag = detections[0]
                # Lógica de control aquí...
                print(f"Tag detectado en: {tag.center}")
            
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Deteniendo...")
    finally:
        cap.release()

if __name__ == "__main__":
    main()
