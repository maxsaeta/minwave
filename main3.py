import cv2
import time
import serial
import numpy as np
from dt_apriltags import Detector
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device

# --- CONFIGURACIÓN DE HARDWARE ---
Device.pin_factory = LGPIOFactory()

# Dirección (Pines: 12, 20, 21)
dir_pwm = PWMOutputDevice(12)
dir_in1 = DigitalOutputDevice(20)
dir_in2 = DigitalOutputDevice(21)

# Tracción (Pines: 13, 19, 26)
trac_pwm = PWMOutputDevice(13)
trac_in1 = DigitalOutputDevice(19)
trac_in2 = DigitalOutputDevice(26)

# --- CONFIGURACIÓN SERIAL (115200 bps) ---
try:
    ser = serial.Serial('/dev/ttyS0', 115200, timeout=0.1)
except Exception as e:
    print(f"Error abriendo Serial: {e}")
    ser = None

# --- PARÁMETROS DE NAVEGACIÓN ---
FRAME_WIDTH = 320
CENTER_X = FRAME_WIDTH // 2
KP_STEER = 0.007       # Ajustado para respuesta más firme
TAG_TARGET_WIDTH = 130 # Meta (detenerse al estar cerca)
SCAN_SPEED = 0.35      # Velocidad de búsqueda
TRAC_SPEED = 0.45      # Velocidad crucero

# Variables de Estado
target_id = None       
current_mode = 'F'     

at_detector = Detector(searchpath=['apriltags'], families='tag36h11', nthreads=4)

def detener_todo():
    dir_pwm.value = 0
    trac_pwm.value = 0
    dir_in1.off(); dir_in2.off()
    trac_in1.off(); trac_in2.off()

def girar_buscando():
    """Busca el primer ID rotando sobre su eje."""
    # Invertido si gira hacia el lado incorrecto
    dir_in1.on(); dir_in2.off(); dir_pwm.value = 0.6
    trac_in1.on(); trac_in2.off(); trac_pwm.value = SCAN_SPEED

def procesar_serial():
    global target_id, current_mode
    if ser and ser.in_waiting > 0:
        try:
            linea = ser.readline().decode('utf-8').strip()
            if ',' in linea:
                partes = linea.split(',')
                current_mode = partes[0].upper()
                target_id = int(partes[1])
                print(f">>> COMANDO RECIBIDO: ID {target_id} Modo {current_mode}")
        except:
            pass

def main():
    global target_id
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    print("Trixie Online. Autodetección activada.")

    try:
        while True:
            procesar_serial()
            
            ret, frame = cap.read()
            if not ret: break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = at_detector.detect(gray)

            # 1. LÓGICA DE ENGANCHE
            if target_id is None:
                if len(detections) > 0:
                    target_id = detections[0].tag_id
                    print(f"!!! Enganchado al ID: {target_id}")
                    my_tag = detections[0]
                else:
                    girar_buscando()
                    continue
            else:
                my_tag = next((d for d in detections if d.tag_id == target_id), None)

            # 2. LÓGICA DE MOVIMIENTO (DIRECCIONES CORREGIDAS POR SOFTWARE)
            if my_tag:
                # --- DIRECCIÓN ---
                error_x = my_tag.center[0] - CENTER_X
                if abs(error_x) > 15:
                    if error_x < 0: # Tag a la izquierda en imagen
                        # AJUSTE: Invertido para que cruce hacia el tag
                        dir_in1.on(); dir_in2.off() 
                    else:           # Tag a la derecha en imagen
                        # AJUSTE: Invertido para que cruce hacia el tag
                        dir_in1.off(); dir_in2.on()
                    dir_pwm.value = max(min(abs(error_x * KP_STEER), 0.8), 0.3)
                else:
                    dir_pwm.value = 0

                # --- TRACCIÓN ---
                tag_width = np.linalg.norm(my_tag.corners[0] - my_tag.corners[1])
                
                if tag_width < TAG_TARGET_WIDTH:
                    if current_mode == 'F':
                        # AJUSTE: Invertido para que avance hacia el tag
                        trac_in1.off(); trac_in2.on() 
                    else: # Modo B
                        # AJUSTE: Invertido para que retroceda
                        trac_in1.on(); trac_in2.off()
                    trac_pwm.value = TRAC_SPEED
                else:
                    print(f"Llegamos al ID {target_id}. Misión finalizada.")
                    detener_todo()
                    target_id = None 
            else:
                # --- SEGURIDAD ---
                if target_id is not None:
                    print(f"ID {target_id} perdido de vista. Deteniendo motores.")
                    detener_todo()
                else:
                    girar_buscando()

    except KeyboardInterrupt:
        print("\nSaliendo...")
    finally:
        detener_todo()
        cap.release()
        if ser: ser.close()

if __name__ == "__main__":
    main()
