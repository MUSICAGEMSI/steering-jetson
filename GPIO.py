import Jetson.GPIO as GPIO
import time
import struct
import math 

# Mapeamento dos pinos físicos da Jetson AGX Xavier
DATA_PIN = 18   # GPIO DATA (pino físico 18)
CLOCK_PIN = 16  # GPIO CLOCK (pino físico 16)

# Configuração GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(DATA_PIN, GPIO.OUT)
GPIO.setup(CLOCK_PIN, GPIO.OUT)

CLOCK_DELAY = 0.0005 

def send_angle(angle_rad):
    value = int(angle_rad * 100)
    packed = struct.pack('>h', value)

    print(f"[DEBUG] Enviando ângulo: {angle_rad:.2f} rad ({value})")

    for byte in packed:
        for i in range(8):
            bit = (byte >> (7 - i)) & 1
            GPIO.output(DATA_PIN, bit)
            GPIO.output(CLOCK_PIN, GPIO.HIGH)
            time.sleep(CLOCK_DELAY)
            GPIO.output(CLOCK_PIN, GPIO.LOW)
            time.sleep(CLOCK_DELAY)
            
try:
    while True:
        user_input = input("Digite um ângulo em GRAUS (ex: 180) ou exit")
        if user_input.lower() in ['exit']:
            break
        try:
            angle_deg = float(user_input)
            angle_rad = math.radians(angle_deg)
            send_angle(angle_rad)
        except ValueError:
            print("Entrada inválida. Digite um número (ex: 180)")

finally:
    GPIO.cleanup()
    print("FINISH")

