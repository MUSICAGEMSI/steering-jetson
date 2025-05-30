import serial

# Configurar porta serial correta (ex: /dev/ttyUSB0 ou /dev/ttyTHS1)
ser = serial.Serial('/dev/ttyUSB0', 115200)

print("Digite o ângulo desejado entre -180 a 180 (graus):")

while True:
    try:
        cmd = input(">> ")
        angle = float(cmd)  # valida se é número
        if -180 <= angle <= 180:
            ser.write(f"{angle}\n".encode('utf-8'))
        else:
            print("Erro: ângulo fora do intervalo [-180, 180]")
    except Exception as e:
        print(f"Erro: {e}")
