import cv2
import time
import serial
from simple_pid import PID

cv2.namedWindow('frame')

baud_rate = 38400
port = "/dev/tty.IRB-G04"

#Se inicia el PID en los dos motores
pidR = PID(1, 0.1, 0.05, setpoint=0)
pidL = PID(1, 0.1, 0.05, setpoint=0)

def send(ser, msg):
    ser.write(msg.encode())
    print(f"Sent: {msg}")

def read_velocity(ser):
    ser.write(b"GET_VEL;\n")
    line = ser.readline().decode().strip()
    if line.startswith("VEL"):
        parts = line.split(';')
        vel0 = float(parts[1])
        vel1 = float(parts[2])
        return vel0, vel1
    return None, None

def main():
    # Check if port is available
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port}")
    except:
        print("Port not available")
        return

    time.sleep(2)

    while True:
        vel0, vel1 = read_velocity(ser)
        if vel0 is None or vel1 is None:
            print("Failed to read velocities")
            continue

        #Valores a que se quiera llegar
        setpointR = 100  
        setpointL = 100 

        # Ajuste de los valores en los controladores PID
        pidR.setpoint = setpointR
        pidL.setpoint = setpointL

        # Calcular las salidas PID
        controlR = pidR(vel0)
        controlL = pidL(vel1)

        # Enviar comandos de control al Arduino
        send(ser, f"SET_RPM;{controlR};{controlL};")

        print(f"Vel0: {vel0}, ControlR: {controlR}, SetpointR: {setpointR}")
        print(f"Vel1: {vel1}, ControlL: {controlL}, SetpointL: {setpointL}")

        # Pausar un momento antes de la siguiente iteración
        #Ajustar?
        time.sleep(0.1)
        
        if cv2.waitKey(10) & 0xFF == 27:
            break

    ser.close()

if __name__ == "__main__":
    main()
