import cv2
import numpy as np
import math
import time 
import serial
from simple_pid import PID
from PIL import Image

# Abre la cámara
vid = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Verifica si la cámara se abrió correctamente
if not vid.isOpened():
    print("Error: No se pudo abrir la cámara.")
    exit()

# Obtener los fps del video
fps = vid.get(cv2.CAP_PROP_FPS)

# A partir de los fps, obtener el waitkey correcto
if fps == 0:
    waitkey = 1
else:
    waitkey = int(1000 / fps)

msgOn = ";"  # Distancia
msgOnEncode = str.encode(msgOn)

# Configurar el puerto serial
try:
    ser = serial.Serial("/dev/tty.IRB-G04", baudrate=38400, timeout=1)
    time.sleep(1)  # Esperar a que el Arduino se reinicie
except serial.SerialException:
    print("Error: No se pudo abrir el puerto serial.")
    exit()

class DosRuedasAutoController:
    def __init__(self, Kp_angle, Ki_angle, Kd_angle, Kp_distance, Ki_distance, Kd_distance):
        self.pid_angle = PID(Kp_angle, Ki_angle, Kd_angle, setpoint=0)
        self.pid_distance = PID(Kp_distance, Ki_distance, Kd_distance, setpoint=0)

    def update(self, target_angle, current_angle, target_distance, current_distance, dt):
        self.pid_angle.setpoint = target_angle
        self.pid_distance.setpoint = target_distance
        angle_control = self.pid_angle(current_angle)
        distance_control = self.pid_distance(current_distance)
        voltage_left = distance_control + angle_control
        voltage_right = distance_control - angle_control
        voltage_left = max(min(voltage_left, 5), -5)
        voltage_right = max(min(voltage_right, 5), -5)
        return voltage_left, voltage_right

def calculate_angle_between_points(p1, p2):
    delta_x = p2[0] - p1[0]
    delta_y = p2[1] - p1[1]
    angle = math.degrees(math.atan2(delta_y, delta_x))
    return angle

def calculate_turn_angle(current_angle, target_angle):
    turn_angle = target_angle - current_angle
    while turn_angle > 180:
        turn_angle -= 360
    while turn_angle < -180:
        turn_angle += 360
    return turn_angle

def create_masks(img, colors):
    masks = []
    for color in colors:
        low, high = color
        mask = cv2.inRange(img, low, high)
        masks.append(mask)
    return masks

def create_combined_mask(*args):
    combined = np.zeros_like(args[0])
    for mask in args:
        combined = cv2.bitwise_or(combined, mask)
    return combined

def boundy_box(img_masked, bounding_box, img, text):
    x1, y1, x2, y2 = bounding_box
    cv2.rectangle(img_masked, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(img_masked, text, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(img, text, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return (x1 + x2) // 2, (y1 + y2) // 2

def mask_and_bound(*args):
    boxes = []
    for mask in args:
        mask_ = Image.fromarray(mask)
        bounding_box = mask_.getbbox()
        if bounding_box is not None:
            boxes.append(bounding_box)
    return boxes

def distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def angle(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.atan2(y2 - y1, x2 - x1)

def rad_to_deg(rad):
    return rad * 180 / math.pi

KP = 0.03
KI = 0.01
KD = 0.05

KPA = 0.02
KIA = 0.0005
KDA = 0.05

controlador_robot = DosRuedasAutoController(KPA, KIA, 0, KP, 0.0, 0.0)

while True:
    ret, img = vid.read()
    if not ret:
        print("Error: No se pudo capturar la imagen.")
        break

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    txt = ["R", "Y", "B"]
    low_blue_r = np.array([100, 120, 120])
    high_blue_r = np.array([110, 255, 255])

    low_red_r = np.array([170, 140, 140])
    high_red_r = np.array([180, 255, 255])

    low_yellow = np.array([20, 135, 135])
    high_yellow = np.array([40, 255, 255])

    colors = [(low_red_r, high_red_r), (low_yellow, high_yellow), (low_blue_r, high_blue_r)]
    masks = create_masks(img_hsv, colors)
    bounding_boxes = mask_and_bound(*masks)
    combined = create_combined_mask(*masks)
    img_masked = cv2.bitwise_and(img, img, mask=combined)

    centers = []
    for box, t in zip(bounding_boxes, txt):
        centers.append(boundy_box(img_masked, box, img, t))
    
    try:
        red_c, yellow_c, blue_c = centers
    except ValueError:
        print("Error: No se encontraron todos los colores.")
        continue

    current_angle = calculate_angle_between_points(blue_c, red_c)
    target_angle = calculate_angle_between_points(blue_c, yellow_c)

    angle1 = calculate_turn_angle(current_angle, target_angle)
    angle1 = round(angle1, 3)
    dist = distance(blue_c, yellow_c) if -10 < angle1 < 10 else 0
    dist = round(dist, 3)

    vleft, vright = controlador_robot.update(0, angle1, 0, dist, 0.01)
    vleft = round(vleft, 3)
    vright = round(vright, 3)

    msg = str.encode(f"L{vleft}R{vright}")
    ser.write(msg)

    cv2.putText(img, f"Angulo: {angle1}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, f"Distancia: {dist}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow("Output", img)

    if cv2.waitKey(waitkey) & 0xFF == ord('q'):
        ser.write("L0R0;".encode())
        break

ser.close()
vid.release()
cv2.destroyAllWindows()
