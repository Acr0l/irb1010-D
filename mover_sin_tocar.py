# Importamos las librerías necesarias
import cv2 
import numpy as np
import math
import time 
import serial
from simple_pid import PID
import sys

# Importamos la clase Image de la librería PIL
from PIL import Image

# Abre la cámara
vid = cv2.VideoCapture(1, cv2.CAP_DSHOW) 

#Iniciar segunda imagen 
#im2 = cv2.imread("black.jpg")

#Es importante mencionar que el video debe estar en el mismo directorio que el script, de lo contrario, no funcionará

#obtener los fps del video
fps = vid.get(cv2.CAP_PROP_FPS)

#A partir de los fps, obtener el waikey correcto
if fps == 0:
    waitkey = 1

else:
    waitkey = int(1000/fps)


msgOn = ";" # Distancia
#msgOff = "A0;" # Ángulo
# El Ambos mensajes que estan en formato Sring deben ser transformados en un arreglo de bytes mediante la funcion .encode
msgOnEncode = str.encode(msgOn) 
#msgOffEncode = str.encode(msgOff)

# seria.Serial nos permite abrir el puerto COM deseado
#/dev/tty.IRB-G04
ser = serial.Serial("COM5",baudrate = 38400,timeout = 1)

# Cuando se abre el puerto serial con el Arduino, este siempre se reinicia por lo que hay que esperar a que inicie para enviar los mensajes
time.sleep(1)


class DosRuedasAutoController:
    def __init__(self, Kp_angle, Ki_angle, Kd_angle, Kp_distance, Ki_distance, Kd_distance):
        # Crear controladores pid
        self.pid_angle = PID(Kp_angle, Ki_angle, Kd_angle, setpoint=0)
        self.pid_distance = PID(Kp_distance, Ki_distance,
                                Kd_distance, setpoint=0)

    def update(self, target_angle, current_angle, target_distance, current_distance, dt):
        # Actualizar setpoints
        self.pid_angle.setpoint = target_angle
        self.pid_distance.setpoint = target_distance

        # Calcular output pid
        angle_control = self.pid_angle(current_angle)
        distance_control = self.pid_distance(current_distance)

        # Determinar voltaje para las ruedas
        voltage_left = distance_control + angle_control
        voltage_right = distance_control - angle_control

        # Limitar voltaje +-7
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


# Función que itera por cada lista de colores entregada, returnando máscaras.
def create_masks(img, colors):
    masks = []
    for color in colors:
        low, high = color
        mask = cv2.inRange(img, low, high)
        masks.append(mask)
    return masks

# Se combinan las máscaras entregadas
def create_combined_mask(*args):
    args = list(args)
    for i in range(len(args)):
        if i == len(args) - 1:
            break
        else:
            combined = cv2.bitwise_or(args[i], args[i+1])
            args[i+1] = combined
    return combined

# Se crea un bounding box en la imagen original y en la máscara y regresa el centro para trabajar con segmentos
def boundy_box(img_masked, bounding_box, img, text):
    x1, y1, x2, y2 = bounding_box
    cv2.rectangle(img_masked, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(img_masked, text, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(img, text, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    # Regresará el centro de cada bounding box
    return (x1+x2)//2, (y1+y2)//2

# Se regresa una lista con las bounding boxes de cada máscara
def mask_and_bound(*args):
    boxes = []
    for mask in args:
        mask_ = Image.fromarray(mask)
        bounding_box = mask_.getbbox()
        if bounding_box is not None:
            boxes.append(bounding_box)
    return boxes

# Pitágoras
def distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

# Como no se pueden utilizar más librerías, se crean funciones para trabajar con ángulos
def angle(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.atan2(y2-y1, x2-x1)

def rad_to_deg(rad):
    return rad * 180 / math.pi # En caso de que no se pueda ocupar math, por favor considerar aproximación a 3.141592653589793

KP = 0.03
KI = 0.01
KD = 0.05

KPA = 0.02
KIA = 0.0005
KDA = 0.05

controlador_robot = DosRuedasAutoController(KPA, KIA, 0, KP, 0.0, 0.0)

while(True): 
    # Se obtiene un único frame
    ret, img = vid.read() 
    # Se transforma la imagen a HSV
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    txt = ["R", "Y", "B"]
    # Color trasero del robot
    low_blue_r = np.array([100, 120, 120])
    high_blue_r = np.array([110, 255, 255])

    # Color frontal del robot
    low_red_r = np.array([170, 140, 140])
    high_red_r = np.array([180, 255, 255])

    # Color de la pelota
    low_yellow = np.array([20, 135, 135])
    high_yellow = np.array([40, 255, 255])

    # Lista de colores
    colors = [(low_red_r, high_red_r), (low_yellow, high_yellow), (low_blue_r, high_blue_r)]

    # Se crean las máscaras
    masks = create_masks(img_hsv, colors)

    # Con las máscaras se obtienen las bounding boxes
    bounding_boxes = mask_and_bound(*masks)
    
    # Se crea la máscara combinada, para poder visualizarla
    combined = create_combined_mask(*masks)

    # Se aplica la máscara a la imagen original
    img_masked = cv2.bitwise_and(img, img, mask=combined)

    # Se obtienen los centros de las bounding boxes (cada color) para trabajar segmentos
    centers = []
    for box, t in zip(bounding_boxes, txt):
        centers.append(boundy_box(img_masked, box, img, t))
    
    #Se asigna el centro de cada color
    try:
        red_c, yellow_c, blue_c = centers
    
    except:
        #red
        try:
            if red_c == None:
                red_c = (0, 0)
                print("red failed")
        
        except:
            red_c = (0, 0)
            print("red failed")
        
        #yellow
        try:
            if yellow_c == None:
                yellow_c = (0, 0)
                print("yellow failed")
        
        except:
            yellow_c = (0, 0)
            print("yellow failed")
        
        #blue

        try:
            if blue_c == None:
                blue_c = (0, 0)
                print("blue failed")
            
        except:
            blue_c = (0, 0)
            print("blue failed")

   #Se repite el proceso, ahora para encontar los arcos
    txt_arcos = ["M","verde"]

    #Lineas verdes
    low_green = np.array([40, 100, 100])
    high_green = np.array([80, 255, 255])

    # Lista de colores arcos
    colors_arcos = [(low_green, high_green)]

    # Se crean las máscaras
    masks_arcos = create_masks(img_hsv, colors_arcos)

    # Con las máscaras se obtienen las bounding boxes
    bounding_boxes_arcos = mask_and_bound(*masks_arcos)
    
    # Se aplica la máscara a la imagen original
    img_masked_arcos = cv2.bitwise_and(img, img, mask=masks_arcos[0])

    # Se obtienen los centros de las bounding boxes (cada arco) para trabajar segmentos
    centers_arcos = []
    x1a, y1a, x2a, y2a = bounding_boxes_arcos[0]
    cv2.circle(img, (x1a, (y1a+y2a)//2), 5, (0, 255, 0), -1)
    cv2.circle(img, (x2a, (y1a+y2a)//2), 5, (0, 255, 0), -1)
    cv2.circle(img, ((x1a+x2a)//2, (y1a+y2a)//2), 5, (0, 255, 0), -1)
    for box, t in zip(bounding_boxes_arcos, txt_arcos):
        centers_arcos.append(boundy_box(img_masked, box, img, t))
    
    #Se asigna el centro de cada arco
    try:
        morado_c, verde_c = centers_arcos
    
    except:
        #purple
        try:
            if morado_c == None:
                morado_c = (0, 0)
                print("morado failed")
        
        except:
            morado_c = (0, 0)
            print("morado failed")
        
        #dark blue
        try:
            if verde_c == None:
                verde_c = (0, 0)
                print("verde failed")
        
        except:
            verde_c = (0, 0)
            print("verde failed")
        
    current_angle = calculate_angle_between_points(blue_c, red_c)
    target_angle = calculate_angle_between_points(blue_c, yellow_c)

    angle1 = calculate_turn_angle(current_angle, target_angle)
    angle1 = round(angle1, 3)

    #angle1 = rad_to_deg(angle(blue_c, yellow_c)- angle(blue_c, red_c))
    #angle1 = round(angle1, 3)
    dist = distance(blue_c, yellow_c) if angle1 < 10 and angle1 > -10 else 0
    dist = round(dist, 3)
    dist_real = distance(blue_c, yellow_c)
    dist_real = round(dist_real, 3)

    #Ver si estamos cerca, si estamos cerca, parar el robot
    margen_parar_distancia = 20
    
    if dist > margen_parar_distancia:
        sys.exit(0)
        
    
    #Actualizar PID
    vleft, vright = controlador_robot.update(0, angle1, 0, dist, 0.01)

    vleft = round(vleft, 3)
    vright = round(vright, 3)

    msg = str.encode(f"L{vleft}R{vright}")

    #msg = str.encode(f"L{4}R{3}")

    #print(f"Ángulo: {angle1}, Distancia: {dist}, Distancia real: {dist_real}")


    #print(centers_arcos)

    ser.write(msg)
    #print(msg)
    time.sleep(0.3)

    cv2.putText(img, f"Angulo: {angle1}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, f"Distancia: {dist}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow("o", img)
    

    #cv2.imshow("info", im2_copy)
    
    if cv2.waitKey(waitkey) & 0xFF == ord('q'):
        ser.write("L0R0;".encode())
        break


#cerrar puerto serial
ser.close()