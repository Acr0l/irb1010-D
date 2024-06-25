# Importamos las librerías necesarias
import cv2 
import numpy as np
import math
import time 
import serial

# Importamos la clase Image de la librería PIL
from PIL import Image

# Abre la cámara (funcionan 0 o 1)
vid = cv2.VideoCapture(0) 

#Es importante mencionar que el video debe estar en el mismo directorio que el script, de lo contrario, no funcionará

# COM5 para windows
port = "/dev/tty.IRB-G04"
baud_rate = 38400

#obtener los fps del video
fps = vid.get(cv2.CAP_PROP_FPS)

#A partir de los fps, obtener el waikey correcto
if fps == 0:
    waitkey = 1

else:
    waitkey = int(1000/fps)


#/dev/tty.IRB-G04
ser = serial.Serial(port, baudrate = baud_rate, timeout = 1)
# Cuando se abre el puerto serial con el Arduino, este siempre se reinicia por lo que hay que esperar a que inicie para enviar los mensajes
time.sleep(2)

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

def main():
    try:
        ser = serial.Serial(port, baudrate = baud_rate, timeout = 1)
        time.sleep(5)
        print("Estableción establecidada")
    except:
        print("No se pudo establecer conexión")
        return
    
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
        
        #poner if
        red_c, yellow_c, blue_c = centers

        angle1 = rad_to_deg(angle(blue_c, yellow_c)- angle(blue_c, red_c))
        dist = distance(blue_c, yellow_c) if angle1 < 10 and angle1 > -10 else 0
        
        msg = str.encode(f"A{angle1}D{dist}")
        print(f"Distancia: {distance(blue_c, yellow_c)}")
        print(f"Ángulo: {rad_to_deg(angle(blue_c, yellow_c) - angle(blue_c, red_c))}")
        print("-----------------")
        ser.write(msg)
        cv2.imshow("o", img)
        
        if cv2.waitKey(waitkey) & 0xFF == ord('q'):
            break


    #cerrar puerto serial
    ser.close()

if __name__ == "__main__":
    main()