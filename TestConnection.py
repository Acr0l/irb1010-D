import cv2
import time
import serial

cv2.namedWindow('frame')

baud_rate = 38400
# COMX en Windows, /dev/tty.IRB-G04 en Linux/Mac
port = "/dev/cu.IRB-G04"

def send(ser, msg):
	ser.write(msg.encode())
	print(f"Sent: {msg}")

def main():
	# Check if port is available
	try:
		ser = serial.Serial(port, baud_rate, timeout=1)
		print(f"Connected to {port}")
	except:
		print("Port not available")
		return
	
	time.sleep(5)

	while(True):
		send(ser, "A100;")
		time.sleep(5)
		send(ser, "A0;")
		time.sleep(5)
		
	
	ser.close()

if __name__ == "__main__":
	main()