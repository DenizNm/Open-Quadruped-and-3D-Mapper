import socket
from time import sleep as sl

ip = '****' #IP Address of Arduino ESP8266 Printed from Arduino IDE
port = 8080
conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
conn.connect((ip,port))
counter = 0
while 1:
    data = f"10-7000-15"
    counter += 1
    conn.send(data.encode())
    print(f"IS SENT: {data}")
    sl(2)
