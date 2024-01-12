import socket

turretSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#turretSock.bind((socket.gethostname(),12345))

turretSock.connect(('192.168.137.111',12345))
while True:
    inp = input("pwm: ")
    turretSock.send(inp.encode())