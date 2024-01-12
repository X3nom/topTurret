import socket

class Controller():
    def __init__(self,targetIp='192.168.137.170'):
        self.targetIp = targetIp
        self.turretSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.turretSock.connect((targetIp,12345))

    def send(self,data):
        self.turretSock.send(data.encode())

    def rotate(self,x,y,z):
        sendstring = f"{x}-{y}-{z}"
        self.send(sendstring)

#turretSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)