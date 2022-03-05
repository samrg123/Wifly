
import signal
import socket
from threading import Thread, Lock
from time import sleep

kHost = "192.168.0.162"
kPort = 1234

stdOutLock = Lock()

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def SafePrint(str):
    stdOutLock.acquire()
    print(str)
    stdOutLock.release()

def RecvData():
    while True:
        try:
            response = sock.recv(1024)
        except socket.error:
            SafePrint("Disconnected")
            break

        SafePrint("Server Says: " + response.decode('utf-8').strip())            

def Shutdown(returnCode=0):
    SafePrint("Shuting Down...")

    sock.shutdown(socket.SHUT_RDWR)
    sock.close()
    t.join()
    print("Done")
    
    exit(returnCode)

# open connection 
sock.connect((kHost, kPort))
print(f"Connected to: {kHost}:{kPort}")

# listen for incoming data
t = Thread(target=RecvData)
t.start()

# send commands
while True:

    try:
        command = input()
    except EOFError:
        Shutdown(1)
    
    if command == "exit":
        Shutdown()
    
    SafePrint(f"Sending Command: '{command}'")
    sock.sendall((command+"\n").encode())
