
import signal
import socket
from threading import Thread, Lock
from time import sleep

# kHost = "192.168.0.162"
kHost = "192.168.43.140"
kPort = 1234

stdOutLock = Lock()

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def SafePrint(string):
    stdOutLock.acquire()
    print(string)
    stdOutLock.release()

def RecvData():
    while True:
        try:
            response = sock.recv(1024)
        except socket.error:
            SafePrint("Disconnected")
            break

        prefix = "<- "
        
        try:
            responseStr = response.decode('ascii').strip()
        except UnicodeDecodeError:
            responseStr = f"Failed to ascii decode: '{response}'"

        responseStr = responseStr.replace("\n", "\n" + " "*len(prefix))
        SafePrint(prefix + responseStr)            

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
    
    SafePrint(f"-> '{command}'")
    sock.sendall((command+"\n").encode())
    