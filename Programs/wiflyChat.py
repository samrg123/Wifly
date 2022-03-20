
import socket
from threading import Thread, Lock

gStdOutLock = Lock()
def SafePrint(string, end="\n", flush=False):
    gStdOutLock.acquire()
    print(string, end=end, flush=flush)
    gStdOutLock.release()

class WiflyConnection:


    def __init__(self, host, port, connectionTimeout=3):
        
        self.host = host 
        self.port = port
        self.connectionTimeout = connectionTimeout
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # open connection 
        self.sock.settimeout(self.connectionTimeout)
        try:
            self.sock.connect((self.host, self.port))
        except socket.timeout:
            SafePrint(f"Exceeded timeout ({self.connectionTimeout} sec)")
            exit(1)

        SafePrint(f"Connected to: {self.host}:{self.port}")            

        self.sock.settimeout(None)

        self.recvThread = Thread(target=self.RecvData) 
        self.recvThread.start()

    def RecvData(self):
        while True:

            c = ""
            responseStr = ""
            while c != "\n":
                responseStr+= c
                
                try:
                    c = self.sock.recv(1)
                except socket.error:
                    SafePrint("Disconnected")
                    return

                try:
                    c = c.decode('ascii')
                except UnicodeDecodeError:
                    c = "[?]"

            prefix = "<- "
            responseStr = responseStr.replace("\n", "\n" + prefix)
            SafePrint(prefix + responseStr, flush=True)

    def SendCommand(self, command, echo=True):
        if(echo):
            SafePrint(f"-> '{command}'")
        
        self.sock.sendall((command+"\n").encode())

    def Disconnect(self):
        SafePrint("Disconnecting...")
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        self.recvThread.join()
        
def main():
    wiflyConnection = WiflyConnection("192.168.43.140", 1234)

    def Shutdown(returnCode=0):
        SafePrint("Shuting Down...")
        wiflyConnection.Disconnect()
        SafePrint("Done")
        exit(returnCode)

    # send commands
    while True:
        try:
            command = input()
        except (EOFError, KeyboardInterrupt):
            Shutdown(1)
        
        if command == "exit":
            Shutdown()

        wiflyConnection.SendCommand(command)


if __name__ == "__main__":
    main()    

    