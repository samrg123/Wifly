
from time import sleep

from click import echo
from wiflyChat import WiflyConnection

def main():
    wiflyConnection = WiflyConnection("192.168.43.140", 1234)

    def Shutdown():
        wiflyConnection.Disconnect()
        exit(0)

    # send commands
    while True:
        
        try:
            wiflyConnection.SendCommand("read time accel gyro rssi", echo=False)
        except KeyboardInterrupt:
            Shutdown()

if __name__ == "__main__":
    main()