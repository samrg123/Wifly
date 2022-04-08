
from time import sleep

from click import echo
from wiflyChat import WiflyConnection

def main():
    wiflyConnection = WiflyConnection("192.168.43.140", 1234)
    wiflyConnection.SendCommand("stream display off", echo=False)
    wiflyConnection.SendCommand("stream wifi on", echo=False)

    def Shutdown():
        wiflyConnection.SendCommand("stream wifi off", echo=False)
        wiflyConnection.SendCommand("stream display on", echo=False)
        wiflyConnection.Disconnect()
        exit(0)

    # send commands
    while True:
        
        try:
            sleep(.001)
        except KeyboardInterrupt:
            Shutdown()

if __name__ == "__main__":
    main()