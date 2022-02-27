
import socket
from time import sleep
kHost = "192.168.0.162"
kPort = 1234

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.connect((kHost, kPort))

    print(f"Connected to: {kHost}:{kPort}")
        
    greeting = sock.recv(1024)
    print(f"Greeting: {greeting}")

    while True:   
        response = input()
        sock.sendall(response.encode())

        data = sock.recv(1024)
        print(f"Response: {data}")        
