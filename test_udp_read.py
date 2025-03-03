import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 10000

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

print("UDP client is listening...")
while True:
    try:
        data, addr = sock.recvfrom(25) # buffer size is 1024 bytes
        print("received message: %s" % data)
    except KeyboardInterrupt:
        break