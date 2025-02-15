import socket as s
host = ""
port = 10001                             # Port to bind to
sock = s.socket(s.AF_INET, s.SOCK_DGRAM)
sock.setsockopt(s.SOL_SOCKET, s.SO_REUSEADDR, 1)
sock.setblocking(0)
sock.bind((host, port))
data = []

print("Waiting for data on Port:", port)

while 1:                                # loop forever
    try:
        data, addr = sock.recvfrom(1024)    # wait to receive data
    except KeyboardInterrupt:
        pass
    except BlockingIOError:
        pass
    if data != []:
        print(data)
