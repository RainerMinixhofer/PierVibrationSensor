"""
Test serving a webserver through W5500 module
"""
import time
# pylint: disable=import-error
import network
from usocket import socket
from machine import Pin,SPI
# pylint: enable=import-error

led = Pin("LED", Pin.OUT, value=0)

#W5x00 chip init
def w5x00_init():
    """
    initialize W5500 ethernet stack and get IP address
    """
    spi=SPI(0,2_000_000, mosi=Pin(19),miso=Pin(16),sck=Pin(18))
    nic = network.WIZNET5K(spi,Pin(17),Pin(20)) #spi,cs,reset pin # type: ignore
    nic.active(True)
    nic.ipconfig(dhcp4=True)
    while not nic.isconnected():
        time.sleep(1)
        print("Waiting for connection...")
    mac = nic.config("mac").hex()
    print("MAC address:"+':'.join([mac[i:i+2] for i in range(0,11,2)]))
    print("IP-Address:",nic.ifconfig())
    return nic.ifconfig()

def web_page():
    """
    serve web page for toggling on board led
    """
    if led.value()==1:
        led_state="ON"
    else:
        led_state="OFF"

    html = """
    <!DOCTYPE html>
    <html lang="en">
    <head>
    <meta charset="UTF-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Raspberry Pi Pico Web server - WIZnet W5100S</title>
    </head>
    <body>
    <div align="center">
    <H1>Raspberry Pi Pico Web server & WIZnet Ethernet HAT</H1>
    <h2>Control LED</h2>
    PICO LED state: <strong>""" + led_state + """</strong>
    <p><a href="/?led=on"><button class="button">ON</button></a><br>
    </p>
    <p><a href="/?led=off"><button class="button button2">OFF</button></a><br>
    </p>
    </div>
    </body>
    </html>
    """
    return html

def await_connection(s):
    """
    webserver loop including connection handling
    """
    print(' >> Awaiting connection ...')
    try:

        conn, addr = s.accept()

        while True:
            # keep receiving commands on the open connection until the client stops sending
            straddr = str(addr)
            print(f'Connect from {straddr:s}')
            request = conn.recv(1024)
            if not request:
                print(f' >> {addr} disconnected')
                break
            else:
                request = str(request)

            led_on = request.find('/?led=on')
            led_off = request.find('/?led=off')
            if led_on == 6:
                print("LED ON")
                led.value(1)
            if led_off == 6:
                print("LED OFF")
                led.value(0)
            response = web_page()
            conn.send('HTTP/1.1 200 OK\n')
            conn.send('Connection: close\n')
            conn.send('Content-Type: text/html\n')
            lresponse=len(response)
            conn.send(f'Content-Length: {lresponse:d}\n\n')
            conn.send(response)

    except OSError as e:
        print(f' >> ERROR: {e}')

    finally:
        # apparently, context managers are currently not supported in MicroPython, therefore the connection is closed manually
        conn.close()
        print(' >> Connection closed.')

def main():
    """
    main program
    """
    ipaddr = w5x00_init()[0]
    s = socket()
    s.bind((ipaddr, 80))
    s.listen(5)

    while True:
        await_connection(s)

if __name__ == "__main__":
    main()
