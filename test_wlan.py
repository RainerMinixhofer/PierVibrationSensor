"""
Test WLAN stack of Pico2 W
"""
import sys
import time
import random
from secrets import WiFi
# pylint: disable=import-error
import network
import ubinascii
import rp2

from machine import Pin
# pylint: enable=import-error
import uasyncio as asyncio

wifi_ssid = WiFi.ssid
wifi_password = WiFi.password

print(sys.implementation)

onboard = Pin("LED", Pin.OUT, value=0)

LEDSTATE = "OFF"
RVAL = 0

# HTML template for the webpage
def webpage(value, state):
    """HTML template for the webpage"""
    html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Pico Web Server</title>
            <meta name="viewport" content="width=device-width, initial-scale=1">
        </head>
        <body>
            <h1>Raspberry Pi Pico2 W Web Server over WLAN</h1>
            <h2>Led Control</h2>
            <form action="./lighton">
                <input type="submit" value="Light on" />
            </form>
            <br>
            <form action="./lightoff">
                <input type="submit" value="Light off" />
            </form>
            <p>LED state: {state}</p>
            <h2>Fetch Random Value</h2>
            <form action="./value">
                <input type="submit" value="Fetch value" />
            </form>
            <p>Fetched value: {value}</p>
        </body>
        </html>
        """
    return str(html)

def connect_to_network(ssid, password):
    """
    Enable WLAN interface and get IP address
    """
    wlan = network.WLAN(network.STA_IF)
    rp2.country('AT')
    wlan.active(True)
    wlan.config(pm = 0xa11140) # Disable power-save mode to get faster responsivity
    mac = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()
    print("MAC address of Raspberry Pico 2W: ",mac)
    print("WLAN Channel: ",wlan.config('channel'))
    print("WLAN ESSID: ",wlan.config('essid'))
    print("WLAN TXPOWER: ",wlan.config('txpower'))
    wlan.connect(ssid, password)

    connection_timeout = 10
    while connection_timeout > 0:
        if wlan.status() < 0 or wlan.status() >= 3:
            break
        connection_timeout -= 1
        print('waiting for connection...')
        time.sleep(1)

    if wlan.status() != 3:
        raise RuntimeError('network connection failed')
    print('connected')
    status = wlan.ifconfig()
    print('ip = ' + status[0])
    return True

async def serve_client(reader, writer):
    """
    Serve webserver to switch onboard LED on and off
    """
    global LEDSTATE #pylint: disable=global-statement
    print("Client connected")
    request_line = await reader.readline()
    print("Request:", request_line)
    # We are not interested in HTTP request headers, skip them
    while await reader.readline() != b"\r\n":
        pass

    request = str(request_line, 'utf-8').split()[1]
    print('Request:', request)

    # Process the request and update variables
    if request == '/lighton?':
        print('LED on')
        onboard.value(1)
        LEDSTATE = 'ON'
    elif request == '/lightoff?':
        print('LED off')
        onboard.value(0)
        LEDSTATE = 'OFF'
    elif request == '/value?':
        global RVAL #pylint: disable=global-statement
        RVAL = random.randint(0, 20)

    response = webpage(RVAL, LEDSTATE)
    writer.write('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    writer.write(response)

    await writer.drain()
    await writer.wait_closed()
    print("Client disconnected")

async def main():
    """
    main loop
    """
    print('Connecting to Network...')
    if not connect_to_network(wifi_ssid, wifi_password):
        print('Exiting program.')
        return

    print('Setting up webserver...')
    server = asyncio.start_server(serve_client, "0.0.0.0", 80) # type: ignore
    asyncio.create_task(server)
    print('Waiting for connections...')
    while True:
        # Add other tasks that you might need to do in the loop
        await asyncio.sleep(5)
        print("heartbeat")

try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()
