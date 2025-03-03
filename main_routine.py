"""
Main routine to start a webserver and serve the sensor data through it
"""
#pylint: disable=too-many-arguments, too-many-instance-attributes, too-few-public-methods
import sys
import time
import json
import socket
import struct
import array
import _thread
from secrets import WiFi
from math import floor
# import uasyncio as asyncio
# pylint: disable=import-error
import network
from machine import I2C, Pin, SPI, RTC
import rp2
# pylint: enable=import-error
import mpu6500
from ads1256lib import ADS1256

from ws_server import WebSocketServer, WebSocketClient

class RingBuffer:
    """
    Ring buffer class to store data in a circular buffer
    size: size of the ring buffer
    index: index of the newest value in the ring buffer
    retrieve_index: index of the oldest value in the ring buffer for retrieval
    """
    def __init__(self, size, datatype='h'):
        """
        Initialize the ring buffer with the given size and type.
        type is signed short integer (16 bits) by default.
        """
        self.size = size
        self.buffer =  array.array(datatype, (0 for _ in range(size)))
        self.index = 0
        self.retrieve_index = 0
        self.unread = 0
        self.is_full = False
        self.t_first_write = None
        self.t_till_full = None

    def add(self, data):
        """
        Add data to the ring buffer.
        """
        if self.t_first_write is None:
            self.t_first_write = time.ticks_ms() #pylint: disable=no-member
        self.buffer[self.index] = data
        self.index = (self.index + 1) % self.size
        if self.index == 0:
            self.is_full = True
            t = time.ticks_ms() #pylint: disable=no-member
            # time in seconds until the buffer is full
            self.t_till_full = time.ticks_diff(t, self.t_first_write)/1000 #pylint: disable=no-member
            self.t_first_write = t
        self.unread = self.unread + 1

    def get(self, count=0):
        """
        Retrieve data from the ring buffer in the correct order.
        If count is zero all values from the oldest up to the newest value (given by index) will be retrieved.
        If count is > 0 then count value starting with the oldest (not yet retrieved value) given by retrieve_index will be retrieved.
        """
        if count == 0 or count > self.size:
            if not self.is_full:
                result = self.buffer[:self.index]
            else:
                result = self.buffer[self.index:] + self.buffer[:self.index]
            self.retrieve_index = self.index
            self.unread = 0
        else:
            end_index = (self.retrieve_index + count) % self.size
            if self.retrieve_index < end_index:
                result = self.buffer[self.retrieve_index:end_index]
            else:
                result = self.buffer[self.retrieve_index:] + self.buffer[:end_index]
            self.retrieve_index = end_index
            self.unread = self.unread - count
        return result

    def read(self):
        """
        Read the last value written to the ring buffer.
        If the buffer is empty, None is returned.
        """
        if self.t_first_write is not None:
            return self.buffer[self.size-1 if self.index == 0 else self.index-1]
        return None

    def __repr__(self):
        """
        String representation of the ring buffer.
        """
        return f"RingBuffer({self.get()})"

    def samples_per_second(self):
        """
        Calculate the number of samples per second based on time between first write and time of buffer full
        """
        if self.is_full:
            return self.size/self.t_till_full
        return None

def rescale_array(data, new_count):
    """
    See Mathematica Notebook 1D_Array_Rescaling.nb for derivation of the algorithm
    array: list of integer values of length N
    m: number of values requested for the rescaled list
    """
    x =  [i*(len(data)-1)/(new_count-1) for i in range(1,new_count-1)]
    x0 = [floor(x[i]) for i in range(0,len(x))]
    x1 = [x0[i]+1 for i in range(0,len(x))]
    dx = [x[i]-x0[i] for i in range(0,len(x))]
    return [data[0]] + [round(data[x0[i]]-dx[i]*(data[x0[i]]-data[x1[i]])) for i in range(0,len(x))] + [data[-1]]

class Vibrationsensor:
    """
    Overarching class for the vibration sensor
    """
    def __init__(self, config):
        """
        Initialize the vibration sensor with the given configurations.
        If use_wlan is True, the wlan interface is activated with the given credentials ssid and password, 
        otherwise the wired ethernet interface is activated.
        """
        self.config = config
        # 3-axis acceleration sample size in bytes from MPU6500. One axis has 16 bits and we have 3 axes.
        self.acceleration_samplesize = 3*16/8
        # 2-channel seismic speed data sample size in bytes from ADS1256. One channel has 24 bits and we have 2 channels.
        self.speed_samplesize = 2*24/8
        # target sample frequency in Hz. Must be a divisor of 1000Hz
        self.fbuffer = 100
        # total time of data collection in seconds. For fbuffer = 100Hz, samplesize = 12 and tbuffer = 60s, we need 1.2kB of memory per second
        self.tbuffer = 10
        self.ax_ring_buffer = RingBuffer(self.fbuffer * self.tbuffer, 'h')
        self.ay_ring_buffer = RingBuffer(self.fbuffer * self.tbuffer, 'h')
        self.az_ring_buffer = RingBuffer(self.fbuffer * self.tbuffer, 'h')
        # For the speeds we have to take signed 32-bit integers since 24-bit signed integers supplied by the ADS1256 are not supported
        self.vx_ring_buffer = RingBuffer(self.fbuffer * self.tbuffer, 'l')
        self.vy_ring_buffer = RingBuffer(self.fbuffer * self.tbuffer, 'l')
        self.udpip = ""
        self.udpport = 10000
        self.udpstream = False
        self.udpsocket = None
        self.udppacketsize = 25
        self.server_start_time = 0

        # Initialize the LED pin for providing heartbeat of webserver
        self.led = Pin(config["Pico"]["LED"], Pin.OUT)
        self.mpu6500 = self.init_mpu6500(config["MPU6500"]["I2C"],config["MPU6500"]["SCL"],config["MPU6500"]["SDA"],config["MPU6500"]["INT"])
        self.ads1256 = self.init_ads1256(config["ADS1256"]["BPS"],config["ADS1256"]["SCK"],config["ADS1256"]["MOSI"],config["ADS1256"]["MISO"],
                                         config["ADS1256"]["CS"],config["ADS1256"]["SYNC"],config["ADS1256"]["DRDY"])
        self.nic = self.init_wiznet5k(config["WIZNET5K"]["SPIBUS"],config["WIZNET5K"]["BPS"],config["WIZNET5K"]["MISO"],config["WIZNET5K"]["CS"],
                                      config["WIZNET5K"]["SCK"],config["WIZNET5K"]["MOSI"],config["WIZNET5K"]["RESET"])
        self.nic_int_pin = config["WIZNET5K"]["INT"]
        self.network = "WLAN" if config["Network"]["WLAN"] else "Ethernet"
        self.ntpserver = config["Network"]["NTPSERVER"]
        self.ip = None
        self.mac = None
        self.iface = self.connect_to_network(config["Network"]["WLAN"], config["Network"]["SSID"], config["Network"]["PASSWORD"],
                                             config["Network"]["CONNECTION_TIMEOUT"])
        self.rtc = self.set_rtc()
        self.server = self.AppServer(self, "readings.html", self.iface)
        self.s_lock = _thread.allocate_lock()

    class ValueGenerator(WebSocketClient):
        """
        Class to generate sensor values and send them to the webserver
        """
        def __init__(self, outer_instance, conn):
            super().__init__(conn)
            self.outer_instance = outer_instance
            self.accel = [0, 0, 0]
            self.speed = [0, 0]
            self.data = []
            self.v = []
            self.vsens = 28.8
            self.instgain = 1
            self.pga = [1, 2, 4, 8, 16, 32, 64][self.outer_instance.ads1256.read_pga()]
            self.vlsb = (10 ** 6) * 2 * 2.5 / (self.vsens * self.instgain * self.pga * (2 ** 23 - 1))
            self.vmin = -self.vlsb * 2 ** 23
            self.vmax = +self.vlsb * (2 ** 23 - 1)

        def process(self):
            # process messages from the client
            message=self.connection.read()
            if message is not None and message != b'':
                print("Got websocket message:", message)
                message = json.loads(message)
                self.outer_instance.udpip = message["ip"]
                self.outer_instance.udpport = int(message["port"])
                self.outer_instance.udpstream = message["udpstream"]
                if self.outer_instance.udpstream:
                    self.outer_instance.udpsocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                elif self.outer_instance.udpsocket is not None:
                    self.outer_instance.udpsocket.close()
            # write messages to the client
            try:
                if self.outer_instance.ax_ring_buffer.t_first_write is not None:
                    self.accel = list(self.outer_instance.mpu6500.raw_to_acceleration([self.outer_instance.ax_ring_buffer.read(),
                                                                                       self.outer_instance.ay_ring_buffer.read(),
                                                                                       self.outer_instance.az_ring_buffer.read()]))
                else:
                    self.accel = [0, 0, 0]
                self.accel = [(-1 if i[0] == "-" else 1) * self.accel[ord(i[-1]) - 120] for i in ["-z", "x", "y"]]
                if self.outer_instance.vx_ring_buffer.t_first_write is not None:
                    self.speed = [self.outer_instance.vx_ring_buffer.read(), self.outer_instance.vy_ring_buffer.read()]
                else:
                    self.speed = [0, 0]
                self.v = [self.vlsb * s for s in self.speed]
                self.connection.write(json.dumps({"network":self.outer_instance.network,
                                                  "mac":self.outer_instance.mac,
                                                  "ax": f"{self.accel[0]:.2f}",
                                                  "ay": f"{self.accel[1]:.2f}",
                                                  "az": f"{self.accel[2]:.2f}",
                                                  "vx": f"{self.v[0]:.2f}",
                                                  "vy": f"{self.v[1]:.2f}",
                                                  "vmin": f"{self.vmin:.2f}",
                                                  "vmax": f"{self.vmax:.2f}"
                                                 }))
            except Exception as e: # pylint: disable=broad-except
                print(f"Error: {e}")

    class AppServer(WebSocketServer):
        """
        provide HTML page through webserver and read sensor data
        """
        def __init__(self, outer_instance, page, interface, max_connections=10):
            super().__init__(page, interface, max_connections)
            self.outer_instance = outer_instance

        def _make_client(self, conn):
            return self.outer_instance.ValueGenerator(self.outer_instance, conn)

    def start_webserver(self):
        """
        Start the webserver
        """
        self.server.start(3000)
        self.server_start_time = time.time()

    def stop_webserver(self):
        """
        Stop the webserver
        """
        self.server.stop()
        self.server_start_time = 0

    def loop_webserver(self):
        """
        Process all clients
        """
        try:
            while True:
                # Run webserver on 2nd core
                self.s_lock.acquire()
                self.server.process_all()
                self.led.toggle()
#                print("time: ", round(time.time()-self.server_start_time,1), " s")
#                print("MPU6500 Ringbuffer: i=", self.ax_ring_buffer.index, " ri=", self.ax_ring_buffer.retrieve_index,
#                      " u=", self.ax_ring_buffer.unread, " f=", self.ax_ring_buffer.is_full, " s=", self.ax_ring_buffer.size)
#                print("ADS1256 Ringbuffer: i=", self.vy_ring_buffer.index, " ri=", self.vy_ring_buffer.retrieve_index,
#                      " u=", self.vx_ring_buffer.unread, " f=", self.vx_ring_buffer.is_full, " s=", self.vx_ring_buffer.size)
#                if self.ax_ring_buffer.samples_per_second() is not None:
#                    print("MPU6500 Sample Data rate (ax): ", self.ax_ring_buffer.samples_per_second())
#                    print("MPU6500 Sample Data rate (ay): ", self.ay_ring_buffer.samples_per_second())
#                    print("MPU6500 Sample Data rate (az): ", self.az_ring_buffer.samples_per_second())
#                    print("ADS1256 Sample Data rate (vx): ", self.vx_ring_buffer.samples_per_second())
#                    print("ADS1256 Sample Data rate (vy): ", self.vy_ring_buffer.samples_per_second())
                time.sleep(1)
                # Release lock on core 2
                self.s_lock.release()
        except KeyboardInterrupt:
            pass

    def send_udp(self, datapacket, channel = 'EHN', ip = '10.0.0.141', port = 9999):
        """
        send udp package with sensor data of Channel <channel> using socket <s>
        e.g. channel 'EHN' means  100 sample per second (E), high gain (H), N-component seismograph
        to ip address <ip> and port <port>
        channel 'ENN' means 100 sample per second N-component accelerometer instead of high gain
        """
        if len(datapacket) != self.udppacketsize:
            raise ValueError(f"Data packet must have {self.udppacketsize} values")
        if len(channel) != 3:
            raise ValueError("Channel must have 3 characters")
        if self.udpsocket is None:
            raise ValueError("UDP socket is not initialized")

        timestamp = int(round(time.time(), 3)) # timestamp, in epoch seconds, down to milliseconds
        datapipe = []

        for data in datapacket:
            datapipe.append(data)
        datapipestr = ','.join(map(str, datapipe))
        packet = f"{{'{channel:s}', {timestamp:.3f} {datapipestr}}}"

        self.udpsocket.sendto(packet.encode(), (ip, port))

    def init_mpu6500(self, i2c, scl, sda, intr):
        """
        initialize MPU6500 6-axis inertial sensor
        """
        i2c = I2C(0, scl=Pin(scl), sda=Pin(sda))
        # Read all sensor data directly (no FIFO) with a sample frequency of 100Hz (defined by fbuffer=100)
        # Set digital low pass filter 1 for gyro and accel. Internal sample frequency is 1kHz.
        # Accelerometer DLPF with have Bandwidth: 184Hz, Delay: 5.8ms, Noise: 0.3mg/sqrt(Hz)
        imu = mpu6500.MPU6500(i2c, fs_sel=0, afs_sel=0, accel_unit_sf=mpu6500.SF_M_S2, gyro_disabled=True,
                              fifo_enable=0x00, sample_rate_divider=1000 // self.fbuffer, dlpf_cfg=1, a_dlpf_cfg=1)
        # Set the interrupt pin to the <intr> pin and define as input
        self.mpu6500_int_pin = Pin(intr, mode=Pin.IN, pull=None)
        # set the interrupt pin to active low similiar to the behavior of the ADS1256 data ready pin
        imu.int_pin_active_low()
        # set the interrupt pin to pulse mode to generate a pulse of 50us when data is ready
        imu.int_pin_pulse()
        # set the interrupt pin to push-pull mode
        imu.int_pin_push_pull()
        # set the interrupt pin to clear the interrupt when any register is read
        imu.int_pin_any_read_clear()
        # Disable the interrupt callback to avoid lockup at startup
        self.irq_mpu6500(False)

        print("MPU6500 id: " + hex(imu.whoami))

        return imu

    def init_ads1256(self, bps, sck, mosi, miso, cs, sync, drdy):
        """
        initialize ADS1256 low-noise ADC
        """
        # Set the interrupt pin to the <drdy> pin and define as input
        self.ads1256_int_pin = Pin(drdy, mode=Pin.IN, pull=None)
        adcspi = SPI(1,baudrate=bps, polarity=0, phase=1, bits=8, firstbit=SPI.MSB,
                    sck=Pin(sck, Pin.OUT), mosi=Pin(mosi, Pin.OUT), miso=Pin(miso, Pin.IN, pull=None))
        # Since we have only one SPI device on SPI bus 1 we can set the CS pin to permanent mode
        adc = ADS1256(adcspi, cs=Pin(cs, Pin.OUT, value=1), drdy=self.ads1256_int_pin,
                      sync_pwdn=Pin(sync, Pin.OUT, value=1), ref_voltage=2.5, pga = ADS1256.PGA1, cs_permanent=True)
        adc.reset()
        adc.wakeup()
        print("ADS1256 id: " + hex(adc.whoami()))
        # We have to set a data rate of 500 SPS because we multiplex two channels and need a net data rate of 100Hz per channel
        # The oversampling is corrected for after the ringbuffer is full by using rescale_array
        adc.write_reg(ADS1256.DRATE,ADS1256.DR500)
        reg = adc.read_reg(ADS1256.DRATE)[0]
        assert (reg == ADS1256.DR500), f"ADS1256 DRATE register should read DR500, got {reg}"
        adc.write_reg(ADS1256.MUX,ADS1256.CH0)
        reg = adc.read_reg(ADS1256.MUX)[0]
        assert (reg == ADS1256.CH0), f"ADS1256 MUX register should read CH0, got {reg}"
        # Disable the interrupt callback to avoid lockup at startup
        self.irq_ads1256(False)

        return adc

    def init_wiznet5k(self, spibus, bps, miso, cs, sck, mosi, reset):
        """
        Initialize WIZNET5K Ethernet HAT
        """
        nicspi=SPI(spibus, bps, mosi=Pin(mosi),miso=Pin(miso),sck=Pin(sck))
        nic = network.WIZNET5K(nicspi, Pin(cs), Pin(reset)) #spi,cs and reset pins # type: ignore
        return nic

    def connect_to_network(self, use_wlan, ssid=None, password=None, connection_timeout = 10):
        """
        Enable Network interface and get IP address
        If wlan = True then wlan interface is activated
        If wlan = False then the wired ethernet W5500 interface is activated
        """
        # Use WLAN
        wlan = network.WLAN(network.STA_IF)
        if use_wlan:
            self.nic.active(False)
            rp2.country('AT')
            wlan.active(True)
            wlan.config(pm = 0xa11140) # Disable power-save mode to get faster responsivity
            self.mac = network.WLAN().config('mac').hex()
            self.mac = ':'.join([self.mac[i:i+2] for i in range(0,11,2)])
            print("MAC address of Raspberry Pico 2W: "+self.mac)
            print("WLAN Channel: ",wlan.config('channel'))
            print("WLAN ESSID: ",wlan.config('essid'))
            print("WLAN TXPOWER: ",wlan.config('txpower'))
            if (ssid is None or password is None):
                raise RuntimeError('no ssid or password provided')
            wlan.connect(ssid, password)

            while connection_timeout > 0:
                if wlan.status() < 0 or wlan.status() >= 3:
                    break
                connection_timeout -= 1
                print('waiting for connection...')
                time.sleep(1)

            if wlan.status() != 3:
                raise RuntimeError('network connection failed')
            print('connected')
            ip = wlan.ifconfig()
            self.ip = ip
            print('ip = ' + self.ip[0])
            return wlan
        # Use W5500 Ethernet HAT and disable wlan
        wlan.disconnect()
        wlan.active(False)
        wlan.deinit()
        self.mac = self.nic.config("mac").hex()
        self.mac = ':'.join([self.mac[i:i+2] for i in range(0,11,2)])
        print("MAC address of W5500 Ethernet HAT: "+self.mac)
        self.nic.active(True)
        self.nic.ipconfig(dhcp4=True)
        while connection_timeout > 0:
            if self.nic.isconnected():
                break
            connection_timeout -= 1
            print('waiting for connection...')
            time.sleep(1)
        if not self.nic.isconnected():
            raise RuntimeError('network connection failed')
        print('connected')
        self.ip = self.nic.ipconfig('addr4')
        print('ip = ',self.ip[0])
        return self.nic

    def set_rtc(self):
        """
        get time for pico from NTP server and update RTC
        """
        ntp_delta = 2208988800
        ntp_query = bytearray(48)
        ntp_query[0] = 0x1B
        addr = socket.getaddrinfo(self.ntpserver, 123)[0][-1]
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.settimeout(1)
            s.sendto(ntp_query, addr)
            msg = s.recv(48)
        finally:
            s.close()
        val = struct.unpack("!I", msg[40:44])[0]
        t = val - ntp_delta
        tm = time.gmtime(t)
        rtc = RTC().datetime((tm[0], tm[1], tm[2], tm[6] + 1, tm[3], tm[4], tm[5], 0))
        return rtc

    def irq_mpu6500(self, enable):
        """
        Enable or disable the interrupt for the MPU6500 with the given enable flag.
        The interrupt is triggered on the falling edge of the interrupt pin.
        The callback function to handle data collection from the MPU6500 is read_mpu6500data.
        """
        if enable:
            self.mpu6500_int_pin.irq(trigger=Pin.IRQ_FALLING, handler=self.read_mpu6500data)
        else:
            self.mpu6500_int_pin.irq(handler=None)

    def read_mpu6500data(self, _):
        """
        Read data from MPU6500
        Callback function to handle data collection from mpu6500
        Interrupt is driven by the mpu6500 INT pin and is triggered on falling edge
        Data in ring buffer is stored in the following format: [ax, ay, az]
        """
        if self.mpu6500.data_ready_interrupt:
            data = self.mpu6500.acceleration_raw
            self.ax_ring_buffer.add(data[0])
            self.ay_ring_buffer.add(data[1])
            self.az_ring_buffer.add(data[2])
            if self.ax_ring_buffer.unread >= self.udppacketsize and self.udpstream and self.udpsocket is not None:
                self.send_udp(self.ax_ring_buffer.get(self.udppacketsize), channel = 'ENN', ip = self.udpip, port = self.udpport)
            if self.ay_ring_buffer.unread >= self.udppacketsize and self.udpstream and self.udpsocket is not None:
                self.send_udp(self.ay_ring_buffer.get(self.udppacketsize), channel = 'ENE', ip = self.udpip, port = self.udpport)
            if self.az_ring_buffer.unread >= self.udppacketsize and self.udpstream and self.udpsocket is not None:
                self.send_udp(self.az_ring_buffer.get(self.udppacketsize), channel = 'ENZ', ip = self.udpip, port = self.udpport)
            if self.vx_ring_buffer.unread >= self.udppacketsize and self.udpstream and self.udpsocket is not None:
                self.send_udp(self.vx_ring_buffer.get(self.udppacketsize), channel = 'EHN', ip = self.udpip, port = self.udpport)
            if self.vy_ring_buffer.unread >= self.udppacketsize and self.udpstream and self.udpsocket is not None:
                self.send_udp(self.vy_ring_buffer.get(self.udppacketsize), channel = 'EHE', ip = self.udpip, port = self.udpport)


    def irq_ads1256(self, enable):
        """
        Enable or disable the interrupt for the ADS1256 with the given enable flag.
        The interrupt is triggered on the falling edge of the data ready (drdy) pin.
        The callback function to handle data collection from the ADS1256 is read_ads1256data.
        """
        if enable:
            self.ads1256_int_pin.irq(trigger=Pin.IRQ_FALLING, handler=self.read_ads1256data)
        else:
            self.ads1256_int_pin.irq(handler=None)

    def read_ads1256data(self, _):
        """
        Read data from ADS1256
        Callback function to handle data collection from ads1256
        Interrupt is driven by the ads1256 DRDY pin and is triggered on falling edge
        Data in ring buffer is stored in the following format: [speedx, speedy]
        """
        if self.ads1256.drdy.value() == 0:
            cur_ch = self.ads1256.current_channel()
            nxt_ch = ADS1256.CH0 if cur_ch == ADS1256.CH7 else ADS1256.CH7
            data = self.ads1256.raw_to_signed(self.ads1256.cycle_channel(nxt_ch, ignore_drdy=True))
            # We have to take signed 32-bit integers since 24-bit signed integers are not supported by the ring buffer
            if cur_ch == ADS1256.CH0:
                self.vx_ring_buffer.add(data)
            else:
                self.vy_ring_buffer.add(data)

    def read_sensor(self):
        """
        Collect sensor data into ringbuffer memories
        We log the following channels into memory:
        - 3-axis acceleration from MPU6500. 3x16bit = 6 bytes
        - 2-channel seismic speed data from ADS1256. 2x24bit = 6 bytes
        - Gives a total of <samplesize> = 12 bytes per sample.
        The selected sample frequency is <fbuffer> = 100Hz, so we need to store 1.2 kbytes per second for all recorded channels.
        The RP2350 has 520kB of SRAM, so we can store 43,333 samples or 7.22 minutes of data which 
        is more than enough for our purposes.
        We define the total time of data collection <tbuffer> as 1 minute to limit the memory usage to 72kB.
        We do NOT use the FIFO feature of the MPU6500 which would reduce the I2C bus traffic and improve the performance 
        but complicates the readout sigificantly.
        The MPU6500 FIFO can store up to 512 bytes of data, so we can buffer up to 85 samples of 6 bytes each, which is 0.85 seconds of data.
        The ADS1256 has no FIFO, so we read the data directly from the ADC.
        """

        pass # pylint: disable=unnecessary-pass

    def detect_vibration(self):
        """Code to detect vibration"""
        pass # pylint: disable=unnecessary-pass

    def reset_sensor(self):
        """Code to reset the sensor"""
        pass # pylint: disable=unnecessary-pass

configuration = {
    "MPU6500": {
        "I2C": 0,
        "SDA": 8,
        "SCL": 9,
        "INT": 5
    },
    "ADS1256": {
        # with a master clock of 7,68MHz the maximum SPI clock is 1,9MHz (4x lower than master clock)
        # the BPS value has been optimized for minimizing the timing measurement using the timing_ads1256() function (about 2700 uS minimum)
        "BPS": 1_400_000,
        "SCK": 10,
        "MOSI": 11,
        "MISO": 12,
        "CS": 13,
        "SYNC": 14,
        "DRDY": 15
    },
    "WIZNET5K": {
        "SPIBUS": 0,
        "BPS": 2_000_000,
        "MISO": 16,
        "CS": 17,
        "SCK": 18,
        "MOSI": 19,
        "RESET": 20,
        "INT": 21
    },
    "Network": {
        "WLAN": False,
        "SSID": WiFi.ssid,
        "PASSWORD": WiFi.password,
        "CONNECTION_TIMEOUT": 10,
        "NTPSERVER": 'pool.ntp.org'
    },
    "Pico": {
        "LED": "LED"
    }
}

vibrationsensor = Vibrationsensor(configuration)

vibrationsensor.irq_mpu6500(True)
vibrationsensor.irq_ads1256(True)
vibrationsensor.start_webserver()

_thread.start_new_thread(vibrationsensor.loop_webserver(), ())
vibrationsensor.stop_webserver()
vibrationsensor.irq_mpu6500(False)
vibrationsensor.irq_ads1256(False)

sys.exit(0)
