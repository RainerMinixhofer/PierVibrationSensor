"""
Test program for reading and writing to the 6-axis intertial sensor MPU6500
"""
# pylint: disable=import-error
import utime
from machine import I2C, Pin
# pylint: enable=import-error
import mpu6500

i2c = I2C(0, scl=Pin(9), sda=Pin(8))
#Read all sensor data in FIFO burst mode with a sample frequency of 100Hz
#Set digital low pass filter 1 for gyro and accel.
#Gives at an internal sample frequency of 1kHz (see p. 14-15 in the datasheet)
#Gyroscope - Bandwidth: 184Hz, Delay: 2.9ms, Noise: 0.01dps/sqrt(Hz)
#Accelerometer - Bandwidth: 184Hz, Delay: 5.8ms, Noise: 0.3mg/sqrt(Hz)
#Temperature - Bandwidth: 188Hz, Delay: 1.9ms, Noise: tbd degC/sqrt(Hz)
sensor = mpu6500.MPU6500(i2c, sample_rate_divider=10, dlpf_cfg=1, a_dlpf_cfg=1)

print("MPU6500 id: " + hex(sensor.whoami))

self_test_gyro = sensor.self_test_gyro
print("SELF_TEST_GYRO: [" + ",".join([hex(self_test_gyro[i]) for i in range(3)])+"]")
self_test_accel = sensor.self_test_accel
print("SELF_TEST_GYRO: [" + ",".join([hex(self_test_accel[i]) for i in range(3)])+"]")
#sensor.offs_usr = [-10, -10, -10]
#offs_usr = sensor.offs_usr
#print("OFFS_USR set: [" + ",".join([str(offs_usr[i]) for i in range(3)])+"]")
#sensor.offs_usr = [0, 0, 0]
offs_usr = sensor.offs_usr
print("OFFS_USR reset: [" + ",".join([str(offs_usr[i]) for i in range(3)])+"]")
sample_rate_divider = sensor.samplerate_divider
print("Sample rate divider: " + hex(sample_rate_divider))
config = sensor.config
print("Configuration: " + hex(config))
int_pin_cfg = sensor.int_pin_cfg
print("Interrupt pin configuration: " + hex(int_pin_cfg))

# Read all sensor data in FIFO burst mode when fifo count is at least 3 times the sample width of the data stored in the FIFO
for _ in range(10):
    while sensor.fifo_count < 3*sensor.fifo_sample_width:
        utime.sleep_ms(1)
    data = sensor.fifo_read
    print("FIFO data: " + ",".join([str(data[i]) for i in range(len(data))]))

while True:
    acc=list(sensor.acceleration)
    print(f"ax={acc[0]:f} / ay={acc[1]:f} / az={acc[2]:f}")
    gyr=list(sensor.gyro)
    print(f"dtheta/dt={gyr[0]:.3f} deg/s / dphi/dt={gyr[1]:.3f} deg/s / dpsi/dt={gyr[2]:.3f} deg/s")
    print(f"Temperature: {sensor.temperature:2.1f} degC")

    utime.sleep_ms(1000)
