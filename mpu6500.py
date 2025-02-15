# Copyright (c) 2018-2023 Mika Tuupola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of  this software and associated documentation files (the "Software"), to
# deal in  the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copied of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# https://github.com/tuupola/micropython-mpu9250

"""
MicroPython I2C driver for MPU6500 6-axis motion tracking device
"""

__version__ = "0.4.0"

# pylint: disable=import-error
import ustruct
import utime
from micropython import const
# pylint: enable=import-error

_SELF_TEST_X_GYRO = const(0x00)
_SELF_TEST_X_ACCEL = const(0x0d)
_X_OFFS_USRH = const(0x13)
_SMPLRT_DIV = const(0x19)
_CONFIG = const(0x1a)
_GYRO_CONFIG = const(0x1b)
_ACCEL_CONFIG = const(0x1c)
_ACCEL_CONFIG2 = const(0x1d)
_FIFO_ENABLE = const(0x23)
_I2C_MST_CTRL = const(0x24)
_INT_PIN_CFG = const(0x37)
_INT_ENABLE = const(0x38)
_INT_STATUS = const(0x3a)
_ACCEL_XOUT_H = const(0x3b)
_TEMP_OUT_H = const(0x41)
_GYRO_XOUT_H = const(0x43)
_SIGNAL_PATH_RESET = const(0x68)
_USER_CTRL = const(0x6a)
_PWR_MGMT_1 = const(0x6B)
_PWR_MGMT_2 = const(0x6C)
_FIFO_COUNT_H = const(0x72)
_FIFO_R_W = const(0x74)
_WHO_AM_I = const(0x75)


ACCEL_FS_SEL_2G = const(0b00000000)
ACCEL_FS_SEL_4G = const(0b00001000)
ACCEL_FS_SEL_8G = const(0b00010000)
ACCEL_FS_SEL_16G = const(0b00011000)

GYRO_FS_SEL_250DPS = const(0b00)
GYRO_FS_SEL_500DPS = const(0b01)
GYRO_FS_SEL_1000DPS = const(0b01)
GYRO_FS_SEL_2000DPS = const(0b11)

_TEMP_SENSITIVITY = 333.87 # as per datasheet p. 10
_TEMP_OFFSET = 21 # as per datasheet p. 10

SF_G = 1
SF_M_S2 = 9.80665 # 1 g = 9.80665 m/s2 ie. standard gravity
SF_DEG_S = 1
SF_RAD_S = 0.017453292519943 # 1 deg/s is 0.017453292519943 rad/s

#FIFO_EN bits
TEMP_FIFO_EN = const(1<<7)
XG_FIFO_EN = const(1<<6)
YG_FIFO_EN = const(1<<5)
ZG_FIFO_EN = const(1<<4)
ACCEL_FIFO_EN = const(1<<3)
SLV2_FIFO_EN = const(1<<2)
SLV1_FIFO_EN = const(1<<1)
SLV0_FIFO_EN = const(1)
#USER_CTRL bits
DMP_EN = const(1<<7) # Enables DMP features
FIFO_EN = const(1<<6) # Enables FIFO operation mode
I2C_MST_EN = const(1<<5) # Enables I2C master I/F module
I2C_IF_DIS = const(1<<4) # Reset I2C slave module and put the serial interface in SPI mode only
DMP_RST = const(1<<3) # Resets then DMP when set to 1, auto clears
FIFO_RST = const(1<<2) # Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle of the internal 20MHz clock.
I2C_MST_RST = const(1<<1) # Reset I2C master module. Reset is asynchronous. This bit auto clears after one clock cycle of the internal 20MHz clock.
# Reset all gyro digital signal path, accel digital signal path, and temp digital signal path.
# This bit also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide.
SIG_COND_RST = const(1)

class MPU6500:
    """
    Class which provides interface to MPU6500 6-axis motion tracking device.
    i2c -- I2C bus the device is connected to.
    address -- I2C address of the device.
    gyro_disabled -- Disable the gyro.
    fs_sel -- Gyro full scale range. 0 = 250 dps, 1 = 500 dps, 2 = 1000 dps, 3 = 2000 dps.
    afs_sel -- Accel full scale range. 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g.
    accel_unit_sf -- Acceleration unit scale factor. Default is 9.80665 m/s2.
    gyro_unit_sf -- Gyro unit scale factor. Default is 0.017453292519943 rad/s.
    gyro_offset -- Gyro offset. Default is (0, 0, 0).
    fifo_enable -- FIFO enable bits. Default is ACCEL_, XG_, YG_, ZG_ and TEMP_FIFO_EN.
    sample_rate_divider -- Sample rate divider. Default is 10.
    fifo_mode -- FIFO mode. Default is False, When the FIFO is full, additional writes will be written to the FIFO, replacing the oldest data.
    dlpf_cfg -- Digital low pass filter configuration. Default is 1 which gives a bandwidth of 184 Hz and a delay of 2.9 ms.
    a_dlpf_cfg -- Accelerometer digital low pass filter configuration. Default is 1 which gives a bandwidth of 184 Hz and a delay of 5.8 ms.
    """
    def __init__(
        self, i2c, address=0x68, gyro_disabled=False,
        fs_sel = 0, afs_sel= 0,
        accel_unit_sf=SF_M_S2, gyro_unit_sf=SF_RAD_S,
        gyro_offset=(0, 0, 0),
        fifo_enable=ACCEL_FIFO_EN+XG_FIFO_EN+YG_FIFO_EN+ZG_FIFO_EN+TEMP_FIFO_EN,
        sample_rate_divider=1,
        fifo_mode=False,
        dlpf_cfg=1, a_dlpf_cfg=1
    ):
        self.i2c = i2c
        self.address = address

        # 0x70 = standalone MPU6500, 0x71 = MPU6250 SIP, 0x90 = MPU6700
        if self.whoami not in [0x71, 0x70, 0x90]:
            raise RuntimeError("MPU6500 not found in I2C bus.")

        # Power on sequence as per https://stackoverflow.com/a/60495338
        # Configure Power Management 1 to wake the IMU (don't reset)
        self._register_char(_PWR_MGMT_1, 0x00)
        # Reset, disable sleep mode
        self.reset()
        if gyro_disabled:
            self.disable_gyro()
        # Configure accelerometer and gyro sensitivities
        self._fs_sel = self._gyro_fs_sel(fs_sel)
        self._afs_sel = self._accel_fs_sel(afs_sel)
        # Set full scale range <fs_range> (in deg/s and g respectively) and
        # sensitivity scale factor <sens_sf> (in LSB/deg/s and LSB/g respectively)
        self.gyro_fs_range = [250, 500, 1000, 2000][fs_sel]
        self._gyro_sens_sf = [131, 62.5, 32.8, 16.4][fs_sel]
        self.accel_fs_range = [2, 4, 8, 16][afs_sel]
        self._accel_sens_sf = [16384, 8192, 4096, 2048][afs_sel]
        # Set the unit scale factors
        self._accel_unit_sf = accel_unit_sf
        self._gyro_unit_sf = gyro_unit_sf
        # Set the gyro offset
        self._gyro_offset = gyro_offset
        # Set the digital low pass filter configuration for the gyro and accelerometer
        self.dlpf_cfg = dlpf_cfg
        self.a_dlpf_cfg = a_dlpf_cfg
        # Set the sample rate divider and thus the sample rate
        self.samplerate_divider = sample_rate_divider
        # Enable interrupts after every sensor refresh
        self.data_ready_interrupt_enable()
        # Set the fifo mode
        self.fifo_mode = fifo_mode
        # Enable and reset the FIFO
        self.fifo_reset()
        # Configure the data pushed to the FIFO
        self.fifo_enable = fifo_enable
        # Do not wait for external sensor data
        self.wait_for_es_disable()

    def reset(self):
        """Reset the sensor."""
        reg = self._register_char(_PWR_MGMT_1) & ~(1<<7)
        self._register_char(_PWR_MGMT_1, reg | 1<<7)
        utime.sleep_ms(100)
        self.signal_path_reset()
        utime.sleep_ms(100)

    def sleep(self):
        """Put the sensor into sleep mode."""
        reg = self._register_char(_PWR_MGMT_1) & ~(1<<6)
        self._register_char(_PWR_MGMT_1, reg | 1<<6)

    def signal_path_reset(self, reset_gyro=True, reset_accel=True, reset_temp=True):
        """Reset signal paths."""
        reset = 0
        if reset_gyro:
            reset |= 1<<2
        if reset_accel:
            reset |= 1<<1
        if reset_temp:
            reset |= 1<<0
        self._register_char(_SIGNAL_PATH_RESET, reset)

    def cycle(self):
        """Cycle the sensor."""
        reg = self._register_char(_PWR_MGMT_1) & ~(0b111<<4)
        self._register_char(_PWR_MGMT_1, reg | 1<<5)

    def disable_gyro(self):
        """Disable the gyro."""
        reg = self._register_char(_PWR_MGMT_2) & ~(0b111)
        self._register_char(_PWR_MGMT_2, reg | 0b111)

    def enable_gyro(self):
        """Enable the gyro."""
        reg = self._register_char(_PWR_MGMT_2) & ~(0b111)
        self._register_char(_PWR_MGMT_2, reg)

    @property
    def acceleration_raw(self):
        """
        Acceleration measured by the accelerometer. X, Y, Z as signed shorts.
        """
        return self._register_three_shorts(_ACCEL_XOUT_H)

    def raw_to_acceleration(self, raw):
        """
        X, Y, Z acceleration in m/s^2 as floats.
        By default will return a 3-tuple of X, Y, Z axis acceleration values in m/s^2 as floats. 
        Will return values in g if constructor was provided `accel_unit_sf=SF_M_S2` parameter.
        """
        so = self._accel_sens_sf
        sf = self._accel_unit_sf

        return tuple(value / so * sf for value in raw)

    @property
    def acceleration(self):
        """
        By default will return a 3-tuple of X, Y, Z axis acceleration values in m/s^2 as floats.
        Will return values in g if constructor was provided `acce
        l_unit_sf=SF_M_S2` parameter.
        """
        return self.raw_to_acceleration(self.acceleration_raw)

    @property
    def gyro_raw(self):
        """
        phi, theta, psi measured by gyro as integers.
        """

        return tuple(self._register_three_shorts(_GYRO_XOUT_H))

    @property
    def gyro(self):
        """
        By default will return a 3-tuple of phi, theta, psi axis angular velocity values in rad/s as floats.
        Will return values in deg/s if constructor was provided `gyro_unit_sf=SF_DEG_S` parameter.
        Offset is subtracted from the raw values.
        """
        so = self._gyro_sens_sf
        sf = self._gyro_unit_sf
        ox, oy, oz = self._gyro_offset

        xyz = self.gyro_raw
        xyz = [value / so * sf for value in xyz]

        xyz[0] -= ox
        xyz[1] -= oy
        xyz[2] -= oz

        return tuple(xyz)

    @property
    def temperature(self):
        """
        Die temperature in celcius as a float.
        """
        temp = self._register_short(_TEMP_OUT_H)
        return ((temp - _TEMP_OFFSET) / _TEMP_SENSITIVITY) + _TEMP_OFFSET

    @property
    def whoami(self):
        """ Value of the whoami register. """
        return self._register_char(_WHO_AM_I)

    def calibrate(self, count=256, delay=0):
        """ Calibration routine. """
        ox, oy, oz = (0.0, 0.0, 0.0)
        self._gyro_offset = (0.0, 0.0, 0.0)
        n = float(count)

        while count:
            utime.sleep_ms(delay)
            gx, gy, gz = self.gyro
            ox += gx
            oy += gy
            oz += gz
            count -= 1

        self._gyro_offset = (ox / n, oy / n, oz / n)
        return self._gyro_offset

    @property
    def self_test_gyro(self):
        """ Perform self test for gyro. """
        return self._register_three_char(_SELF_TEST_X_GYRO)

    @property
    def self_test_accel(self):
        """ Perform self test for accel. """
        return self._register_three_char(_SELF_TEST_X_ACCEL)

    @property
    def offs_usr(self):
        """ Return the user offsets. """
        [x, y, z] = self._register_three_shorts(_X_OFFS_USRH)
        return [x, y, z]
    @offs_usr.setter
    def offs_usr(self, value):
        """ Set the user offsets. """
        self._register_three_shorts(_X_OFFS_USRH, value)

    @property
    def samplerate_divider(self):
        """ 
        Sample rate divider. Divides the internal sample rate of 1kHz by value to generate the sample rate.
        """
        reg = self._register_char(_SMPLRT_DIV)
        return reg + 1
    @samplerate_divider.setter
    def samplerate_divider(self, value):
        """
        Set the sample rate divider.
        This register is only effective when FCHOICE_B is 0b00, ACCEL_FCHOICE_B is 0b0 and 0 < DLPF_CFG < 7,
        we therefore set FCHOICE_B to 0b00, ACCEL_FCHOICE_B to 0b0 and
        if DLPF_CFG is outside of 0 < DLPF_CFG < 7 it is forced to 1.
        If another digital low pass filter setting is required, this register should be set accordingly.
        """
        assert 0 < value < 257, "divider needs to be 0 < x < 257"
        self._fchoice_b(0)
        self._a_fchoice_b(0)
        dlpf_cfg = self._dlpf_cfg()
        if dlpf_cfg == 7 | dlpf_cfg == 0:
            self._dlpf_cfg(1)
        self._register_char(_SMPLRT_DIV, value-1)

    @property
    def dlpf_cfg(self):
        """ Digital low pass filter configuration. """
        return self._dlpf_cfg()
    @dlpf_cfg.setter
    def dlpf_cfg(self, value):
        """ Set the digital low pass filter configuration. """
        assert 0 <= value <= 7, "DLPF_CFG needs to be 0 <= x <= 7"
        self._dlpf_cfg(value)

    @property
    def a_dlpf_cfg(self):
        """ Accelerometer digital low pass filter configuration. """
        return self._a_dlpf_cfg()
    @a_dlpf_cfg.setter
    def a_dlpf_cfg(self, value):
        """ Set the accelerometer digital low pass filter configuration. """
        assert 0 <= value <= 7, "DLPF_CFG needs to be 0 <= x <= 7"
        self._a_dlpf_cfg(value)

    @property
    def config(self):
        """ Configuration. """
        return self._register_char(_CONFIG)
    @config.setter
    def config(self, value):
        """ Set the configuration. """
        self._register_char(_CONFIG, value)

    @property
    def fifo_enable(self):
        """ read FIFO enable register. """
        reg = self._register_char(_FIFO_ENABLE)
        return reg

    @fifo_enable.setter
    def fifo_enable(self, value):
        """
        Set the FIFO enable register. When one of the FIFO_EN bits are set, the FIFO is enabled via the FIFO_EN register.
        """
        self._register_char(_FIFO_ENABLE, value)
        if value>0:
            self.user_control |= FIFO_EN

    def fifo_reset(self):
        """ Reset the FIFO. """
        self.user_control |= FIFO_RST

    @property
    def fifo_mode(self):
        """ FIFO mode. """
        return bool(self._register_char(_CONFIG) & (1<<6))
    @fifo_mode.setter
    def fifo_mode(self, value):
        """ Set the FIFO mode. """
        reg = self._register_char(_CONFIG) & ~(1<<6)
        self._register_char(_CONFIG, reg | (1 if value else 0))

    def fifo_disable(self):
        """ Disable the FIFO. """
        self.user_control &= ~FIFO_EN
        self._register_char(_FIFO_ENABLE, 0)

    def dmp_enable(self):
        """ Enable the DMP. """
        self.user_control |= DMP_EN

    def dmp_reset(self):
        """ Reset the DMP. """
        self.user_control |= DMP_RST

    def dmp_disable(self):
        """ Disable the DMP. """
        self.user_control &= ~DMP_EN

    def signal_condition_reset(self):
        """ Reset the signal condition. """
        self.user_control |= SIG_COND_RST

    @property
    def fifo_count(self):
        """ FIFO count. """
        return self._register_short(_FIFO_COUNT_H)

    @property
    def fifo_sample_width(self):
        """ 
        Return the sample width of the FIFO. 
        we multiply the number of bytes per sensor data by the bits set in the FIFO enable register.
        The SLV_x registers are not included in the FIFO, so we set the width to 0 for these.
        """
        channel_widths = [0, 0, 0, 6, 2, 2, 2, 2]
        bits = self.fifo_enable
        result = [width * ((bits >> i) & 1) for i, width in enumerate(channel_widths)]
        return sum(result)

    @property
    def fifo_read(self):
        """ Read the FIFO. """
        count = self.fifo_count
        if count == 0:
            return []
        return self.i2c.readfrom_mem(self.address, _FIFO_R_W, count)

    @property
    def int_pin_cfg(self):
        """ Interrupt pin configuration. """
        return self._register_char(_INT_PIN_CFG)
    @int_pin_cfg.setter
    def int_pin_cfg(self, value):
        """ Set the interrupt pin configuration. """
        self._register_char(_INT_PIN_CFG, value)

    def int_pin_active_low(self):
        """ Set the interrupt pin active low. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<7)
        self._register_char(_INT_PIN_CFG, reg)

    def int_pin_active_high(self):
        """ Set the interrupt pin active high. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<7)
        self._register_char(_INT_PIN_CFG, reg | 1<<7)

    def int_pin_open_drain(self):
        """ Set the interrupt pin open drain. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<6)
        self._register_char(_INT_PIN_CFG, reg | 1<<6)

    def int_pin_push_pull(self):
        """ Set the interrupt pin push pull. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<6)
        self._register_char(_INT_PIN_CFG, reg)

    def int_pin_latch(self):
        """ Set the interrupt pin latch. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<5)
        self._register_char(_INT_PIN_CFG, reg | 1<<5)

    def int_pin_pulse(self):
        """ Set the interrupt pin pulse. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<5)
        self._register_char(_INT_PIN_CFG, reg)

    def int_pin_any_read_clear(self):
        """ Set the interrupt pin any read clear. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<4)
        self._register_char(_INT_PIN_CFG, reg | 1<<4)

    def int_pin_read_clear(self):
        """ Set the interrupt pin read clear. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<4)
        self._register_char(_INT_PIN_CFG, reg)

    def fsync_pin_active_low(self):
        """ Set the fsync pin active low. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<3)
        self._register_char(_INT_PIN_CFG, reg)

    def fsync_pin_active_high(self):
        """ Set the fsync pin active high. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<3)
        self._register_char(_INT_PIN_CFG, reg | 1<<3)

    def fsync_pin_enable(self):
        """ Enable the fsync pin. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<2)
        self._register_char(_INT_PIN_CFG, reg | 1<<2)

    def fsync_pin_disable(self):
        """ Disable the fsync pin. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<2)
        self._register_char(_INT_PIN_CFG, reg)

    def i2c_bypass_enable(self):
        """ Enable the I2C bypass. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<1)
        self._register_char(_INT_PIN_CFG, reg | 1<<1)

    def i2c_bypass_disable(self):
        """ Disable the I2C bypass. """
        reg = self._register_char(_INT_PIN_CFG) & ~(1<<1)
        self._register_char(_INT_PIN_CFG, reg)

    @property
    def int_enable(self):
        """ Interrupt enable. """
        return self._register_char(_INT_ENABLE)
    @int_enable.setter
    def int_enable(self, value):
        """ Set the interrupt enable bits. """
        self._register_char(_INT_ENABLE, value)

    def int_enable_clear(self):
        """ Clear the interrupt enable. """
        self._register_char(_INT_ENABLE, 0)

    def wom_enable(self):
        """ Enable the wake on motion interrupt. """
        reg = self._register_char(_INT_ENABLE) & ~(1<<6)
        self._register_char(_INT_ENABLE, reg | 1<<6)

    def wom_disable(self):
        """ Disable the wake on motion interrupt. """
        reg = self._register_char(_INT_ENABLE) & ~(1<<6)
        self._register_char(_INT_ENABLE, reg)

    def fifo_overflow_enable(self):
        """ Enable the FIFO overflow interrupt. """
        reg = self._register_char(_INT_ENABLE) & ~(1<<4)
        self._register_char(_INT_ENABLE, reg | 1<<4)

    def fifo_overflow_disable(self):
        """ Disable the FIFO overflow interrupt. """
        reg = self._register_char(_INT_ENABLE) & ~(1<<4)
        self._register_char(_INT_ENABLE, reg)

    def fsync_interrupt_enable(self):
        """ Enable the FSYNC interrupt. """
        reg = self._register_char(_INT_ENABLE) & ~(1<<2)
        self._register_char(_INT_ENABLE, reg | 1<<2)

    def fsync_interrupt_disable(self):
        """ Disable the FSYNC interrupt. """
        reg = self._register_char(_INT_ENABLE) & ~(1<<2)
        self._register_char(_INT_ENABLE, reg)

    def data_ready_interrupt_enable(self):
        """ Enable the data ready interrupt. """
        reg = self._register_char(_INT_ENABLE) & ~(1<<0)
        self._register_char(_INT_ENABLE, reg | 1<<0)

    def data_ready_interrupt_disable(self):
        """ Disable the data ready interrupt. """
        reg = self._register_char(_INT_ENABLE) & ~(1<<0)
        self._register_char(_INT_ENABLE, reg)

    def wait_for_es_enable(self):
        """ Wait for external sensor data. """
        reg = self._register_char(_I2C_MST_CTRL) & ~(1<<6)
        self._register_char(_I2C_MST_CTRL, reg | 1<<6)

    def wait_for_es_disable(self):
        """ Disable waiting for external sensor data. """
        reg = self._register_char(_I2C_MST_CTRL) & ~(1<<6)
        self._register_char(_I2C_MST_CTRL, reg)

    @property
    def int_status(self):
        """ Interrupt status read only. """
        return self._register_char(_INT_STATUS)

    @property
    def wom_detected(self):
        """ Wake on motion detected. """
        return bool(self.int_status & (1<<6))

    @property
    def fifo_overflow(self):
        """ FIFO overflow detected. """
        return bool(self.int_status & (1<<4))

    @property
    def fsync_interrupt(self):
        """ FSYNC interrupt detected. """
        return bool(self.int_status & (1<<2))

    @property
    def dmp_interrupt(self):
        """ DMP interrupt detected. """
        return bool(self.int_status & (1<<1))

    @property
    def data_ready_interrupt(self):
        """ Data ready interrupt detected. """
        return bool(self.int_status & (1<<0))

    @property
    def user_control(self):
        """ Interrupt pin configuration. """
        return self._register_char(_USER_CTRL)
    @user_control.setter
    def user_control(self, value):
        """ Set the interrupt pin configuration. """
        self._register_char(_USER_CTRL, value)

    def _register_short(self, register, value=None, buf=bytearray(2)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return ustruct.unpack(">h", buf)[0]

        ustruct.pack_into(">h", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _register_three_shorts(self, register, value=None, buf=bytearray(6)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return ustruct.unpack(">hhh", buf)
        ustruct.pack_into(">hhh", buf, 0, value[0], value[1], value[2])
        return self.i2c.writeto_mem(self.address, register, buf)

    def _register_char(self, register, value=None, buf=bytearray(1)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return buf[0]

        ustruct.pack_into("<b", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _register_three_char(self, register, value=None, buf=bytearray(3)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return ustruct.unpack("<BBB", buf)
        ustruct.pack_into("<BBB", buf, 0, value[0], value[1], value[2])
        return self.i2c.writeto_mem(self.address, register, buf)

    def _dlpf_cfg(self, value=None):
        if value is None:
            return self._register_char(_CONFIG) & 0b111
        reg = self._register_char(_CONFIG) & ~(0b111)
        return self._register_char(_CONFIG, reg | value)

    def _gyro_fs_sel(self, value=None):
        if value is None:
            return (self._register_char(_GYRO_CONFIG) >> 3) & 0b11
        reg = self._register_char(_GYRO_CONFIG) & ~(0b11 << 3)
        return self._register_char(_GYRO_CONFIG, reg | (value << 3))

    def _fchoice_b(self, value=None):
        if value is None:
            return self._register_char(_GYRO_CONFIG) & 0b11
        reg = self._register_char(_GYRO_CONFIG) & ~(0b11)
        return self._register_char(_GYRO_CONFIG, reg | value)

    def _accel_fs_sel(self, value=None):
        if value is None:
            return (self._register_char(_ACCEL_CONFIG) >> 3) & 0b11
        reg = self._register_char(_ACCEL_CONFIG) & ~(0b11 << 3)
        return self._register_char(_ACCEL_CONFIG, reg | (value << 3))

    def _a_fchoice_b(self, value=None):
        if value is None:
            return (self._register_char(_ACCEL_CONFIG2) >> 3) & 0b1
        reg = self._register_char(_ACCEL_CONFIG2) & ~(0b1 << 3)
        return self._register_char(_ACCEL_CONFIG2, reg | value << 3)

    def _a_dlpf_cfg(self, value=None):
        if value is None:
            return self._register_char(_ACCEL_CONFIG2) & 0b111
        reg = self._register_char(_ACCEL_CONFIG2) & ~(0b111)
        return self._register_char(_ACCEL_CONFIG2, reg | value)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass
