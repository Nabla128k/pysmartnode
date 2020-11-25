# Author: Kevin Köck
# Copyright Kevin Köck 2017-2020 Released under the MIT license
# Created on 2020-10-23

# Original bme280 Micropython driver
# Authors: Paul Cunnane 2016, Peter Dahlebrg 2016
#
# This module borrows from the Adafruit BME280 Python library. Original
# Copyright notices are reproduced below.
#
# Those libraries were written for the Raspberry Pi. This modification is
# intended for the MicroPython and esp8266 boards.
#
# Copyright (c) 2014 Adafruit Industries
# Author: Tony DiCola
#
# Based on the BMP280 driver with BME280 changes provided by
# David J Taylor, Edinburgh (www.satsignal.eu)
#
# Based on Adafruit_I2C.py created by Kevin Townsend.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
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
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

#test

"""
example config:
{
    package: .sensors.bme280
    component: BME280
    constructor_args: {
        i2c: i2c                    # i2c object created before
        precision_temp: 2           # precision of the temperature value published
        precision_humid: 1          # precision of the humid value published
        precision_press: 2
        temp_offset: 0              # offset for temperature to compensate bad sensor reading offsets
        humid_offset: 0
        press_offset: 0             # ...
        # friendly_name_temp: null  # optional, friendly name shown in homeassistant gui with mqtt discovery
        # friendly_name_humid: null # optional, friendly name shown in homeassistant gui with mqtt discovery
        # friendly_name_press: null
    }
}
NOTE: additional constructor arguments are available from base classes, check COMPONENTS.md!
"""

__updated__ = "2020-10-23"
__version__ = "0.0"

import gc
import uasyncio as asyncio
from pysmartnode import config
from pysmartnode import logging
from pysmartnode.utils.component.sensor import ComponentSensor, SENSOR_TEMPERATURE, SENSOR_HUMIDITY, SENSOR_PRESSURE

from ustruct import unpack, unpack_from
from array import array

####################
# choose a component name that will be used for logging (not in leightweight_log) and
# a default mqtt topic that can be changed by received or local component configuration
COMPONENT_NAME = "BME280"
# define (homeassistant) value templates for all sensor readings
_VAL_T_TEMPERATURE = "{{ value_json.temperature }}"
_VAL_T_HUMIDITY = "{{ value_json.humidity }}"
_VAL_T_PRESSURE = "{{ value_json.pressure }}"
####################

_mqtt = config.getMQTT()
_log = logging.getLogger(COMPONENT_NAME)
gc.collect()

_unit_index = -1

# BME280 default address.
BME280_I2CADDR = 0x77 # Andafruit = 0x77, other? = 0x76 ?

# Operating Modes
BME280_OSAMPLE_1 = 1
BME280_OSAMPLE_2 = 2
BME280_OSAMPLE_4 = 3
BME280_OSAMPLE_8 = 4
BME280_OSAMPLE_16 = 5

BME280_REGISTER_CONTROL_HUM = 0xF2
BME280_REGISTER_CONTROL = 0xF4


class BME280(ComponentSensor):
    def __init__(self, i2c, precision_temp: int = 2, precision_humid: int = 2, precision_press: int = 2,
                 temp_offset: float = 0, humid_offset: float = 0, press_offset: float = 0,
                 friendly_name_temp=None, friendly_name_humid=None, friendly_name_press=None, 
                 mode=BME280_OSAMPLE_1, address=BME280_I2CADDR, **kwargs):
        # This makes it possible to use multiple instances of MySensor and have unique identifier
        global _unit_index
        _unit_index += 1
        super().__init__(COMPONENT_NAME, __version__, _unit_index, logger=_log, **kwargs)
        # discover: boolean, if this component should publish its mqtt discovery.
        # This can be used to prevent combined Components from exposing underlying
        # hardware components like a power switch

        if mode not in [BME280_OSAMPLE_1, BME280_OSAMPLE_2, BME280_OSAMPLE_4,
                        BME280_OSAMPLE_8, BME280_OSAMPLE_16]:
            _log.error(
                'Unexpected mode value {0}. Set mode to one of '
                'BME280_ULTRALOWPOWER, BME280_STANDARD, BME280_HIGHRES, or '
                'BME280_ULTRAHIGHRES'.format(mode))

            gc.collect()
            return None

        self._mode = mode
        self.address = address

        self.t_fine = 0
        self._l1_barray = bytearray(1)
        self._l8_barray = bytearray(8)
        self.calibration_done = False

        self.i2c = i2c
        self._addSensorType(SENSOR_TEMPERATURE, precision_temp, temp_offset, _VAL_T_TEMPERATURE,
                            "°C", friendly_name_temp)
        self._addSensorType(SENSOR_HUMIDITY, precision_humid, humid_offset, _VAL_T_HUMIDITY, "%",
                            friendly_name_humid)
        self._addSensorType(SENSOR_PRESSURE, precision_press, press_offset, _VAL_T_PRESSURE, "hPa",
                            friendly_name_press)
        gc.collect()

    async def _calibrate(self):
        try:
            # load calibration data
            dig_88_a1 = self.i2c.readfrom_mem(self.address, 0x88, 26)
            dig_e1_e7 = self.i2c.readfrom_mem(self.address, 0xE1, 7)
            self.dig_T1, self.dig_T2, self.dig_T3, self.dig_P1, \
                self.dig_P2, self.dig_P3, self.dig_P4, self.dig_P5, \
                self.dig_P6, self.dig_P7, self.dig_P8, self.dig_P9, \
                _, self.dig_H1 = unpack("<HhhHhhhhhhhhBB", dig_88_a1)

            self.dig_H2, self.dig_H3 = unpack("<hB", dig_e1_e7)
            e4_sign = unpack_from("<b", dig_e1_e7, 3)[0]
            self.dig_H4 = (e4_sign << 4) | (dig_e1_e7[4] & 0xF)

            e6_sign = unpack_from("<b", dig_e1_e7, 5)[0]
            self.dig_H5 = (e6_sign << 4) | (dig_e1_e7[4] >> 4)

            self.dig_H6 = unpack_from("<b", dig_e1_e7, 6)[0]

            self.i2c.writeto_mem(self.address, BME280_REGISTER_CONTROL,
                                 bytearray([0x3F]))
            #self.t_fine = 0

            # temporary data holders which stay allocated
            #self._l1_barray = bytearray(1)
            #self._l8_barray = bytearray(8)
            #self._l3_resultarray = array("i", [0, 0, 0])
        
            self.calibration_done = True

        except Exception as e:
            #s = io.StringIO()
            #sys.print_exception(e, s)
            await self._log.asyncLog("error", "Error setting calibration data:", e, timeout=10)
            self.calibration_done = False

            return None

    async def _read(self):
        if not self.calibration_done:
            await self._calibrate()

        else:
            raw = await self._issue_measurement_async()
            if raw:
                raw_temp, raw_press, raw_hum = raw

                # temperature
                var1 = ((raw_temp >> 3) - (self.dig_T1 << 1)) * (self.dig_T2 >> 11)
                var2 = (((((raw_temp >> 4) - self.dig_T1) *
                          ((raw_temp >> 4) - self.dig_T1)) >> 12) * self.dig_T3) >> 14
                self.t_fine = var1 + var2
                temp = (self.t_fine * 5 + 128) >> 8

                # pressure
                var1 = self.t_fine - 128000
                var2 = var1 * var1 * self.dig_P6
                var2 = var2 + ((var1 * self.dig_P5) << 17)
                var2 = var2 + (self.dig_P4 << 35)
                var1 = (((var1 * var1 * self.dig_P3) >> 8) +
                        ((var1 * self.dig_P2) << 12))
                var1 = (((1 << 47) + var1) * self.dig_P1) >> 33
                if var1 == 0:
                    pressure = 0
                else:
                    p = 1048576 - raw_press
                    p = (((p << 31) - var2) * 3125) // var1
                    var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
                    var2 = (self.dig_P8 * p) >> 19
                    pressure = ((p + var1 + var2) >> 8) + (self.dig_P7 << 4)

                # humidity
                h = self.t_fine - 76800
                h = (((((raw_hum << 14) - (self.dig_H4 << 20) -
                        (self.dig_H5 * h)) + 16384)
                      >> 15) * (((((((h * self.dig_H6) >> 10) *
                                    (((h * self.dig_H3) >> 11) + 32768)) >> 10) +
                                  2097152) * self.dig_H2 + 8192) >> 14))
                h = h - (((((h >> 15) * (h >> 15)) >> 7) * self.dig_H1) >> 4)
                h = 0 if h < 0 else h
                h = 419430400 if h > 419430400 else h
                humidity = h >> 12

            if raw is not None:                
                pressure = pressure // 256
                pi = pressure // 100
                pd = pressure - pi * 100

                hi = humidity // 1024
                hd = humidity * 100 // 1024 - hi * 100
                
                te = "{}".format(temp / 100)
                pr = "{}.{:02d}".format(pi, pd)
                hu = "{}.{:02d}".format(hi, hd)

                await self._setValue(SENSOR_TEMPERATURE, te)
                await self._setValue(SENSOR_HUMIDITY, hu)
                await self._setValue(SENSOR_PRESSURE, pr)

    async def _issue_measurement_async(self):
        try:
            self._l1_barray[0] = self._mode
            self.i2c.writeto_mem(self.address, BME280_REGISTER_CONTROL_HUM,
                                 self._l1_barray)
            self._l1_barray[0] = self._mode << 5 | self._mode << 2 | 1
            self.i2c.writeto_mem(self.address, BME280_REGISTER_CONTROL,
                                 self._l1_barray)

        except Exception as e:
            await self._log.asyncLog("error", "Error reading sensor:", e, timeout=10)
            return None

        sleep_time = 1250 + 2300 * (1 << self._mode)
        sleep_time = sleep_time + 2300 * (1 << self._mode) + 575
        sleep_time = sleep_time + 2300 * (1 << self._mode) + 575
        #time.sleep_us(sleep_time)  # Wait the required time
        await asyncio.sleep_ms(sleep_time)

        raw = [0, 0, 0]
        
        try:
            # burst readout from 0xF7 to 0xFE, recommended by datasheet
            self.i2c.readfrom_mem_into(self.address, 0xF7, self._l8_barray)
            readout = self._l8_barray
            # pressure(0xF7): ((msb << 16) | (lsb << 8) | xlsb) >> 4
            raw_press = ((readout[0] << 16) | (readout[1] << 8) | readout[2]) >> 4
            # temperature(0xFA): ((msb << 16) | (lsb << 8) | xlsb) >> 4
            raw_temp = ((readout[3] << 16) | (readout[4] << 8) | readout[5]) >> 4
            # humidity(0xFD): (msb << 8) | lsb
            raw_hum = (readout[6] << 8) | readout[7]
            raw[0] = raw_temp
            raw[1] = raw_press
            raw[2] = raw_hum
            return raw

        except Exception as e:
            await self._log.asyncLog("error", "Error reading sensor:", e, timeout=10)
            return None