# Author: Kevin Köck
# Copyright Kevin Köck 2018-2020 Released under the MIT license
# Created on 2018-06-25

"""
example config:
{
    package: .sensors.dht22
    component: DHT22
    constructor_args: {
        pin: 4                    #pin number or label (on NodeMCU)
        precision_temp: 2         #precision of the temperature value published
        precision_humid: 1        #precision of the humid value published
        offset_temp: 0            #offset for temperature to compensate bad sensor reading offsets
        offset_humid: 0           #...
        # friendly_name: null # optional, friendly name shown in homeassistant gui with mqtt discovery
    }
}
NOTE: additional constructor arguments are available from base classes, check COMPONENTS.md!
"""

__updated__ = "2020-03-29"
__version__ = "1.2"

from pysmartnode import config
from pysmartnode import logging
import uasyncio as asyncio
from pysmartnode.components.machine.pin import Pin
from pysmartnode.utils.component.sensor import ComponentSensor, SENSOR_TEMPERATURE, SENSOR_HUMIDITY
import gc

####################
# import your library here
from dht import DHT22 as Sensor

# choose a component name that will be used for logging (not in leightweight_log) and
# a default mqtt topic that can be changed by received or local component configuration
COMPONENT_NAME = "DHT22"
# define (homeassistant) value templates for all sensor readings
_VAL_T_TEMPERATURE = "{{ value_json.temperature }}"
_VAL_T_HUMIDITY = "{{ value_json.humidity }}"
####################

_log = logging.getLogger(COMPONENT_NAME)
_mqtt = config.getMQTT()
gc.collect()

_unit_index = -1


class DHT22(ComponentSensor):
    def __init__(self, pin, precision_temp=2, precision_humid=1,
                 offset_temp=0, offset_humid=0, friendly_name_temp=None, friendly_name_humid=None,
                 **kwargs):
        # This makes it possible to use multiple instances of MySensor and have unique identifier
        global _unit_index
        _unit_index += 1
        super().__init__(COMPONENT_NAME, __version__, _unit_index, logger=_log, **kwargs)
        self._addSensorType(SENSOR_TEMPERATURE, precision_temp, offset_temp, _VAL_T_TEMPERATURE,
                            "°C", friendly_name_temp)
        self._addSensorType(SENSOR_HUMIDITY, precision_humid, offset_humid, _VAL_T_HUMIDITY, "%",
                            friendly_name_humid)
        ##############################
        # create sensor object
        self.sensor = Sensor(Pin(pin))  # add neccessary constructor arguments here
        ##############################
        gc.collect()

    async def _read(self):
        try:
            self.sensor.measure()
            await asyncio.sleep(1)
            self.sensor.measure()
        except Exception as e:
            await _log.asyncLog("error", "DHT22 is not working,", e, timeout=10)
            return None, None
        await asyncio.sleep_ms(100)  # give other tasks some time as measure() is slow and blocking
        try:
            temp = self.sensor.temperature()
            humid = self.sensor.humidity()
        except Exception as e:
            await _log.asyncLog("error", "Error reading DHT22:", e, timeout=10)
        else:
            await self._setValue(SENSOR_TEMPERATURE, temp)
            await self._setValue(SENSOR_HUMIDITY, humid)
