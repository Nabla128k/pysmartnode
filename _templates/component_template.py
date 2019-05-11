'''
Created on 2018-06-22

@author: Kevin Köck
'''

"""
example config for MyComponent:
{
    package: <package_path>
    component: MyComponent
    constructor_args: {
        my_value: "hi there"             
        # mqtt_topic: sometopic  # optional, defaults to home/<controller-id>/<component_name>/<component-count>/set
        # mqtt_topic2: sometopic # optional, defautls to home/sometopic
        # friendly_name: null # optional, friendly name shown in homeassistant gui with mqtt discovery
    }
}
"""

__updated__ = "2019-05-11"
__version__ = "1.0"

import uasyncio as asyncio
from pysmartnode import config
from pysmartnode import logging
from pysmartnode.utils.component import Component, DISCOVERY_SWITCH
import gc

####################
# choose a component name that will be used for logging (not in leightweight_log),
# a default mqtt topic that can be changed by received or local component configuration
# as well as for the component name in homeassistant.
_component_name = "MyComponent"
# define the type of the component according to the homeassistant specifications
_component_type = "switch"
####################

_log = logging.getLogger(_component_name)
_mqtt = config.getMQTT()
gc.collect()

_count = 0


class MyComponent(Component):
    def __init__(self, my_value,  # extend or shrink according to your sensor
                 mqtt_topic=None, mqtt_topic2=None,
                 friendly_name=None):
        super().__init__()
        self._command_topic = mqtt_topic or _mqtt.getDeviceTopic("{!s}/{!s}".format(_component_name, _count),
                                                                 is_request=True)
        # This will generate a topic like: home/31f29s/MyComponent/0/set

        # These calls subscribe the topics, don't use _mqtt.subscribe.
        self._subscribe(self._command_topic)
        self._subscribe(mqtt_topic2 or "home/sometopic")
        # Alternatively self._topics can be set manually and _init overriden to subscribe the way you want

        self.my_value = my_value

        # This makes it possible to use multiple instances of MyComponent
        global _count
        self._count = _count
        _count += 1

        self._frn = friendly_name  # will default to unique name in discovery if None

        gc.collect()

    async def _init(self):
        await super()._init()  # if topics should be subscribed normally
        # alternatively if e.g. the retained state topic doesn't matter, don't call super()._init
        # but use this:
        # await _mqtt.subscribe(self._command_topic, check_retained_state_topic=False)
        # await _mqtt.subscribe(self._topics[1])
        # await self._discovery()

        # Any loops can be started here for the component, this is just an example
        while True:
            await asyncio.sleep(5)
            await _mqtt.publish(self._command_topic[:-4], "ON", qos=1)  # publishing to state_topic

    async def _discovery(self):
        name = "{!s}{!s}".format(_component_name, self._count)
        component_topic = _mqtt.getDeviceTopic(name)
        # component topic could be something completely user defined. No need to follow the pattern:
        component_topic = self._command_topic[:-4]  # get the state topic of custom component topic
        friendly_name = self._frn  # define a friendly name for the homeassistant gui. Doesn't need to be unique
        await self._publishDiscovery(_component_type, component_topic, name, DISCOVERY_SWITCH, friendly_name)
        del name, component_topic, friendly_name
        gc.collect()

    async def on_message(self, topic, message, retained):
        """
        MQTTHandler is calling this async method whenever a message is received that matches
        one of the topics in self._topics (if it is a list). Differentiating between different
        topics has to be done by the component in this method.
        :param topic: str
        :param message: str/dict/list (json converted)
        :param retained: bool
        :return:
        """
        if _mqtt.matchesSubscription(topic, self._command_topic):
            print("Do something")
        else:
            # matches topic2
            print("Do something else")
        return True  # When returning True, the value of arg "message" will be published to the state topic
