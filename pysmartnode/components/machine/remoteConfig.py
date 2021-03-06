# Author: Kevin Köck
# Copyright Kevin Köck 2019-2020 Released under the MIT license
# Created on 2019-09-15 

__updated__ = "2020-08-11"
__version__ = "0.93"

from pysmartnode.utils.component import ComponentBase
from pysmartnode import config
from pysmartnode import logging
import uasyncio as asyncio
from sys import platform
import gc
import os

COMPONENT_NAME = "remoteConfig"

_mqtt = config.getMQTT()
_log = logging.getLogger(COMPONENT_NAME)

# SPIRAM is very slow when importing modules
WAIT = 1.5 if platform == "esp8266" else (
    0.5 if os.uname() == "posix" or "(spiram)" not in os.uname().machine else 3)


class RemoteConfig(ComponentBase):
    def __init__(self, **kwargs):
        super().__init__(COMPONENT_NAME, __version__, unit_index=0, logger=_log, **kwargs)
        self._topic = "{!s}/login/{!s}/#".format(_mqtt.mqtt_home, _mqtt.client_id)
        self._icomp = None
        self._rcomp = []
        self._done = False
        self._watcher_task = asyncio.create_task(self._watcher())

    def done(self):
        return self._done

    async def _watcher(self):
        mqtt = _mqtt
        mqtt.subscribeSync(self._topic, self.on_message, self)
        try:
            while True:
                while mqtt.isconnected() is False:
                    await asyncio.sleep(1)
                if await mqtt.awaitSubscriptionsDone(await_connection=False):
                    _log.debug("waiting for config", local_only=True)
                    await _mqtt.publish(
                        "{!s}/login/{!s}/set".format(mqtt.mqtt_home, mqtt.client_id),
                        [config.VERSION, platform, WAIT])
                    gc.collect()
                else:
                    await asyncio.sleep(20)
                    continue
                for _ in range(120):
                    if mqtt.isconnected() is False:
                        break
                    await asyncio.sleep(1)
                    # so that it can be cancelled properly every second
        except asyncio.CancelledError:
            if config.DEBUG is True:
                _log.debug("_watcher cancelled", local_only=True)
        except Exception as e:
            await _log.asyncLog("error", "Error watching remoteConfig:", e)
        finally:
            await mqtt.unsubscribe(self._topic, self)
            self._done = True

    def _saveComponent(self, name, data):
        pass
        # save if save is enabled

    async def on_message(self, topic, msg, retain):
        if retain is True:
            return False
        m = memoryview(topic)
        if m[-4:] == b"/set":
            return False
        if m == memoryview(self._topic)[:-2]:
            print("received amount", msg)
            self._icomp = int(msg)
            # no return so it can end if 0 components are expected
        elif self._icomp is None:
            await _log.asyncLog("error", "Need amount of components first")
            return False
        else:
            if type(msg) != dict:
                await _log.asyncLog("error", "Received config is no dict")
                return False
            name = topic[topic.rfind("/") + 1:]
            del topic
            gc.collect()
            _log.info("received config for component", name, ":", msg, local_only=True)
            if name in self._rcomp:
                # received config already, typically happens if process was
                # interrupted by network error
                return False
            self._rcomp.append(name)
            self._saveComponent(name, msg)
            await config.registerComponent(name, msg)
        if len(self._rcomp) == self._icomp:  # received all components
            self._watcher_task.cancel()
        return False
