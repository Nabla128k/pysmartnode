'''
Created on 09.03.2018

@author: Kevin Köck
'''
##
# Configuration management file
##

__updated__ = "2018-09-18"

from config import *
from sys import platform

# General
VERSION = const(401)
print("PySmartNode version {!s} started".format(VERSION))

import gc
import sys
import time

if DEBUG:
    def __printRAM(start, info=""):
        print(info, "Mem free", gc.mem_free(), "diff:", gc.mem_free() - start)
else:
    __printRAM = lambda *_: None

_mem = gc.mem_free()

from pysmartnode.utils import sys_vars

id = sys_vars.getDeviceID()
gc.collect()
__printRAM(_mem, "Imported .sys_vars")

import uasyncio as asyncio

LEN_ASYNC_QUEUE = 16 if platform == "esp8266" else 32
loop = asyncio.get_event_loop(runq_len=LEN_ASYNC_QUEUE, waitq_len=LEN_ASYNC_QUEUE)

gc.collect()
__printRAM(_mem, "Imported uasyncio")

gc.collect()
__printRAM(_mem, "Imported os")
from pysmartnode import logging

gc.collect()
_log = logging.getLogger("config")

gc.collect()
__printRAM(_mem, "Imported logging")

from pysmartnode.networking.mqtt import MQTTHandler, Lock  # Lock possibly needed by other modules

gc.collect()
__printRAM(_mem, "Imported MQTTHandler")

COMPONENTS = {}
COMPONENTS["mqtt"] = MQTTHandler(MQTT_RECEIVE_CONFIG)
gc.collect()
__printRAM(_mem, "Created MQTT")


async def registerComponentsAsync(data):
    _log.debug("RAM before import registerComponents: {!s}".format(gc.mem_free()))
    import pysmartnode.utils.registerComponents
    gc.collect()
    _log.debug("RAM after import registerComponents: {!s}".format(gc.mem_free()))
    await pysmartnode.utils.registerComponents.registerComponentsAsync(data, _log, LEN_ASYNC_QUEUE)
    _log.debug("RAM before deleting registerComponents: {!s}".format(gc.mem_free()))
    del pysmartnode.utils.registerComponents
    del sys.modules["pysmartnode.utils.registerComponents"]
    gc.collect()
    _log.debug("RAM after deleting registerComponents: {!s}".format(gc.mem_free()))


async def loadComponentsFile():
    _log.debug("RAM before import registerComponents: {!s}".format(gc.mem_free()))
    import pysmartnode.utils.registerComponents
    gc.collect()
    _log.debug("RAM after import registerComponents: {!s}".format(gc.mem_free()))
    await pysmartnode.utils.registerComponents.loadComponentsFile(_log)
    _log.debug("RAM before deleting registerComponents: {!s}".format(gc.mem_free()))
    del pysmartnode.utils.registerComponents
    del sys.modules["pysmartnode.utils.registerComponents"]
    gc.collect()
    _log.debug("RAM after deleting registerComponents: {!s}".format(gc.mem_free()))


def getComponent(name):
    if name in COMPONENTS:
        return COMPONENTS[name]
    else:
        return None


def addComponent(name, obj):
    if name in COMPONENTS:
        raise ValueError("Component {!s} already registered, can't add".format(name))
    COMPONENTS[name] = obj


def getMQTT():
    if "mqtt" in COMPONENTS:
        return COMPONENTS["mqtt"]
    return None
