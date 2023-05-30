#!/usr/bin/env python3

from pyModbusTCP.client import ModbusClient
import ctypes
import time


try:

    c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)

    time.sleep(1)

    registers: int = c.read_holding_registers(0, 15)

    print('registers 0-15: ', registers)

except Exception as e:
    print('Fail to connect PLC')
    print(e)

