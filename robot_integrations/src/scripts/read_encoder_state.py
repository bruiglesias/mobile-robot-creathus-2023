#!/usr/bin/env python3

from pyModbusTCP.client import ModbusClient
import ctypes
import time


while True:
    try:

        c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)

        time.sleep(1)

        registers: int = c.read_holding_registers(0, 15)

        word_1: int = c.read_holding_registers(1)[0]
        word_0: int = c.read_holding_registers(0)[0]
        ticks_encoder_1 = ctypes.c_int32((word_1 << 16) | (word_0 & 0xFFFF)).value

        word_5: int = c.read_holding_registers(5)[0]
        word_4: int = c.read_holding_registers(4)[0]
        ticks_encoder_2 = ctypes.c_int32((word_5 << 16) | (word_4 & 0xFFFF)).value

        # Encoder 1 - Direita
        # Encoder 2 - Esquerda
        print('raw: ', ticks_encoder_1, ticks_encoder_2)
        print('ajusted: ', int(ticks_encoder_1 + 540), ticks_encoder_2)
        print('registers 0-15: ', registers)

    except Exception as e:
        print('Fail to connect PLC')
        print(e)

