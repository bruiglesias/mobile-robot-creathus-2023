#!/usr/bin/env python3

import rospy
from pyModbusTCP.client import ModbusClient
import ctypes

if __name__ == '__main__':

    rospy.init_node('read_value_clp')

    
    # Ler os ticks dos encoders - Implementação espefífica
    try:
        c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)

        while not rospy.is_shutdown():

            word1: int = c.read_holding_registers(0)[0]
            word2: int = c.read_holding_registers(1)[0]
            dword = ctypes.c_int32((word1 << 16) | (word2 & 0xFFFF)).value

            print("world1 ", word1, "world2 ",word2)
            print("encoder 1: ", dword)

    except Exception as e:
        print('Fail to connect PLC')
        print(e)
    
