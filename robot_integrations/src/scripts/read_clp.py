#!/usr/bin/env python3

import rospy
from pyModbusTCP.client import ModbusClient


if __name__ == '__main__':

    rospy.init_node('read_value_clp')

    
    # Ler os ticks dos encoders - Implementação espefífica
    try:
        c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)
        regs = c.read_holding_registers(10, 15)
        if regs:
            print(regs)
        else:
            print("Fail read values")
    except:
        print('Fail to connect PLC')
    
    rospy.spin()