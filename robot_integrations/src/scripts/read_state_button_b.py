#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from pyModbusTCP.client import ModbusClient


def read_state_and_publisher():

    rospy.init_node('state_button_b_publisher', anonymous=True)

    address = 18
    # address 17 - botão A
    # address 18 - botão B
    # address 19 - botão carregador

    c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)

    # Criando um publicador para o tópico /botao1
    pub = rospy.Publisher('/buttonB', Int32, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            state: int = c.read_holding_registers(address)[0]
            pub.publish(state)
        except Exception as e:
            rospy.logerr("Erro ao ler endereço : %s", str(address))

        rate.sleep()

if __name__ == '__main__':
    try:
        read_state_and_publisher()
    except rospy.ROSInterruptException:
        pass
