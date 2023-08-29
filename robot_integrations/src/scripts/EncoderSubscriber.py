#!/usr/bin/env python3

import rospy
from pyModbusTCP.client import ModbusClient
import ctypes
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3Stamped, Vector3


class EncoderPublisher:
    def __init__(self):
        rospy.init_node('encoder_plc_publisher')

        self.filter_window_size = 20
        self.encoder_history_left = [0] * self.filter_window_size
        self.encoder_history_right = [0] * self.filter_window_size

        # Parameters
        self.R = 0.1016
        self.TPR_L = 680
        self.TPR_R = 1680
        self.PI = 3.14159265358979323846

        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0

        self.encoder_raw_pub = rospy.Publisher("/data/encoder", Vector3Stamped, queue_size=1)
        self.encoder_filtered_pub = rospy.Publisher("/data/encoder/filtered", Vector3Stamped, queue_size=1)
        self.encoder_tick_pub = rospy.Publisher("/data/tick_encoder", Vector3, queue_size=1)

        self.c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)
        self.last_time_callback = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.rate = rospy.Rate(50)  # Taxa de publicação de 20 Hz

    def apply_filter_left(self, value):
        self.encoder_history_left.append(value)
        self.encoder_history_left = self.encoder_history_left[1:]
        filtered_value = sum(self.encoder_history_left) / self.filter_window_size
        return filtered_value

    def apply_filter_right(self, value):
        self.encoder_history_right.append(value)
        self.encoder_history_right = self.encoder_history_right[1:]
        filtered_value = sum(self.encoder_history_right) / self.filter_window_size
        return filtered_value

    def encoder_callback(self):
        current_time = rospy.Time.now()

        word_1: int = self.c.read_holding_registers(1)[0]
        word_0: int = self.c.read_holding_registers(0)[0]
        ticks_encoder_1 = ctypes.c_int32((word_1 << 16) | (word_0 & 0xFFFF)).value

        word_5: int = self.c.read_holding_registers(5)[0]
        word_4: int = self.c.read_holding_registers(4)[0]
        ticks_encoder_2 = ctypes.c_int32((word_5 << 16) | (word_4 & 0xFFFF)).value

        #print(ticks_encoder_1, ticks_encoder_2)
        # ** TODO: VERIFICAR SE OS ENCONDERS ESTÃO CORRETOS
        self.left_ticks = ticks_encoder_1
        self.right_ticks = ticks_encoder_2

        dt = (current_time - self.last_time_callback).to_sec()

        # Pulblicar os ticks das rodas para debug
        tick = Vector3()
        tick.x = self.left_ticks
        tick.y = self.right_ticks
        tick.z = dt

        self.encoder_tick_pub.publish(tick)
        self.last_time_callback = current_time

        #rospy.loginfo('Raw Ticks encoder_left: %d, encoder_right: %d', self.left_ticks, self.right_ticks)

    def publish_raw_encoder(self, vel_left_raw, vel_right_raw):

        encoder_raw = Vector3Stamped()
        encoder_raw.header.frame_id = "data_encoder_raw"
        encoder_raw.vector.x = vel_left_raw
        encoder_raw.vector.y = vel_right_raw
        self.encoder_raw_pub.publish(encoder_raw)

        #rospy.loginfo('Raw Velocity - encoder_raw_left: %lf, encoder_raw_right: %lf', vel_left_raw, vel_right_raw)

    def publish_filtered_encoder(self, vel_left_filtered, vel_right_filtered):

        encoder_filtered = Vector3Stamped()
        encoder_filtered.header.frame_id = "data_encoder_filtered"
        encoder_filtered.vector.x = vel_left_filtered
        encoder_filtered.vector.y = vel_right_filtered
        self.encoder_filtered_pub.publish(encoder_filtered)

        #rospy.loginfo('Filter Velocity - encoder_filtered_left: %lf, encoder_filtered_right: %lf', vel_left_filtered, vel_right_filtered)

    def run(self):
        while not rospy.is_shutdown():
            try:
                current_time = rospy.Time.now()
                self.encoder_callback()

                delta_L = self.left_ticks - self.last_left_ticks
                delta_R = self.right_ticks - self.last_right_ticks

                resolution_left = (2 * self.PI) / self.TPR_L  # rad
                resolution_right = (2 * self.PI) / self.TPR_R  # rad

                dt = (current_time - self.last_time).to_sec()

                vl = (resolution_left * delta_L * self.R) / dt  # m/s
                vr = (resolution_right * delta_R * self.R) / dt  # m/s

                self.publish_raw_encoder(vl, vr)

                # Aplica o filtro de média móvel aos valores de encoder
                vel_left_filtered = self.apply_filter_left(vl)
                vel_right_filtered = self.apply_filter_right(vr)

                self.publish_filtered_encoder(vel_left_filtered, vel_right_filtered)

                self.last_left_ticks = self.left_ticks
                self.last_right_ticks = self.right_ticks
                self.last_time = current_time
                self.rate.sleep()

            except Exception as e:
                print('Fail to connect PLC')
                print(e)


if __name__ == '__main__':
    try:
        encoder_publisher = EncoderPublisher()
        encoder_publisher.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(e)
