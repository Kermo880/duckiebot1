#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from smbus2 import SMBus
import time
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        self.left_encoder = 0.0
        self.right_encoder = 0.0
        self.v0 = 0
        self.L = 0.1
        self.flag = 0
        self.speed = WheelsCmdStamped()
        self.bus = SMBus(1)
        self.i2c_address = 62
        self.register_address = 17

        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher('bestduckbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        rospy.Subscriber('/bestduckbot/front_center_tof_driver_node/range', Range, self.callback)
        rospy.Subscriber('/bestduckbot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.callback_left_encoder)
        rospy.Subscriber('/bestduckbot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.callback_right_encoder)

    def on_shutdown(self):
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.speed.vel_left = 0
        self.speed.vel_right = 0
        self.pub.publish(self.speed)

    def callback(self, data):
        self.distance = data.range

    def callback_left_encoder(self, data):
        self.left_encoder = data.data

    def callback_right_encoder(self, data):
        self.right_encoder = data.data

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.speed.vel_left = 0
            self.speed.vel_right = 0

            data = int('{:08b}'.format(self.bus.read_byte_data(self.i2c_address, self.register_address))[::-1], 2)

            print(f"Line follower data: {data}")

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='linefollower')

    # run node
    node.run()
    # keep spinning
    rospy.spin()