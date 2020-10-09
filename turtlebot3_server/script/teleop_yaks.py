# Copyright (c) 2014,2020 ADLINK Technology Inc.
#
# See the NOTICE file(s) distributed with this work for additional
# information regarding copyright ownership.
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
# which is available at https://www.apache.org/licenses/LICENSE-2.0.
#
# SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#
# Author: Gabriele Baldoni

import os
import select
import sys
import tty
import signal

from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import QoSProfile

from zenoh import Zenoh
import time
import json


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.22
ANG_VEL_STEP_SIZE = 0.71

TURTLEBOT3_MODEL = 'burger'

CONTROL_RESOURCE = '/turtlebot/move'
STATE_RESOURCE = '/turtlebot/status'

class Controller():
    def __init__(self, locator):
        self.zenoh = Zenoh({'mode':'client','peer':locator})
        self.ws = self.zenoh.workspace()
        rclpy.init()
        self.qos = QoSProfile(depth=10)
        self.node = rclpy.create_node('teleop_yaks')
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', self.qos)
        self.running = False
        self.target_linear_velocity   = 0.0
        self.target_angular_velocity  = 0.0
        self.control_linear_velocity  = 0.0
        self.control_angular_velocity = 0.0


    def listener(self, kvs):
        for kv in kvs:
            print('>> [Subscription listener] Received PUT : "{}"'.format(kv))
            v = kv.get_value().get_value()
            self.move(v)


    def move(self, v):
        d = {
            'fwd':self.fwd,
            'bwd':self.bwd,
            'h':self.halt,
            'sx':self.sx,
            'dx':self.dx
        }
        f = d.get(v, None)
        if f is not None:
            f()



    def fwd(self):
        self.target_linear_velocity = check_linear_limit_velocity(self.target_linear_velocity + LIN_VEL_STEP_SIZE)
        self.send_vel()

    def bwd(self):
        self.target_linear_velocity = check_linear_limit_velocity(self.target_linear_velocity - LIN_VEL_STEP_SIZE)
        self.send_vel()

    def halt(self):
        self.target_linear_velocity   = 0.0
        self.target_angular_velocity  = 0.0
        self.control_linear_velocity  = 0.0
        self.control_angular_velocity = 0.0
        self.send_vel()

    def sx(self):
        self.target_angular_velocity = check_angular_limit_velocity(self.target_angular_velocity + ANG_VEL_STEP_SIZE)
        self.send_vel()

    def dx(self):
        self.target_angular_velocity = check_angular_limit_velocity(self.target_angular_velocity - ANG_VEL_STEP_SIZE)
        self.send_vel()

    def start(self):
        self.running = True
        self.sid = self.ws.subscribe(CONTROL_RESOURCE, self.listener)
        self.send_vel()
        while self.running:
            time.sleep(1)


    def stop(self):
        self.ws.unsubscribe(self.sid)
        self.running = False

    def send_vel(self):
        twist = Twist()

        self.control_linear_velocity = make_simple_profile(
                    self.control_linear_velocity,
                    self.target_linear_velocity,
                    (LIN_VEL_STEP_SIZE/2.0))

        twist.linear.x = self.control_linear_velocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        self.control_angular_velocity = make_simple_profile(
                self.control_angular_velocity,
                self.target_angular_velocity,
                (ANG_VEL_STEP_SIZE/2.0))

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.control_angular_velocity

        self.pub.publish(twist)
        d = {
        'target_linear_velocity': self.target_linear_velocity,
        'target_angular_velocity': self.target_angular_velocity,
        'control_linear_velocity': self.control_linear_velocity,
        'control_angular_velocity': self.control_angular_velocity
        }
        js_state = json.dumps(d)
        self.ws.put(STATE_RESOURCE, Value(js_state, Encoding.STRING))



def make_simple_profile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)


def main():
    y = sys.argv[1]
    c = Controller(y)

    def sh(sig, frame):
        c.stop()

    signal.signal(signal.SIGINT, sh)

    c.start()






if __name__ == '__main__':
    main()
