# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Darby Lim

import os
import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import QoSProfile

import json
from flask import Flask



BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = "burger" #os.environ['TURTLEBOT3_MODEL']


app = Flask(__name__)



@app.route('/')
def index():
    return create_response()

@app.route('/fwd')
def fwd():
    # increases the linear velocity by LIN_VEL_STEP_SIZE
    global target_linear_velocity
    target_linear_velocity = check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
    send_vel()
    return create_response()


@app.route('/bwd')
def bwd():
    # reduces the linear velocity by LIN_VEL_STEP_SIZE
    global target_linear_velocity
    target_linear_velocity = check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
    send_vel()
    return create_response()

@app.route('/stop')
def stop():
    # sets all velocity to 0
    global target_linear_velocity
    target_linear_velocity   = 0.0
    global target_angular_velocity
    target_angular_velocity  = 0.0
    global control_linear_velocity
    control_linear_velocity  = 0.0
    global control_angular_velocity
    control_angular_velocity = 0.0
    send_vel()
    return create_response()

@app.route('/sx')
def sx():
    # increases  the angular velocity by ANG_VEL_STEP_SIZE
    global target_angular_velocity
    target_angular_velocity = check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
    send_vel()
    return create_response()

@app.route('/dx')
def dx():
    # reduces the angular velocity by ANG_VEL_STEP_SIZE
    global target_angular_velocity
    target_angular_velocity = check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
    send_vel()
    return create_response()



def create_response():
    d = {
        'target_linear_velocity': target_linear_velocity,
        'target_angular_velocity': target_angular_velocity,
        'control_linear_velocity': control_linear_velocity,
        'control_angular_velocity': control_angular_velocity
    }
    return json.dumps(d)

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


def send_vel():
    twist = Twist()

    global control_linear_velocity
    control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE/2.0))

    twist.linear.x = control_linear_velocity
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    global control_angular_velocity
    control_angular_velocity = make_simple_profile(
            control_angular_velocity,
            target_angular_velocity,
            (ANG_VEL_STEP_SIZE/2.0))

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = control_angular_velocity

    pub.publish(twist)



def main():

    qos = QoSProfile(depth=10)
    global pub
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    global target_linear_velocity
    target_linear_velocity   = 0.0
    global target_angular_velocity
    target_angular_velocity  = 0.0
    global control_linear_velocity
    control_linear_velocity  = 0.0
    global control_angular_velocity
    control_angular_velocity = 0.0

    app.run(debug=True)


if __name__ == '__main__':
    main()
