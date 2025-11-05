#!/usr/bin/env python3

""" 
Node to teleoperate both a differential and an omnidirectional type mobile robot,
sending control velocities (V,W or Vx,W,Vy), with maximum velocities.
To finish this node, please press 'ctrl + C'.
"""

__author__ = "C. Mauricio Arteaga-Escamilla from 'Robotica Posgrado' (Youtube channel)"


import os, select, sys, rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

MAX_LIN_VEL = 0.5
MAX_ANG_VEL = 1.2

LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your Mobile Robot!
---------------------------
  Moving around:    Lateral motion (Omni)
        w                    
   a    s    d          g       j
        x

w/x : increase/decrease linear velocity, Vx  (Max vel: 0.5)
a/d : increase/decrease angular velocity, W  (Max vel: 1.2)
g/j : increase/decrease lateral velocity, Vy (Max vel: 0.5)
space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity_x, target_angular_velocity, target_linear_velocity_y):
    print("currently:\tVx: %s\t W: %s\t Vy: %s" % (target_linear_velocity_x,target_angular_velocity,target_linear_velocity_y))

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
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
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('omni_teleop_keyboard')
    cmd_vel_topic = 'cmd_vel'
    pub = node.create_publisher(Twist, cmd_vel_topic, qos)

    status = 0
    target_linear_velocity_x = 0.0
    target_angular_velocity = 0.0
    target_linear_velocity_y = 0.0
    control_linear_velocity_x = 0.0
    control_angular_velocity = 0.0
    control_linear_velocity_y = 0.0

    node.get_logger().info('Created node')
    node.get_logger().warn('By default, publishing on topic: '+ cmd_vel_topic)
    node.get_logger().info('To use a namespace, to remap topics, services and node name, please use:')
    node.get_logger().warn('--ros-args -r __ns:=/new_ns')

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'w':
                target_linear_velocity_x =\
                    check_linear_limit_velocity(target_linear_velocity_x + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity_x, target_angular_velocity, target_linear_velocity_y)
            elif key == 'x':
                target_linear_velocity_x =\
                    check_linear_limit_velocity(target_linear_velocity_x - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity_x, target_angular_velocity, target_linear_velocity_y)

            elif key == 'a':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity_x, target_angular_velocity, target_linear_velocity_y)
            elif key == 'd':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity_x, target_angular_velocity, target_linear_velocity_y)
                
            elif key == 'g':
                target_linear_velocity_y =\
                    check_linear_limit_velocity(target_linear_velocity_y + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity_x, target_angular_velocity, target_linear_velocity_y)
            elif key == 'j':
                target_linear_velocity_y =\
                    check_linear_limit_velocity(target_linear_velocity_y - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity_x, target_angular_velocity, target_linear_velocity_y)

            elif key == ' ' or key == 's':
                target_linear_velocity_x = 0.0
                control_linear_velocity_x = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                target_linear_velocity_y = 0.0
                control_linear_velocity_y = 0.0
                print_vels(target_linear_velocity_x, target_angular_velocity, target_linear_velocity_y)
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()

            control_linear_velocity_x = make_simple_profile(
                control_linear_velocity_x,
                target_linear_velocity_x,
                (LIN_VEL_STEP_SIZE / 2.0))

            control_linear_velocity_y = make_simple_profile(
                control_linear_velocity_y,
                target_linear_velocity_y,
                (LIN_VEL_STEP_SIZE / 2.0))

            twist.linear.x = target_linear_velocity_x
            twist.linear.y = target_linear_velocity_y
            #twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            #twist.angular.x = 0.0; twist.angular.y = 0.0
            twist.angular.z = target_angular_velocity
            
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
