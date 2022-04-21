#!/usr/bin/env python3

from __future__ import print_function

import threading

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
# from mavros_msgs.msg import State
from std_msgs.msg import Bool

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   -/-     w(+y)/    -/-
    a(-x)  s(-y)     d(+x)
Rotate: g(+th)
Up | Down = i(+z) | m(-z)
Stop :p (anything)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

#x, y, z, th
#to make x-y direction movement, just do something like 
# x   y   z  th
# (1, -1, 0, 0)
#or even 3d movement (-1, 1, 1, 0)
moveBindings = {
        'd':(0, -1, 0, 0),
        'a':(0, 1, 0 ,0),
        's':(-1, 0, 0, 0),
        'w':(1, 0, 0, 0),
        'i':(0, 0, 1, 0),
        'm':(0, 0, -1, 0),
        'g':(0, 0, 0, 0.1),
        'D':(0, -1, 0, 0),
        'A':(0, 1, 0 ,0),
        'S':(-1, 0, 0, 0),
        'W':(1, 0, 0, 0),
        'I':(0, 0, 1, 0),
        'M':(0, 0, -1, 0),
        # 'G':(0, 0, 0, 0.1),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

AUTO_LAND_CMD = '6'

# curState = State.MODE_PX4_READY
# def callback(data):
#     global curState
#     curState = data

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.publish_auto_land = rospy.Publisher('auto_land_cmd', Bool, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.do_auto_land = False
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()
    
    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn, do_auto_land):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        self.do_auto_land = do_auto_land
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0, True)
        self.join()

    def run(self):
        twist = Twist()
        # trigger_land = Bool()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn
            # trigger_land.data = do_auto_land
            self.condition.release()

            # Publish.
            self.publisher.publish(twist)
            self.publish_auto_land.publish(do_auto_land)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)
        self.publish_auto_land.publish(False)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    # print("THE SPEED: ")
    user_speed = float(input("input teleop initial-max-speed: "))
    speed = user_speed
    print(user_speed)
    print("The speed {}".format(speed))
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    # rospy.Subscriber("mavros/state", State, callback)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    do_auto_land = False
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn, do_auto_land)

        print(msg)
        print("PRESS {} to request auto land".format(AUTO_LAND_CMD))
        print(vels(speed,turn))
        # print(curState)

        while(1):
            
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                do_auto_land=False
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                do_auto_land=False
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                    print("PRESS {} to request auto land".format(AUTO_LAND_CMD))
                status = (status + 1) % 15
            elif key == AUTO_LAND_CMD:
                do_auto_land=True
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue            

                x = 0
                y = 0
                z = 0
                th = 0
                do_auto_land=False
                if (key == '\x03'):
                    break
 
            pub_thread.update(x, y, z, th, speed, turn, do_auto_land)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
