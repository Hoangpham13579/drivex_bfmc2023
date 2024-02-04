#!/usr/bin/env python3

# Imports
from functools import partial
import json

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


# Direction Callback Function
def processKeyboardTopic(message, **kwargs):
    """Convert /keyboard_cmd to Twist message & publish to /cmd_vel topic"""
    
    # Process message
    data = json.loads(message.data)
    if data.get('action') == '4':
        speed, angular = 0.0, 0.0
    else:
        angular = data.get('steerAngle')
        speed = data.get('speed')

    # Publish messages
    twist_cmd = Twist()  # Message type twist
    twist_cmd.linear.x = speed
    twist_cmd.angular.z = float(angular)
    kwargs['cmd_vel_publisher'].publish(twist_cmd)


def main():
    # Defining variables
    kwargs = dict(twist_publisher=None)

    # Initiating node
    rospy.init_node("keyboard_teleop", anonymous=False)

    # Publish to /cmd_vel topic
    cmd_vel_topic = rospy.get_param("~twist_cmd_vel", "/cmd_vel")  # Publish
    kwargs['cmd_vel_publisher'] = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    # Define partials
    processKeyboardTopic_callback = partial(processKeyboardTopic, **kwargs)
    
    keyboard_topic = rospy.get_param("~keyboard_topic", "/keyboard_cmd")  # Subscribe
    rospy.Subscriber(keyboard_topic, String, processKeyboardTopic_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
