#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sys

def send_command(command):
    pub = rospy.Publisher('/robot_commands', String, queue_size=10)
    rospy.init_node('robot_commander', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    
    # Wait for connections to establish
    rospy.sleep(1)
    
    # Send the command
    pub.publish(command)
    rospy.loginfo(f"Sent command: {command}")
    
    # Give time for the command to be processed
    rospy.sleep(1)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: ./control_robot.py <command>")
        print("Commands:")
        print("  red - Move to and pick up the red object")
        print("  blue - Move to and pick up the blue object")
        print("  move to X Y - Move held object to position X Y")
        print("  home - Return arm to home position")
        sys.exit(1)
        
    command = ' '.join(sys.argv[1:])
    
    try:
        send_command(command)
    except rospy.ROSInterruptException:
        pass
