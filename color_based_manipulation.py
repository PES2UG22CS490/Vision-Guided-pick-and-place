#!/usr/bin/env python3

import rospy
import random
import tf2_ros
import geometry_msgs.msg
import math
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String

class RobotArmSimulation:
    def __init__(self):
        rospy.init_node('robot_arm_simulation', anonymous=True)
        self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=1)
        self.command_sub = rospy.Subscriber('/robot_commands', String, self.command_callback)
        self.rate = rospy.Rate(10)  # 10hz
        
        # Create robot arm and objects
        self.markers = MarkerArray()
        self.create_robot_arm()
        self.create_objects()
        
        # Object positions
        self.object_positions = {
            "red": [0.5, 0.3, 0.0],
            "blue": [0.5, -0.3, 0.0]
        }
        
        # Current arm position
        self.arm_tip_position = [0.0, 0.0, 0.3]
        self.holding_object = None
        
        rospy.loginfo("Robot arm simulation initialized. Send commands to /robot_commands topic.")
        
    def create_robot_arm(self):
        # Base of the robot
        base = Marker()
        base.header.frame_id = "base_link"
        base.ns = "robot_arm"
        base.id = 0
        base.type = Marker.CYLINDER
        base.action = Marker.ADD
        base.pose.position.x = 0
        base.pose.position.y = 0
        base.pose.position.z = 0.05
        base.pose.orientation.w = 1.0
        base.scale.x = 0.2
        base.scale.y = 0.2
        base.scale.z = 0.1
        base.color.r = 0.2
        base.color.g = 0.2
        base.color.b = 0.2
        base.color.a = 1.0
        self.markers.markers.append(base)
        
        # First arm segment
        arm1 = Marker()
        arm1.header.frame_id = "base_link"
        arm1.ns = "robot_arm"
        arm1.id = 1
        arm1.type = Marker.CUBE
        arm1.action = Marker.ADD
        arm1.pose.position.x = 0
        arm1.pose.position.y = 0
        arm1.pose.position.z = 0.2
        arm1.pose.orientation.w = 1.0
        arm1.scale.x = 0.1
        arm1.scale.y = 0.1
        arm1.scale.z = 0.3
        arm1.color.r = 0.5
        arm1.color.g = 0.5
        arm1.color.b = 0.5
        arm1.color.a = 1.0
        self.markers.markers.append(arm1)
        
        # Second arm segment (will be animated)
        arm2 = Marker()
        arm2.header.frame_id = "base_link"
        arm2.ns = "robot_arm"
        arm2.id = 2
        arm2.type = Marker.CUBE
        arm2.action = Marker.ADD
        arm2.pose.position.x = 0
        arm2.pose.position.y = 0
        arm2.pose.position.z = 0.35
        arm2.pose.orientation.w = 1.0
        arm2.scale.x = 0.05
        arm2.scale.y = 0.05
        arm2.scale.z = 0.3
        arm2.color.r = 0.7
        arm2.color.g = 0.7
        arm2.color.b = 0.7
        arm2.color.a = 1.0
        self.markers.markers.append(arm2)
        
        # End effector (gripper)
        gripper = Marker()
        gripper.header.frame_id = "base_link"
        gripper.ns = "robot_arm"
        gripper.id = 3
        gripper.type = Marker.SPHERE
        gripper.action = Marker.ADD
        gripper.pose.position.x = 0
        gripper.pose.position.y = 0
        gripper.pose.position.z = 0.5
        gripper.pose.orientation.w = 1.0
        gripper.scale.x = 0.08
        gripper.scale.y = 0.08
        gripper.scale.z = 0.08
        gripper.color.r = 0.3
        gripper.color.g = 0.3
        gripper.color.b = 0.8
        gripper.color.a = 1.0
        self.markers.markers.append(gripper)
    
    def create_objects(self):
        # Red object
        red_obj = Marker()
        red_obj.header.frame_id = "base_link"
        red_obj.ns = "objects"
        red_obj.id = 4
        red_obj.type = Marker.CUBE
        red_obj.action = Marker.ADD
        red_obj.pose.position.x = 0.5
        red_obj.pose.position.y = 0.3
        red_obj.pose.position.z = 0.05
        red_obj.pose.orientation.w = 1.0
        red_obj.scale.x = 0.1
        red_obj.scale.y = 0.1
        red_obj.scale.z = 0.1
        red_obj.color.r = 1.0
        red_obj.color.g = 0.0
        red_obj.color.b = 0.0
        red_obj.color.a = 1.0
        self.markers.markers.append(red_obj)
        
        # Blue object
        blue_obj = Marker()
        blue_obj.header.frame_id = "base_link"
        blue_obj.ns = "objects"
        blue_obj.id = 5
        blue_obj.type = Marker.CUBE
        blue_obj.action = Marker.ADD
        blue_obj.pose.position.x = 0.5
        blue_obj.pose.position.y = -0.3
        blue_obj.pose.position.z = 0.05
        blue_obj.pose.orientation.w = 1.0
        blue_obj.scale.x = 0.1
        blue_obj.scale.y = 0.1
        blue_obj.scale.z = 0.1
        blue_obj.color.r = 0.0
        blue_obj.color.g = 0.0
        blue_obj.color.b = 1.0
        blue_obj.color.a = 1.0
        self.markers.markers.append(blue_obj)

    def update_arm_position(self, target_position):
        """Update the robot arm position gradually toward the target"""
        steps = 20
        current_position = self.arm_tip_position.copy()
        dx = (target_position[0] - current_position[0]) / steps
        dy = (target_position[1] - current_position[1]) / steps
        dz = (target_position[2] - current_position[2]) / steps
        
        for i in range(steps):
            current_position[0] += dx
            current_position[1] += dy
            current_position[2] += dz
            self.arm_tip_position = current_position.copy()
            
            # Update arm segments
            # Calculate arm joint angles (simplified inverse kinematics)
            x, y, z = current_position
            base_angle = math.atan2(y, x)
            
            # Update second arm segment and gripper position
            self.markers.markers[2].pose.position.x = x * 0.5
            self.markers.markers[2].pose.position.y = y * 0.5
            self.markers.markers[2].pose.position.z = 0.35
            
            # Rotate the arm
            q = geometry_msgs.msg.Quaternion()
            q.x = 0
            q.y = math.sin(base_angle/2)
            q.z = 0
            q.w = math.cos(base_angle/2)
            self.markers.markers[2].pose.orientation = q
            
            # Update gripper position
            self.markers.markers[3].pose.position.x = x
            self.markers.markers[3].pose.position.y = y
            self.markers.markers[3].pose.position.z = z
            
            # If holding an object, update its position
            if self.holding_object:
                color = self.holding_object
                obj_id = 4 if color == "red" else 5
                self.markers.markers[obj_id].pose.position.x = x
                self.markers.markers[obj_id].pose.position.y = y
                self.markers.markers[obj_id].pose.position.z = z - 0.1
            
            # Publish updated markers
            self.marker_pub.publish(self.markers)
            self.rate.sleep()
    
    def command_callback(self, msg):
        """Handle commands sent to the robot"""
        command = msg.data.strip().lower()
        
        if command in ["red", "blue"]:
            color = command
            rospy.loginfo(f"Moving to pick up {color} object")
            
            # Move to the object
            target_position = self.object_positions[color].copy()
            target_position[2] = 0.2  # Hover above the object
            self.update_arm_position(target_position)
            
            # Move down to grasp
            target_position[2] = 0.1
            self.update_arm_position(target_position)
            
            # Set as holding object
            self.holding_object = color
            rospy.loginfo(f"Picked up {color} object")
            
            # Move up with the object
            target_position[2] = 0.3
            self.update_arm_position(target_position)
            
        elif command.startswith("move to"):
            # Parse the target position
            parts = command.split()
            if len(parts) == 5 and parts[1] == "to" and parts[2].replace('.','',1).isdigit() and parts[3].replace('.','',1).isdigit():
                x = float(parts[2])
                y = float(parts[3])
                rospy.loginfo(f"Moving to position ({x}, {y})")
                
                if self.holding_object:
                    # Update object position in our tracking dict
                    color = self.holding_object
                    self.object_positions[color] = [x, y, 0.0]
                    
                    # Move to the target
                    target_position = [x, y, 0.3]
                    self.update_arm_position(target_position)
                    
                    # Lower the object
                    target_position[2] = 0.1
                    self.update_arm_position(target_position)
                    
                    # Release the object
                    obj_id = 4 if color == "red" else 5
                    self.markers.markers[obj_id].pose.position.x = x
                    self.markers.markers[obj_id].pose.position.y = y
                    self.markers.markers[obj_id].pose.position.z = 0.05
                    
                    self.holding_object = None
                    rospy.loginfo(f"Released {color} object at ({x}, {y})")
                    
                    # Move up
                    target_position[2] = 0.3
                    self.update_arm_position(target_position)
                else:
                    rospy.loginfo("No object is currently held")
            else:
                rospy.loginfo("Invalid command format. Use 'move to X Y'")
        
        elif command == "home":
            rospy.loginfo("Moving arm to home position")
            self.update_arm_position([0.0, 0.0, 0.3])
            
        else:
            rospy.loginfo("Unknown command. Valid commands: 'red', 'blue', 'move to X Y', 'home'")
    
    def run(self):
        """Main loop to keep publishing visualization markers"""
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.markers)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        arm_sim = RobotArmSimulation()
        arm_sim.run()
    except rospy.ROSInterruptException:
        pass
