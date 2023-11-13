#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

# Initialize ROS node
rospy.init_node('joint_trajectory_publisher')

# Initialize publisher
pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

# Wait for the publisher to establish connection
rospy.sleep(1)

# Function to publish a trajectory read from a file
def publish_trajectory_from_file(file_path):
    # Open the file
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
        # For each line in the file, publish a JointTrajectory message
        for index, line in enumerate(lines):
            # Parse the joint angles from the line
            joint_angles = [float(angle) for angle in line.strip().split()]
            
            # Create the JointTrajectory message
            trajectory = JointTrajectory()
            trajectory.joint_names = ["arm_base_joint", "shoulder_joint", "bottom_wrist_joint", "elbow_joint", "top_wrist_joint"]
            
            # Create a trajectory point and assign positions and time from start
            point = JointTrajectoryPoint()
            point.positions = joint_angles
            point.time_from_start = rospy.Duration(index)  # Assume 1 second per timestep
            
            # Add the trajectory point to the JointTrajectory message
            trajectory.points.append(point)
            
            # Publish the message
            pub.publish(trajectory)
            rospy.loginfo("Published joint trajectory for timestep: {}".format(index))
            
            # Wait for the next timestep
            rospy.sleep(1)  # Adjust as needed for your specific timestep

if __name__ == '__main__':
    try:
        # Specify the path to your file here
        file_path = '/home/csc0208/catkin_ws/src/human_robotics/data/data.txt'
        publish_trajectory_from_file(file_path)
    except rospy.ROSInterruptException:
        pass
    except IOError as e:
        rospy.logerr('File not found: {}'.format(e))
