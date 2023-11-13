#!/usr/bin/env python

import rospy
import tf
import csv
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

# Initialize ROS node
rospy.init_node('end_effector_logger')

# Initialize tf listener
tf_listener = tf.TransformListener()

# Global variables to store the latest joint efforts
joint_efforts = None

# Callback function for JointState messages
def joint_state_callback(msg):
    global joint_efforts
    joint_efforts = msg.effort

# Subscribe to the JointState topic
rospy.Subscriber('/joint_states', JointState, joint_state_callback)

# Open a CSV file to write the end effector state and joint efforts
csv_file_path = '/home/csc0208/catkin_ws/src/human_robotics/data/end_effector_pose_O_effort_test.csv'
csv_file = open(csv_file_path, 'w')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['time', 'position_x', 'position_y', 'position_z', 
                     'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                     'effort1', 'effort2', 'effort3', 'effort4', 'effort5'])

# Callback function for timer event
def log_end_effector_state_and_effort(event):
    try:
        if not tf_listener.canTransform('/world', '/top_wrist', rospy.Time(0)):
            return

        trans, rot = tf_listener.lookupTransform('/world', '/top_wrist', rospy.Time(0))
        now = rospy.Time.now().to_sec()

        # Combine end-effector state with joint efforts
        log_data = [now] + list(trans) + list(rot)
        if joint_efforts is not None:
            log_data += joint_efforts

        # Write to the CSV file
        csv_writer.writerow(log_data)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn('TF Error: {}'.format(e))

# Start the timer to log at 20Hz
log_timer = rospy.Timer(rospy.Duration(1.0/50.0), log_end_effector_state_and_effort)

# Keep the node running until it's stopped
rospy.spin()

# Close the CSV file when the node is shut down
csv_file.close()
