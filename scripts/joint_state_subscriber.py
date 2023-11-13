#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import csv

csv_file = open('joint_position_wo_saw.csv', 'wb')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['timestamp', 'position'])

def joint_state_callback(msg):
    """
    This callback function prints the effort of each joint to the console in a readable format.
    """
    joint_efforts = msg.effort
    joint_names = msg.name

    timestamp = rospy.get_time()
    
    position = msg.position[4]

    csv_writer.writerow([timestamp, position])

    print_message = 'Joint States Efforts:\n'
    for name, effort in zip(joint_names, joint_efforts):
        print_message += '  {}: {:.2f}\n'.format(name, effort)
    
    rospy.loginfo(print_message)

def listener():
    """
    Initializes the node, subscriber, and spins to keep the node running.
    """
    rospy.init_node('joint_state_printer', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
