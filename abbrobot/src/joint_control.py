import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# This function is joint_state publisher.
def talker():
	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rospy.init_node('joint_state_publisher')
	
	rate = rospy.Rate(2) # 10hz
	robot_hand_JointState = JointState()
	robot_hand_JointState.header = Header()
	robot_hand_JointState.header.stamp = rospy.Time.now()
	robot_hand_JointState.name = ['abb_irb1200_5_90joint_1', 'abb_irb1200_5_90joint_2', 'abb_irb1200_5_90joint_3',
	                              'abb_irb1200_5_90joint_4', 'abb_irb1200_5_90joint_5', 'abb_irb1200_5_90joint_6']

	joint1_angle = 0.0
	joint2_angle = 0.0
	joint3_angle = 0.0
	joint4_angle = 0.0
	joint5_angle = 0.0
	joint6_angle = 0.0

	while not rospy.is_shutdown():
		robot_hand_JointState.header.stamp = rospy.Time.now()
		
		joint1_angle = joint1_angle + 0.1
		joint2_angle = joint2_angle + 0.1
		joint3_angle = joint3_angle + 0.1
		joint4_angle = joint4_angle + 0.1
		joint5_angle = joint5_angle + 0.1
		joint6_angle = joint6_angle + 0.1
		

		robot_hand_JointState.position = [joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle, joint6_angle] 
		                                  
		pub.publish(robot_hand_JointState)

		rate.sleep()


if __name__ == '__main__':
	talker()