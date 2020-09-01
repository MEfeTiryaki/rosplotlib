
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from math import*
rospy.init_node('send', anonymous=True)
pub_x= rospy.Publisher("/x", Float64MultiArray, queue_size=10)
pub_y= rospy.Publisher("/y", Float64MultiArray, queue_size=10)
pub_z= rospy.Publisher("/some_thing", Float64MultiArray, queue_size=10)
rate = rospy.Rate(100) # 10hz\

msg =Float64MultiArray()

msg.data = list(range(0,100))

msg_y = Float64MultiArray()
msg_z = Float64MultiArray()
while not rospy.is_shutdown():
    pub_x.publish(msg)
    msg_y.data = list(map(lambda x : cos(0.1*x+rospy.get_time()%10),msg.data ))
    msg_z.data = list(map(lambda x : sin(0.5*x+rospy.get_time()%10),msg.data ))
    pub_y.publish(msg_y)
    pub_z.publish(msg_z)
    rate.sleep()
