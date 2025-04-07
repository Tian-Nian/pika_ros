import rospy
from geometry_msgs.msg import PoseStamped

def pose_callback_right(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    rospy.loginfo(f"Received right pose: x={x}, y={y}, z={z}")

def pose_callback_left(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    rospy.loginfo(f"Received left pose: x={x}, y={y}, z={z}")

def setup():
    rospy.init_node('pose_subscriber_test', anonymous=True)
    
    rospy.Subscriber('/pika_pose_r', PoseStamped, pose_callback_right)
    rospy.Subscriber('/pika_pose_l', PoseStamped, pose_callback_left)
    
    rospy.loginfo("开始订阅 ROS 节点...")
    rospy.spin()

if __name__ == '__main__':
    setup()
