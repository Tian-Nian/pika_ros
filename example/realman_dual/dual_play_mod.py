import sys
sys.path.append("./")

import rospy
from geometry_msgs.msg import PoseStamped
from RealMan_mod import RM_controller
from Realsense import Img_controller
from tf.transformations import euler_from_quaternion
# from sensor_tools import Gripper
from data_dual import CollectData
from Robotic_Arm.rm_robot_interface import *
import argparse
import os

# right_wrist_controller = RM_controller("192.168.80.18", rm_thread_mode_e.RM_TRIPLE_MODE_E)
right_wrist_controller = RM_controller("192.168.80.18")
state = right_wrist_controller.arm_controller.rm_get_current_arm_state()
print(f"----right_state:{state}")
# left_wrist_controller = RM_controller("192.168.80.19", rm_thread_mode_e.RM_TRIPLE_MODE_E)
left_wrist_controller = RM_controller("192.168.80.19")
state = left_wrist_controller.arm_controller.rm_get_current_arm_state()
print(f"----left_state:{state}")
# Camera serial numbers configuration

CAMERA_SERIALS = {
    'head': '427622270438',  # Replace with actual serial number
    'left_wrist': '427622272401',   # Replace with actual serial number
    'right_wrist': '427622270277',   # Replace with actual serial number
}

imgs_controller = Img_controller(CAMERA_SERIALS)
collect_index = 0
output_path = None

def collect_once():
    global right_wrist_controller, left_wrist_controller, imgs_controller, collect_index, output_path
    # Get robot arm state
    success, right_arm_state = right_wrist_controller.arm_controller.rm_get_current_arm_state()
    if not success:
        raise ConnectionError(f"Failed to get right arm state")
    success, left_arm_state = left_wrist_controller.arm_controller.rm_get_current_arm_state()
    if not success:
        raise ConnectionError(f"Failed to get left arm state")
    # Get gripper state
    success, right_gripper_state = right_wrist_controller.arm_controller.rm_get_gripper_state()
    if not success:
        raise ConnectionError(f"Failed to get right gripper")
    success, left_gripper_state = left_wrist_controller.arm_controller.rm_get_gripper_state()
    if not success:
        raise ConnectionError(f"Failed to get left gripper")
    # get images
    imgs = imgs_controller.get_img()
    data = CollectData(right_arm_state, right_gripper_state, left_arm_state, left_gripper_state, imgs)
    data.write(output_path, collect_index)
    collect_index += 1

def pose_callback_right(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    quaternion = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    )
    rospy.loginfo(f"Received pose: x={x}, y={y}, z={z}")
    #qx, qy, qz, qw = quaternion
    # 使用欧拉角表示姿态
    rx,ry,rz = euler_from_quaternion(quaternion)
    # move robot arm to the received pose
    global right_wrist_controller
    right_wrist_controller.move([x,y,z,rx,ry,rz])
    # set right arm as main arm
    collect_once()


def pose_callback_left(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    quaternion = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    )
    rospy.loginfo(f"Received pose: x={x}, y={y}, z={z}")
    #欧拉角
    rx,ry,rz = euler_from_quaternion(quaternion)
    # move robot arm to the received pose
    global right_wrist_controller
    right_wrist_controller.move([x,y,z,rx,ry,rz])

def gripper_callback_right(msg):
    global right_wrist_controller
    # get gripper data
    gripper = msg.joint_states[0]
    right_wrist_controller.set_gripper(gripper)

def gripper_callback_left(msg):
    global left_wrist_controller
    # get gripper data
    gripper = msg.joint_states[0]
    left_wrist_controller.set_gripper(gripper)

def setup():
    print("Begin initial pos_subscriber!!!")
    #初始化节点
    rospy.init_node('pose_subscriber')

    # rospy.Subscriber('/right_pika_pose', PoseStamped, pose_callback_right)
    # rospy.Subscriber('/left_pika_pose', PoseStamped, pose_callback_left)

    #保持与<node>定义一致
    rospy.Subscriber('/pika_pose_r', PoseStamped, pose_callback_right)
    rospy.Subscriber('/pika_pose_l', PoseStamped, pose_callback_left)

    #Gripper的定义？Griper.py?
    # rospy.Subscriber('/gripper_r/joint_states', Gripper, gripper_callback_right)
    # rospy.Subscriber('/gripper_l/joint_states', Gripper, gripper_callback_left)
    
    rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process some episodes.')
    parser.add_argument('--arm_brand', type=str, default="RealMan",
                        help='arm brand like realman,agilex ...')
    parser.add_argument('--task_name', type=str, 
                        default="pickbottle",help='task_name like shoe_place,cup_pick_up ...')
    parser.add_argument('--episode_index', type=int, default = 0,
                        help='episode_index like 0,1 ...')
    args = parser.parse_args()
    arm_brand = args.arm_brand
    task_name = args.task_name
    episode_index = args.episode_index
    output_path = f"./datasets/npy/{arm_brand}/{task_name}/{episode_index}"
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    setup()
    # while True:
        
        