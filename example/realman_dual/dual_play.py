import sys
sys.path.append("./")

import rospy
from geometry_msgs.msg import PoseStamped
from RealMan import RM_controller
from Realsense import Img_controller
from tf.transformations import euler_from_quaternion
#from sensor_tools import Gripper
from data_dual import CollectData
from Robotic_Arm.rm_robot_interface import *
import argparse
import threading
import os
DEBUG = False

def debug_print(msg, release=False):
    if release or DEBUG:
        print(f"[DEBUG] {msg}")

msg_freq= 50

left_wrist_controller = RM_controller("192.168.80.19",rm_thread_mode_e.RM_TRIPLE_MODE_E)
succ, state = left_wrist_controller.arm_controller.rm_get_current_arm_state()
debug_print(f"  succ {succ}   ----left_state:{state}", True)
right_wrist_controller = RM_controller("192.168.80.18")
succ, state = right_wrist_controller.arm_controller.rm_get_current_arm_state()
debug_print(f"  succ {succ}   ----right_state:{state}", True)
state = right_wrist_controller.get_state()
debug_print(f"----right_state:{state}",True)
state = left_wrist_controller.get_state()
debug_print(f"Second  succ   ----left_state:{state}",True)

succ, state = left_wrist_controller.arm_controller.rm_get_current_arm_state()
debug_print(f"  succ   ----left_state:{state}")
#exit()
# Camera serial numbers configuration

# Camera serial numbers configuration
CAMERA_SERIALS = {
    'head': '427622270438',  # Replace with actual serial number
    'left_wrist': '427622272401',   # Replace with actual serial number
    'right_wrist': '427622270277',   # Replace with actual serial number
}

# imgs_controller = Img_controller(CAMERA_SERIALS)
debug_print("摄像头启动完成")
collect_index = 0
output_path = None
index_right = 0
index_left = 0


def collect_once():
    global right_wrist_controller, left_wrist_controller, collect_index, output_path
    # global right_wrist_controller, left_wrist_controller, imgs_controller, collect_index, output_path
    # Get robot arm state
    right_arm_state = right_wrist_controller.arm_controller.rm_get_current_arm_state()[1]

    left_arm_state = left_wrist_controller.arm_controller.rm_get_current_arm_state()[1]

    #没有取gripper信息，0替代
    # Get gripper state
    # right_gripper_state = right_wrist_controller.arm_controller.rm_get_gripper_state()[1]
    # debug_print(f"THE right gripper state : {right_gripper_state}, type:::: {type(right_gripper_state)}",True)
    # left_gripper_state = left_wrist_controller.arm_controller.rm_get_gripper_state()[1]
    # debug_print(f"THE left gripper state : {left_gripper_state},, type:::: {type(left_gripper_state)}",True)
    right_gripper_state = 0.0
    left_gripper_state = 0.0
    # get images
    # imgs = imgs_controller.get_img()
    imgs = []
    data = CollectData(right_arm_state, right_gripper_state, left_arm_state, left_gripper_state, imgs)
    data.write(output_path, collect_index)
    collect_index += 1

def pose_callback_right(msg):
    # print("call right")
    global right_wrist_controller
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    quaternion = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    )
    # rospy.loginfo(f"right_arm_stateReceived pose: x={x}, y={y}, z={z}")
    global index_right
    if index_right==msg_freq:
        # state = right_wrist_controller.arm_controller.rm_get_current_arm_state()
        
        succ, state = right_wrist_controller.arm_controller.rm_get_current_arm_state()
        # debug_print("succ outer right:",succ)
        # debug_print(f"----right_wrist_controller_state:{state}") 
        index_right=0
        # debug_print(index_right)
        # rospy.loginfo(f"Received pose: x={x}, y={y}, z={z}")
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        # move robot arm to the received pose
        # right_wrist_controller.move_test()
        succ, state = right_wrist_controller.arm_controller.rm_get_current_arm_state()
        # debug_print(f"right::I am going to moving, the state:{state} ------success: {succ}",True)
        right_wrist_controller.move([x,y,z,roll, pitch, yaw])
        # set right arm as main arm
        
    index_right+=1


def pose_callback_left(msg):
    global left_wrist_controller
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    quaternion = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    )
    # rospy.loginfo(f"left_arm_stateReceived pose: x={x}, y={y}, z={z}")
    global index_left
    if index_left==msg_freq:
        # state = left_wrist_controller.arm_controller.rm_get_current_arm_state()
        
        succ, state = left_wrist_controller.arm_controller.rm_get_current_arm_state()
        # debug_print("succ outer left:",succ)
        # debug_print(f"----left_wrist_controller_state:{state}") 
        index_left=0
        # debug_print(index_left)
        # rospy.loginfo(f"Received pose: x={x}, y={y}, z={z}")
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        # move robot arm to the received pose
        # left_wrist_controller.move_test()
        succ, state = left_wrist_controller.arm_controller.rm_get_current_arm_state()
        # debug_print(f"Left::I am going to moving, the state:{state} ------success: {succ}",True)
        left_wrist_controller.move([x,y,z,roll, pitch, yaw])
        # set right arm as main arm
        
    index_left+=1
    

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
    debug_print("开始订阅ros节点 ", True)

    try:
        rospy.init_node('pose_subscriber', anonymous=True)
        debug_print("ROS节点初始化成功", True)
    except Exception as e:
        debug_print(f"初始化失败: {e}")
        return  # 退出 setup 函数
    debug_print("Beging Subscriber!!!!")
    

    rospy.Subscriber('/pika_pose_l', PoseStamped, pose_callback_left)
    rospy.Subscriber('/pika_pose_r', PoseStamped, pose_callback_right)
    
    #rospy.Subscriber('/gripper_r/joint_states', Gripper, gripper_callback_right)
    #rospy.Subscriber('/gripper_l/joint_states', Gripper, gripper_callback_left)
    
    rospy.spin()
    



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process some episodes.')
    parser.add_argument('--arm_brand', type=str, default="RealMan",
                        help='arm brand like realman,agilex ...')
    parser.add_argument('--task_name', type=str, default="example_task",
                        help='task_name like shoe_place,cup_pick_up ...')
    parser.add_argument('--episode_index', type=int, default=0,
                        help='episode_index like 0,1 ...')
    args = parser.parse_args()

    arm_brand = args.arm_brand
    task_name = args.task_name
    episode_index = args.episode_index
    output_path = f"./datasets/npy/{arm_brand}/{task_name}/{episode_index}"
    if not os.path.exists(output_path):
        os.makedirs(output_path)
   
    succ, state = right_wrist_controller.arm_controller.rm_get_current_arm_state()
    debug_print(f"Main:::  succ {succ}   ----right_state:{state}")
    setup()
    # while True:
        
        
