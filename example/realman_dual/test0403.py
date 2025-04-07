from Robotic_Arm.rm_robot_interface import *
import time

class test_rm:
    def __init__(self):
        self.controller = None
        self.handle = None
    
    def setup(self, ip, mode=None):
        self.controller = RoboticArm(mode)
        self.handle = self.controller.rm_create_robot_arm(ip, 8080)
    
    def get_state(self):
        return self.controller.rm_get_current_arm_state()[1]["pose"]

def test_class():
    t1 = test_rm()
    t2 = test_rm()

    t1.setup("192.168.80.18",rm_thread_mode_e.RM_TRIPLE_MODE_E)
    t2.setup("192.168.80.19",rm_thread_mode_e.RM_TRIPLE_MODE_E)
    print("init success!")
    print(t1.get_state())
    print(t2.get_state())

def test_anti_class():
    t1 = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    # t1 = RoboticArm()
    t1_handle = t1.rm_create_robot_arm("192.168.80.18", 8080)
    
    # t2 = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    t2 = RoboticArm()
    t2_handle = t2.rm_create_robot_arm("192.168.80.19", 8080)

    print(t1.rm_get_current_arm_state())

    print(t2.rm_get_current_arm_state())

def test_q():
    t1 = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    t1_handle = t1.rm_create_robot_arm("192.168.80.19", 8080)
    print(t1.rm_get_current_work_frame())
    print(t1.rm_get_current_tool_frame())
    print(t1.rm_get_current_arm_state()[1]["pose"][-3:])

if __name__ == "__main__":
    # test_class()
    test_anti_class()
    # test_q()
