from Robotic_Arm.rm_robot_interface import *
import time
import numpy as np

class RM_controller:
    def __init__(self, rm_ip, thread_mode=None):
        try:
            result = initialize_robot(rm_ip, thread_mode)
            if result is None:
                raise ConnectionError(f"Failed to connect to robot arm at {rm_ip}")
            self.arm_controller, self.handle = result
        except Exception as e:
            raise ConnectionError(f"Failed to initialize robot arm: {str(e)}")
        self.prev_tech_state = None
        self.gripper_close = False
        self.delta = [0, 0, 0]

    def get_state(self):
        return self.arm_controller.rm_get_current_arm_state()[1]["pose"]

    def get_gripper(self):
        return self.arm_controller.rm_get_gripper_state()[1]

    def move_test(self):
        try:
            succ, arm_state = self.arm_controller.rm_get_current_arm_state()
            print("succ :", succ)
        except Exception as e:
            raise RuntimeError(f"Error moving robot: {str(e)}")

    def move(self, tech_state):
        # state= self.get_state()
        # Validate state length
        if len(tech_state) == 6 :  # 6D pose
            print("BGINING MOVING-------6D pose")
        elif len(tech_state) == 7:  # 7D pose
            print("Quaternion pose")
        else:
            raise ValueError(f"Invalid state length: {len(tech_state)}")
        if self.prev_tech_state == None:
            print(f"DEBUG: Frist time setting EEF:{tech_state}")
            self.prev_tech_state = tech_state
            return
        state= self.get_state()
        self.delta[0]=tech_state[0]-self.prev_tech_state[0]
        self.delta[1]=tech_state[1]-self.prev_tech_state[1]
        self.delta[2]=tech_state[2]-self.prev_tech_state[2]
        next_state = [state[0]+ self.delta[0],  
                        state[1]+self.delta[1], 
                        state[2]+self.delta[2], 
                        state[3],
                        state[4],
                        state[5]
                        ] 
        # print("new_state:",state)

        # print (f"DEBUG: Setting EEF: {state},action:{tech_state}")
        # delta postion, abs angle
        # next_state = [state[:3] + (tech_state[:3] - self.prev_tech_state[:3]),tech_state[:-3]]
        """
        self.delta[0]=tech_state[0]-self.prev_tech_state[0]+self.delta[0]
        self.delta[1]=tech_state[1]-self.prev_tech_state[1]+self.delta[1]
        self.delta[2]=tech_state[2]-self.prev_tech_state[2]+self.delta[2]
        next_state = state.copy()  # 复制当前状态
        new_delta = self.delta.copy()    
        if abs(new_delta[0]) > 0.1:
            next_state[0] += new_delta[0]
            new_delta[0] = 0

        if abs(new_delta[1]) > 0.1:
            next_state[1] += new_delta[1]
            new_delta[1] = 0

        if abs(new_delta[2]) > 0.1:
            next_state[2] += new_delta[2]
            new_delta[2] = 0

        # 其他状态保持不变
        next_state[3] = state[3]
        next_state[4] = state[4]
        next_state[5] = state[5]
        # 更新 delta
        self.delta = new_delta
        print("delta:\n",self.delta)
        """
        # print("tech_state:\n",tech_state[-3:])

        # print("delta state:\n",delta_state)

        # print(f"-----next state-----: {next_state}")
        success = self.arm_controller.rm_movep_canfd(next_state,False,0,0)
        # success = self.arm_controller.rm_movep_canfd(next_state,True,1,999)

        self.prev_tech_state = tech_state
    
    def set_gripper(self, gripper):
        try:
            if gripper < 0.10 and not self.gripper_close:
                success = self.arm_controller.rm_set_gripper_pick(100, 100, 0, 0)
                if success:
                    self.gripper_close = True
                else:
                    raise RuntimeError("Failed to close gripper")
            elif gripper > 0.9 and self.gripper_close:
                success = self.arm_controller.rm_set_gripper_release(100, 0, 0)
                if success:
                    self.gripper_close = False
                else:
                    raise RuntimeError("Failed to open gripper")
        except Exception as e:
            raise RuntimeError(f"Error setting gripper pos: {str(e)}")

    def __del__(self):
        try:
            if hasattr(self, 'arm_controller'):
                # Add any necessary cleanup for the arm controller
                pass
        except:
            pass

def initialize_robot(ip, mode=None):
    robot = RoboticArm(mode)
    handle = robot.rm_create_robot_arm(ip, 8080) 
    return  robot, handle

# def initialize_robot(robot_ip, thread_mode=None, connection_level=3):
#     """Initialize robot arm controller and establish connection"""
#     print(f"\nInitializing robot at {robot_ip} with connection level {connection_level}...")
    
#     # Create a new instance with the specified thread mode
#     if thread_mode is not None:
#         print(f"Using thread mode: {thread_mode}")
#         robot_controller = RoboticArm(thread_mode)
#     else:
#         print("Using default thread mode")
#         # Default to single thread mode if none specified
#         robot_controller = RoboticArm()
    
#     # Try to connect with retry logic
#     max_retries = 3
#     for attempt in range(max_retries):
#         print(f"Connecting to robot at {robot_ip}, attempt {attempt+1}/{max_retries}...")
#         handle = robot_controller.rm_create_robot_arm(robot_ip, 8080, connection_level)
        
#         if handle.id != -1:
#             print(f"Successfully connected to robot at {robot_ip}, handle ID: {handle.id}")
#             # Verify connection is active
#             succ, state = robot_controller.rm_get_current_arm_state()
#             print(f"state:::::{state}")
            
#             if succ == 0:
#                 print(f"Connection verified for robot at {robot_ip}")
#                 print(f"Current state: {state}")
                
#                 # Get robot info for additional verification
#                 succ, info = robot_controller.rm_get_robot_info()
#                 if succ == 0:
#                     print(f"Robot info: {info}")
                
#                 return robot_controller, handle
#             else:
#                 print(f"Connection established but couldn't get state from robot at {robot_ip}. Error code: {succ}")
#         else:
#             print(f"Failed to create robot arm handle for {robot_ip}. Handle ID: {handle.id}")
        
#         if attempt < max_retries - 1:
#             print(f"Failed to connect to robot at {robot_ip}, retrying in 2 seconds...")
#             time.sleep(2)
    
#     print(f"Failed to connect to robot at {robot_ip} after {max_retries} attempts")
#     return None
