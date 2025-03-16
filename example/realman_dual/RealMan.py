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

    def get_state(self):
        try:
            # Get arm state
            succ, arm_state = self.arm_controller.rm_get_current_arm_state()
            if succ != 0 or arm_state is None:
                raise RuntimeError("Failed to get arm state")
                
            state = arm_state['pose'].copy()

            return state
        except Exception as e:
            raise RuntimeError(f"Error getting robot state: {str(e)}")

    def move(self, tech_state):
        try:
            # Validate state length
            if len(tech_state) != 6:  # 6 joints
                raise ValueError(f"Invalid state length: {len(tech_state)}")
            if self.prev_tech_prev_tech_stateeef == None:
                print(f"DEBUG: Frist time setting EEF:{tech_state}")
                self.prev_tech_state = tech_state
                return
            state = self.get_state()
            print (f"DEBUG: Setting EEF: {state},action:{tech_state}")
            # delta postion, abs angle
            next_state = [state[:3] + (tech_state[:3] - self.prev_tech_state[:3]),tech_state[:-3]]
            success = self.arm_controller.rm_movej_p(next_state, False, 0, 0, 0)
            self.prev_tech_state = tech_state

            if success != 0:
                raise RuntimeError("Failed to set joint angles")
                
        except Exception as e:
            raise RuntimeError(f"Error moving robot: {str(e)}")
    
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

def initialize_robot(robot_ip, thread_mode=None, connection_level=3):
    """Initialize robot arm controller and establish connection"""
    print(f"\nInitializing robot at {robot_ip} with connection level {connection_level}...")
    
    # Create a new instance with the specified thread mode
    if thread_mode is not None:
        print(f"Using thread mode: {thread_mode}")
        robot_controller = RoboticArm(thread_mode)
    else:
        print("Using default thread mode")
        # Default to single thread mode if none specified
        robot_controller = RoboticArm()
    
    # Try to connect with retry logic
    max_retries = 3
    for attempt in range(max_retries):
        print(f"Connecting to robot at {robot_ip}, attempt {attempt+1}/{max_retries}...")
        handle = robot_controller.rm_create_robot_arm(robot_ip, 8080, connection_level)
        
        if handle.id != -1:
            print(f"Successfully connected to robot at {robot_ip}, handle ID: {handle.id}")
            # Verify connection is active
            succ, state = robot_controller.rm_get_current_arm_state()
            if succ == 0:
                print(f"Connection verified for robot at {robot_ip}")
                print(f"Current state: {state}")
                
                # Get robot info for additional verification
                succ, info = robot_controller.rm_get_robot_info()
                if succ == 0:
                    print(f"Robot info: {info}")
                
                return robot_controller, handle
            else:
                print(f"Connection established but couldn't get state from robot at {robot_ip}. Error code: {succ}")
        else:
            print(f"Failed to create robot arm handle for {robot_ip}. Handle ID: {handle.id}")
        
        if attempt < max_retries - 1:
            print(f"Failed to connect to robot at {robot_ip}, retrying in 2 seconds...")
            time.sleep(2)
    
    print(f"Failed to connect to robot at {robot_ip} after {max_retries} attempts")
    return None
