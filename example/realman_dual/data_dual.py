import os
import PIL.Image
import cv2
import numpy as np
import PIL

class CollectData:
    def __init__(self, right_arm_union, right_gripper, left_arm_union, left_gripper, imgs):
        # Correctly concatenate arrays by passing them as a list/tuple to np.concatenate
        self.joint = np.concatenate([right_arm_union["joint"], left_arm_union["joint"]])
        self.imgs = imgs  # Dictionary containing images from all cameras
        # Convert scalars to arrays before concatenation if needed
        right_gripper_val = np.array([right_gripper])
        left_gripper_val = np.array([left_gripper])
        self.gripper = np.concatenate([right_gripper_val, left_gripper_val])
        self.pos = np.concatenate([right_arm_union["pose"], left_arm_union["pose"]])
        
    def write(self, path, index):
        """Save data to files in the specified directory"""
        # Ensure path exists
        if not os.path.exists(path):
            os.makedirs(path)
            
        # Prepare data dictionary
        data = {
            'joint': np.array(self.joint, dtype=np.float32),
            'pose': np.array(self.pos, dtype=np.float32),
            'gripper': np.array(self.gripper, dtype=np.float32),
        }
        
        # Add camera data to dictionary
        for camera_name in ['left_wrist','right_wrist', 'head']:
            data[f'{camera_name}_color'] = np.array(self.imgs[camera_name]['color'])
            # data[f'{camera_name}_depth'] = np.array(self.imgs[camera_name]['depth'])
            
            # Save color image
            color_path = os.path.join(path, f"{camera_name}_{index}.jpg")
            cv2.imwrite(color_path, self.imgs[camera_name]['color'])
            
            # Save depth image with colormap
            # depth_colorized = cv2.applyColorMap(
            #     cv2.convertScaleAbs(self.imgs[camera_name]['depth'], alpha=0.03),
            #     cv2.COLORMAP_JET
            # )
            # depth_path = os.path.join(path, f"{camera_name}_depth_{index}.jpg")
            # cv2.imwrite(depth_path, self.imgs[camera_name]['depth'])
        
        # Save all data as .npy file
        data_path = os.path.join(path, f"targ{index}.npy")
        np.save(data_path, data)
