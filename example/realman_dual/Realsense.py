import numpy as np
import pyrealsense2 as rs
import time

def find_device_by_serial(devices, serial):
    """Find device index by serial number"""
    for i, dev in enumerate(devices):
        if dev.get_info(rs.camera_info.serial_number) == serial:
            return i
    return None

class Img_controller:
    def __init__(self,CAMERA_SERIALS):
        try:
            # Initialize RealSense context and check for connected devices
            self.ctx = rs.context()
            self.devices = list(self.ctx.query_devices())
            
            if not self.devices:
                raise RuntimeError("No RealSense devices found")
            
            print("\nDetected RealSense devices:")
            for i, dev in enumerate(self.devices):
                print(f"Device {i}: {dev.get_info(rs.camera_info.name)} (SN: {dev.get_info(rs.camera_info.serial_number)})")
            
            # Initialize pipelines and configs
            self.pipelines = {}
            self.configs = {}
            
            # Initialize each camera
            for camera_name, serial in CAMERA_SERIALS.items():
                device_idx = find_device_by_serial(self.devices, serial)
                if device_idx is None:
                    raise RuntimeError(f"Could not find {camera_name} camera with serial number {serial}")
                
                self.pipelines[camera_name] = rs.pipeline()
                self.configs[camera_name] = rs.config()
                
                # Enable device by serial number
                self.configs[camera_name].enable_device(serial)
                
                # Enable color stream only
                self.configs[camera_name].enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                
                # Start streaming
                try:
                    self.pipelines[camera_name].start(self.configs[camera_name])
                    print(f"Started {camera_name} camera (SN: {serial})")
                except RuntimeError as e:
                    raise RuntimeError(f"Error starting {camera_name} camera: {str(e)}")
        except Exception as e:
            self.cleanup()
            raise RuntimeError(f"Failed to initialize cameras: {str(e)}")

    def get_img(self):
        try:
            # Dictionary to store images
            images = {}
            
            # Get frames from cameras in order (head, right_wrist)
            for camera_name in ['head', 'right_wrist','left_wrist']:
                frames = self.pipelines[camera_name].wait_for_frames(timeout_ms=5000)
                if not frames:
                    raise RuntimeError(f"Timeout waiting for {camera_name} camera frames")
                
                # Get color frame
                color_frame = frames.get_color_frame()
                if not color_frame:
                    raise RuntimeError(f"Failed to get color frame from {camera_name} camera")
                
                # Convert to numpy array
                color_image = np.asanyarray(color_frame.get_data())
                
                # Convert BGR to RGB
                # color_image = color_image[..., [2, 1, 0]]
                
                images[camera_name] = {
                    'color': color_image
                }
            
            return images
        except Exception as e:
            raise RuntimeError(f"Error capturing images: {str(e)}")

    def cleanup(self):
        try:
            if hasattr(self, 'pipelines'):
                for pipeline in self.pipelines.values():
                    pipeline.stop()
        except Exception as e:
            print(f"Error during cleanup: {str(e)}")

    def __del__(self):
        self.cleanup()
