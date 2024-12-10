import sys
sys.path.append("/home/test/ros_hk_cam/src/hk_cam_py/scripts")

import cv2
import os
from datetime import datetime
from threading import Lock

import rospy

from jk_cam import JKCam
# from jk_cam_py.msg import Point2f, Vector3f
from jk_cam_py.srv import captureResponse, captureRequest
from jk_cam_py.srv import setExposureTimeResponse, setExposureTimeRequest


# Define constants
DEFAULT_IMG_SAVING_ROOT_DIR = "/home/test/jk_cam_base/outputs/images/"


def make_capture(cam: JKCam, cam_lock: Lock):
    def capture_image(req: captureRequest):
        with cam_lock:
            image = cam.grab_image()
        
        cam_id = cam.cam_params['DeviceUserID']
        now = datetime.now()
        time_string = now.strftime("%Y%m%d_%H_%M_%S")

        if not req.saving_path:
            save_path = f"{DEFAULT_IMG_SAVING_ROOT_DIR}{cam_id}/" 
        else:
            save_path = req.saving_path        
        if not req.file_format:
            img_format = 'bmp'
        else:
            img_format = req.file_format

        os.makedirs(save_path, exist_ok=True)
        file_path = f"{save_path}{time_string}.{img_format}"
        cv2.imwrite(file_path, image)
        rospy.loginfo(f"{cam_id} saved 1 image: {file_path}")
        return captureResponse()
    return capture_image


def make_set_exposure_time(cam: JKCam):
    def set_exposure_time(req: setExposureTimeRequest):
        old_exposure = rospy.get_param("~ExposureTime", 70000.0)
        cam_id = cam.cam_params['DeviceUserID']
        exposure_time = req.exposure
        if cam.set_exposure_time(exposure_time):
            rospy.loginfo(f"{cam_id}: Set exposure time from {old_exposure}us to {exposure_time}us.")  
        return setExposureTimeResponse()
    return set_exposure_time

