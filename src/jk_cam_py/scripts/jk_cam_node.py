#!/usr/bin/env python3
import sys
# Change this path according to your own environment
sys.path.append("/home/test/jk_cam_base/src/jk_cam_py/scripts")

from ctypes import c_double
import threading

import rospy
import camera_calibration_parsers
from sensor_msgs.msg import Image, CameraInfo

from jk_cam import JKCam
import service_callbacks as sc
from jk_cam_py.srv import capture
from jk_cam_py.srv import setExposureTime

def main():
    rospy.init_node('hk_cam')

    # Define a thread lock for camera access
    # So that services and publishers won't access the camera at the same time
    # Thus avoiding potential conflicts
    cam_lock = threading.Lock()

    # Define a camera object and initialize it
    cam = JKCam()
    cam.load_params()  # load configuration from .yaml file
    cam.load_intrinsic_params()  # load intrinsic parameters from .yaml file
    camera_name = cam.cam_params['DeviceUserID']
    if not cam.initialize():
        rospy.logerr(f"{camera_name}: Initialization failed, \
                     Check camera parameters from yaml file!")
        return False

    # Define services
    rospy.Service(name=f"{camera_name}/capture",
                  service_class=capture,
                  handler=sc.make_capture(cam, cam_lock))
    rospy.Service(name=f"{camera_name}/set_exposure_time",
                  service_class=setExposureTime,
                  handler=sc.make_set_exposure_time(cam))

    # If pub_image is False, the node will only provide services
    if not rospy.get_param('~pub_image'):
        rospy.loginfo(f"{camera_name}: waiting for service call...")
        rospy.spin()
    else:
        # Load camera_info, a.k.a the intrinsic parameters
        cam_info_path = rospy.get_param('~camera_info_path')
        camera_name, cam_info = \
            camera_calibration_parsers.readCalibration(cam_info_path)
        if cam_info is None:
            rospy.logerr(f"Failed to read camera calibration from file: {cam_info_path}")
        else:
            rospy.loginfo(f"{camera_name} read camera calibration from file: {cam_info_path}")

        # Define topic publishers
        img_pub = rospy.Publisher(name=f"{camera_name}/image_raw",
                                  data_class=Image,
                                  queue_size=10)
        cam_info_pub = rospy.Publisher(name=f"{camera_name}/camera_info",
                                       data_class=CameraInfo,
                                       queue_size=10)

        
        # Setting cycle frequency to the acquisition frame rate of camera
        # (Maybe a little lower)
        cam_rate = c_double(0.0)
        cam.IMV_GetDoubleFeatureValue(pFeatureName='AcquisitionFrameRate',
                                      pDoubleValue=cam_rate)
        rospy.loginfo(f"Fram Rate of {camera_name} is: {cam_rate.value}")
        rate = rospy.Rate(cam_rate.value)
        rospy.loginfo(f"{camera_name}: waiting for service call while publishing image message...")
        while not rospy.is_shutdown():
            try:
                with cam_lock:
                    image = cam.grab_ros_image()
                if image is not None:
                    img_pub.publish(image)
                    cam_info_pub.publish(cam_info)
                else:
                    rospy.logwarn(f"{camera_name}: Failed to grab image.")
            except Exception as e:
                rospy.logerr(f"{camera_name}: Exception occurred: {e}")
            rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        pass
