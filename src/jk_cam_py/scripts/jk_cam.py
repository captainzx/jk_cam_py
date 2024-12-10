# -- coding: utf-8 --

import sys
# Change this path according to your own environment
sys.path.append("/opt/HuarayTech/MVviewer/share/Python/MVSDK")

import yaml
import rospy
import numpy as np

from IMVApi import *
from ctypes import *
from cv_bridge import CvBridge


class JKCam(MvCamera):
    def __init__(self):
        super().__init__()
        # Camera parameters concerned with ACQUISITION
        # That is what you write in /config/camera_x.yaml file
        self.cam_params = {}
        # Object used to convert between OpenCV Images and ROS image messages
        self.bridge = CvBridge() 
        self.camera_matrix = None
        self.distortion_coefficients = None


    def load_params(self):
        # Load parameters from ROS parameter server
        self.cam_params['Width'] = rospy.get_param('~Width', 2592)
        self.cam_params['Height'] = rospy.get_param('~Height', 1944)
        self.cam_params['ExposureTime'] = \
            rospy.get_param('~ExposureTime', 50000.0)
        self.cam_params['TriggerMode'] = \
            rospy.get_param('~TriggerMode', 0)
        self.cam_params['PixelFormat'] = \
            rospy.get_param('~PixelFormat', 'Mono8')
        self.cam_params['DeviceUserID'] = \
            rospy.get_param('~DeviceUserID', 'cam_hand')
        self.cam_params['DeviceSerialNumber'] = \
            rospy.get_param('~DeviceSerialNumber', 'DF46210BAK00018')
        

    def load_intrinsic_params(self):
        # Load intrinsic parameters from ROS parameter server
        cam_info_path = rospy.get_param('~camera_info_path')
        with open(cam_info_path, 'r') as f:
            camera_info = yaml.safe_load(f)
        self.camera_matrix = \
            np.array(camera_info['camera_matrix']['data']).reshape((3, 3))
        self.distortion_coefficients = \
            np.array(camera_info['distortion_coefficients']['data'])
        

    def initialize(self):
        rospy.loginfo("Initializing camera...")

        deviceList = IMV_DeviceList()
        interfaceType = IMV_EInterfaceType.interfaceTypeAll
        ret = MvCamera.IMV_EnumDevices(deviceList, interfaceType)
        if ret != IMV_OK:
            rospy.logerr("Enum devices failed! ErrorCode[0x%x]" % ret)
            return False
        
        if deviceList.nDevNum == 0:
            rospy.logerr("No devices found!")
            return False       
        rospy.loginfo("Find %d devices!" % deviceList.nDevNum)

        # List information of all cameras
        # Set nConnectionNum according to Serial Number assigned in
        # /config/camera_x.yaml file
        nConnectionNum = 0
        for i in range(0, deviceList.nDevNum):
            pDeviceInfo = deviceList.pDevInfo[i]
            rospy.loginfo(f"==Device {i}==")
            # Maybe you need not to print the information of all cameras
            JKCam.print_device_info(pDeviceInfo)
            strSerialNo = pDeviceInfo.serialNumber.decode("ascii")           
            if (strSerialNo == self.cam_params["DeviceSerialNumber"]):
                nConnectionNum = i
            
        # Select device[nConnectionNum] and create handle
        rospy.loginfo("Creating handle for camera[%x]" % nConnectionNum)
        ret = self.IMV_CreateHandle(IMV_ECreateHandleMode.modeByIndex,byref(c_void_p(int(nConnectionNum))))
        if ret != IMV_OK:
            rospy.logerr(f"{self.cam_params['DeviceUserID']}: Create device handle failed! ErrorCode[0x{ret:x}]")
            return False
            
        # Open device
        rospy.loginfo(f"Opening {self.cam_params['DeviceUserID']}...")
        ret = self.IMV_Open()
        if ret != IMV_OK:
            rospy.logerr(f"{self.cam_params['DeviceUserID']}: Open device failed! ErrorCode[0x{ret:x}]")
            return False 

        # Set trigger mode (Default: off)
        rospy.loginfo(f"{self.cam_params['DeviceUserID']}: Turning off trigger mode...")
        ret = self.IMV_SetEnumFeatureValue("TriggerMode", 
                                           self.cam_params['TriggerMode'])
        if ret != 0:
            rospy.logerr(f"{self.cam_params['DeviceUserID']}: set trigger mode fail! ErrorCode[0x{ret:x}]")
            return False
            
        # Set camera parameters
        rospy.loginfo(f"{self.cam_params['DeviceUserID']}: Setting camera parameters...")
        # Set image size
        ret = self.IMV_SetIntFeatureValue(pFeatureName="Width", 
                                          pIntValue=self.cam_params['Width'])
        if ret != IMV_OK:
            rospy.logerr(f"{self.cam_params['DeviceUserID']}: Set image width failed! ErrorCode[0x{ret:x}]")
            return False
        ret = self.IMV_SetIntFeatureValue(pFeatureName="Height", 
                                          pIntValue=self.cam_params['Height'])
        if ret != IMV_OK:
            rospy.logerr(f"{self.cam_params['DeviceUserID']}: Set image Height failed! ErrorCode[0x{ret:x}]")
            return False
        
        # Set exposure time
        ret = self.IMV_SetDoubleFeatureValue(pFeatureName='ExposureTime', 
                                             doubleValue=self.cam_params['ExposureTime'])
        if ret != IMV_OK:
            rospy.logerr(f"{self.cam_params['DeviceUserID']}: Set exposure time failed! ErrorCode[0x{ret:x}]")
            return False
        
        # Set pixel format
        ret = self.IMV_SetEnumFeatureSymbol(pFeatureName='PixelFormat', 
                                            pEnumSymbol=self.cam_params['PixelFormat'])
        if ret != IMV_OK:
            rospy.logerr(f"{self.cam_params['DeviceUserID']}: Set pixel format failed! ErrorCode[0x{ret:x}]")
            return False  
        
        # Start grabbing image
        rospy.loginfo(f"{self.cam_params['DeviceUserID']}: Starting grabbing...")
        ret = self.IMV_StartGrabbing()
        if ret != IMV_OK:
            rospy.logerr(f"{self.cam_params['DeviceUserID']}: Start grabbing failed! ErrorCode[0x{ret:x}]")
            return False

        rospy.loginfo(f"{self.cam_params['DeviceUserID']}: Initialization finished!")

        return True
    

    # May not need to print these information
    def print_device_info(pDeviceInfo):
        if not pDeviceInfo:
            rospy.loginfo("The pointer to device info is NULL!")
            return False
        
        strType=""
        strVendorName = pDeviceInfo.vendorName.decode("ascii")
        strModeName = pDeviceInfo.modelName.decode("ascii")
        strSerialNumber = pDeviceInfo.serialNumber.decode("ascii")
        strCameraname = pDeviceInfo.cameraName.decode("ascii")
        strIpAdress = pDeviceInfo.DeviceSpecificInfo.gigeDeviceInfo.ipAddress.decode("ascii")
        if pDeviceInfo.nCameraType == typeGigeCamera:
            strType="Gige"
        elif pDeviceInfo.nCameraType == typeU3vCamera:
            strType="U3V"
        rospy.loginfo(f"{strType:<8}{strVendorName:<20}{strModeName:<15}{strSerialNumber:<20}{strCameraname:<10}{strIpAdress:<18}")
        
    
    def shutdown(self):
        rospy.loginfo(f"{self.cam_params['DeviceUserID']} shutting down...")
        # Stop grab image
        ret = self.IMV_StopGrabbing()
        if ret != IMV_OK:
            rospy.logerr(f"{self.cam_params['DeviceUserID']}: Stop grabbing failed! ErrorCode[0x{ret}]")
            return
        rospy.loginfo(f"{self.cam_params['DeviceUserID']}: Grabbing stopped!")
        
        # Close device
        ret = self.IMV_Close()
        if ret != IMV_OK:
            rospy.logerr(f"{self.cam_params['DeviceUserID']}: Close deivce failed! ErrorCode[0x{ret:x}]")
            return
        rospy.loginfo(f"{self.cam_params['DeviceUserID']}: Camera closed!")

        # Destroy handle
        if (self.handle):
            ret = self.IMV_DestroyHandle()
            if ret != IMV_OK:
                rospy.logerr(f"{self.cam_params['DeviceUserID']}: Destroy handle failed! ErrorCode[0x{ret:x}]")
                return
        rospy.loginfo(f"{self.cam_params['DeviceUserID']}: Camera handle destroyed!")

        rospy.loginfo(f"{self.cam_params['DeviceUserID']}: Camera shut down!")

        return   

    
    def grab_ros_image(self):
        frame = IMV_Frame()
        memset(byref(frame), 0, sizeof(frame))
        ret = self.IMV_GetFrame(frame, 500)
        if (ret == IMV_OK) and (frame.pData is not None):
            stPixelConvertParam=IMV_PixelConvertParam()

            """ // mono8和BGR8裸数据不需要转码
            // mono8 and BGR8 raw data: need not to convert """
            if ((frame.frameInfo.pixelFormat != IMV_EPixelType.gvspPixelMono8)
                & (frame.frameInfo.pixelFormat != IMV_EPixelType.gvspPixelBGR8)):
                nSize = frame.frameInfo.width * frame.frameInfo.height * 3
                g_pConvertBuf = (c_ubyte * nSize)()
                """ // 图像转换成BGR8
                // convert image to BGR8 """
                memset(byref(stPixelConvertParam), 0, sizeof(stPixelConvertParam))
                stPixelConvertParam.nWidth = frame.frameInfo.width
                stPixelConvertParam.nHeight = frame.frameInfo.height
                stPixelConvertParam.ePixelFormat = frame.frameInfo.pixelFormat
                stPixelConvertParam.pSrcData = frame.pData
                stPixelConvertParam.nSrcDataLen = frame.frameInfo.size
                stPixelConvertParam.nPaddingX = frame.frameInfo.paddingX
                stPixelConvertParam.nPaddingY = frame.frameInfo.paddingY
                stPixelConvertParam.eBayerDemosaic = IMV_EBayerDemosaic.demosaicNearestNeighbor
                stPixelConvertParam.eDstPixelFormat = IMV_EPixelType.gvspPixelBGR8
                stPixelConvertParam.pDstBuf = g_pConvertBuf
                stPixelConvertParam.nDstBufSize = frame.frameInfo.width * frame.frameInfo.height * 3
                ret = self.IMV_PixelConvert(stPixelConvertParam)
                if IMV_OK != ret:
                    rospy.loginfo("Image convert to BGR failed! ErrorCode[%d]", ret)
                    return False
                pImageData = g_pConvertBuf
                pixelFormat = IMV_EPixelType.gvspPixelBGR8
            else:
                pImageData = frame.pData
                pixelFormat = frame.frameInfo.pixelFormat

            if pixelFormat == IMV_EPixelType.gvspPixelMono8:
                imageSize = frame.frameInfo.width * frame.frameInfo.height
            else:
                imageSize = frame.frameInfo.width * frame.frameInfo.height * 3

            userBuff = c_buffer(b'\0', imageSize)
            memmove(userBuff, pImageData, imageSize)
            if pixelFormat == IMV_EPixelType.gvspPixelMono8:
                numpy_image = np.frombuffer(userBuff, dtype=np.ubyte, count=imageSize). \
                    reshape(frame.frameInfo.height, frame.frameInfo.width)
            else:
                numpy_image = np.frombuffer(userBuff, dtype=np.ubyte, count=imageSize). \
                    reshape(frame.frameInfo.height, frame.frameInfo.width, 3)

            ros_image = self.bridge.cv2_to_imgmsg(numpy_image, "passthrough")

            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = str(frame.frameInfo.blockId)

            self.IMV_ReleaseFrame(frame)
            return ros_image
        else:
            rospy.logerr(f"{self.cam_params['DeviceUserID']}: Get image failed! ErrorCode[0x{ret:x}]")
            return None
    
    
    def grab_image(self):
        frame = IMV_Frame()
        memset(byref(frame), 0, sizeof(frame))
        ret = self.IMV_GetFrame(frame, 500)
        if (ret == IMV_OK) and (frame.pData is not None):
            stPixelConvertParam=IMV_PixelConvertParam()

            """ // mono8和BGR8裸数据不需要转码
            // mono8 and BGR8 raw data: need not to convert """
            if ((frame.frameInfo.pixelFormat != IMV_EPixelType.gvspPixelMono8)
                & (frame.frameInfo.pixelFormat != IMV_EPixelType.gvspPixelBGR8)):
                nSize = frame.frameInfo.width * frame.frameInfo.height * 3
                g_pConvertBuf = (c_ubyte * nSize)()
                """ // 图像转换成BGR8
                // convert image to BGR8 """
                memset(byref(stPixelConvertParam), 0, sizeof(stPixelConvertParam))
                stPixelConvertParam.nWidth = frame.frameInfo.width
                stPixelConvertParam.nHeight = frame.frameInfo.height
                stPixelConvertParam.ePixelFormat = frame.frameInfo.pixelFormat
                stPixelConvertParam.pSrcData = frame.pData
                stPixelConvertParam.nSrcDataLen = frame.frameInfo.size
                stPixelConvertParam.nPaddingX = frame.frameInfo.paddingX
                stPixelConvertParam.nPaddingY = frame.frameInfo.paddingY
                stPixelConvertParam.eBayerDemosaic = IMV_EBayerDemosaic.demosaicNearestNeighbor
                stPixelConvertParam.eDstPixelFormat = IMV_EPixelType.gvspPixelBGR8
                stPixelConvertParam.pDstBuf = g_pConvertBuf
                stPixelConvertParam.nDstBufSize = frame.frameInfo.width * frame.frameInfo.height * 3
                ret = self.IMV_PixelConvert(stPixelConvertParam)
                if IMV_OK != ret:
                    rospy.loginfo("Image convert to BGR failed! ErrorCode[%d]", ret)
                    return False
                pImageData = g_pConvertBuf
                pixelFormat = IMV_EPixelType.gvspPixelBGR8
            else:
                pImageData = frame.pData
                pixelFormat = frame.frameInfo.pixelFormat

            if pixelFormat == IMV_EPixelType.gvspPixelMono8:
                imageSize = frame.frameInfo.width * frame.frameInfo.height
            else:
                imageSize = frame.frameInfo.width * frame.frameInfo.height * 3

            userBuff = c_buffer(b'\0', imageSize)
            memmove(userBuff, pImageData, imageSize)
            if pixelFormat == IMV_EPixelType.gvspPixelMono8:
                numpy_image = np.frombuffer(userBuff, dtype=np.ubyte, count=imageSize). \
                    reshape(frame.frameInfo.height, frame.frameInfo.width)
            else:
                numpy_image = np.frombuffer(userBuff, dtype=np.ubyte, count=imageSize). \
                    reshape(frame.frameInfo.height, frame.frameInfo.width, 3)

            self.IMV_ReleaseFrame(frame)
            return numpy_image
        else:
            rospy.logerr(f"{self.cam_params['DeviceUserID']}: Get image failed! ErrorCode[0x{ret:x}]")
            return None

    
    def set_exposure_time(self, exposure):
        rospy.set_param("~ExposureTime", exposure)
        self.cam_params['ExposureTime'] = exposure
        ret = self.IMV_SetDoubleFeatureValue(pFeatureName='ExposureTime', 
                                             doubleValue=exposure)
        if ret != IMV_OK:
            rospy.logerr(f"{self.cam_params['DeviceUserID']}: Set exposure time failed! ret[0x{ret:x}]")
            return False
        return True
    
    
    # Add other functions if you need
    def set_acquisition_mode(self, mode):
        pass
    
    def set_acquisition_frame_rate(self, rate):
        pass
    
    
    def __del__(self):
        self.shutdown()