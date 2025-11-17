# -*- coding: utf-8 -*-
"""
相机操作模块 - 提供相机设备的低级接口和线程处理
用途：实现相机连接、图像采集、参数设置等功能
主要类：CameraOperation（相机操作类）、CameraGrabThread（取图线程类）
"""

import threading
import time
import sys
import inspect
import ctypes
import random
import os
from ctypes import *
import cv2
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

from hikvisionSDK.CameraParams_header import *
from hikvisionSDK.MvCameraControl_class import *


def Async_raise(tid, exctype):
    """
    强制关闭指定线程（危险操作，谨慎使用）
    
    参数:
        tid: 线程ID
        exctype: 异常类型
    
    说明：
        - 这是Python线程的强制关闭方法
        - 可能导致资源泄漏，仅在必要时使用
    """
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def Stop_thread(thread):
    """
    停止指定的线程
    
    参数:
        thread: 要停止的线程对象
    """
    Async_raise(thread.ident, SystemExit)


def To_hex_str(num):
    """
    将整数转换为十六进制字符串
    
    参数:
        num: 要转换的整数（支持负数）
    
    返回:
        十六进制字符串表示（小写字母）
    """
    chaDic = {10: 'a', 11: 'b', 12: 'c', 13: 'd', 14: 'e', 15: 'f'}
    hexStr = ""
    if num < 0:
        num = num + 2 ** 32
    while num >= 16:
        digit = num % 16
        hexStr = chaDic.get(digit, str(digit)) + hexStr
        num //= 16
    hexStr = chaDic.get(num, str(num)) + hexStr
    return hexStr


def Is_mono_data(enGvspPixelType):
    """
    检查像素格式是否为黑白（单通道）图像
    
    参数:
        enGvspPixelType: 像素格式枚举值
    
    返回:
        True: 是黑白图像，False: 否
    
    支持的格式：
        - Mono8: 8位黑白
        - Mono10: 10位黑白
        - Mono12: 12位黑白
    """
    if PixelType_Gvsp_Mono8 == enGvspPixelType or PixelType_Gvsp_Mono10 == enGvspPixelType \
            or PixelType_Gvsp_Mono10_Packed == enGvspPixelType or PixelType_Gvsp_Mono12 == enGvspPixelType \
            or PixelType_Gvsp_Mono12_Packed == enGvspPixelType:
        return True
    else:
        return False


def Is_color_data(enGvspPixelType):
    """
    检查像素格式是否为彩色（Bayer阵列）图像
    
    参数:
        enGvspPixelType: 像素格式枚举值
    
    返回:
        True: 是彩色图像，False: 否
    
    支持的格式：
        - BayerRG/GR/GB/BG: 8位、10位、12位Bayer格式
        - YUV422: 视频编码格式
    """
    if PixelType_Gvsp_BayerGR8 == enGvspPixelType or PixelType_Gvsp_BayerRG8 == enGvspPixelType \
            or PixelType_Gvsp_BayerGB8 == enGvspPixelType or PixelType_Gvsp_BayerBG8 == enGvspPixelType \
            or PixelType_Gvsp_BayerGR10 == enGvspPixelType or PixelType_Gvsp_BayerRG10 == enGvspPixelType \
            or PixelType_Gvsp_BayerGB10 == enGvspPixelType or PixelType_Gvsp_BayerBG10 == enGvspPixelType \
            or PixelType_Gvsp_BayerGR12 == enGvspPixelType or PixelType_Gvsp_BayerRG12 == enGvspPixelType \
            or PixelType_Gvsp_BayerGB12 == enGvspPixelType or PixelType_Gvsp_BayerBG12 == enGvspPixelType \
            or PixelType_Gvsp_BayerGR10_Packed == enGvspPixelType or PixelType_Gvsp_BayerRG10_Packed == enGvspPixelType \
            or PixelType_Gvsp_BayerGB10_Packed == enGvspPixelType or PixelType_Gvsp_BayerBG10_Packed == enGvspPixelType \
            or PixelType_Gvsp_BayerGR12_Packed == enGvspPixelType or PixelType_Gvsp_BayerRG12_Packed == enGvspPixelType \
            or PixelType_Gvsp_BayerGB12_Packed == enGvspPixelType or PixelType_Gvsp_BayerBG12_Packed == enGvspPixelType \
            or PixelType_Gvsp_YUV422_Packed == enGvspPixelType or PixelType_Gvsp_YUV422_YUYV_Packed == enGvspPixelType:
        return True
    else:
        return False


def Convert_buffer_to_ndarray(buf, buf_len, width, height, pixel_type):
    """
    将相机缓冲区数据转换为OpenCV可用的numpy数组
    
    功能：
    - 读取原始图像缓冲区
    - 根据像素格式进行相应转换
    - 处理Bayer阵列并转换为BGR颜色空间
    - 返回OpenCV格式的图像
    
    参数:
        buf: 图像缓冲区（ctypes数组）
        buf_len: 缓冲区长度（字节数）
        width: 图像宽度（像素）
        height: 图像高度（像素）
        pixel_type: 像素格式枚举值
    
    返回:
        BGR格式的numpy数组，如果转换失败返回None
    
    支持的格式：
        - 黑白图像：Mono8、Mono10、Mono12等
        - Bayer彩色：BayerRG、BayerGR、BayerGB、BayerBG
        - 其他格式：按灰度处理
    """
    try:
        # 创建numpy数组从缓冲区
        img_data = np.frombuffer(buf, dtype=np.uint8, count=buf_len)

        # 根据像素格式处理
        if Is_mono_data(pixel_type):
            # 黑白图像：直接reshape，然后转为三通道BGR
            img = img_data.reshape((height, width))
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        elif Is_color_data(pixel_type):
            # Bayer彩色格式：转换为BGR颜色空间
            img = img_data.reshape((height, width))
            if pixel_type == PixelType_Gvsp_BayerRG8:
                img = cv2.cvtColor(img, cv2.COLOR_BAYER_RG2BGR)
            elif pixel_type == PixelType_Gvsp_BayerGR8:
                img = cv2.cvtColor(img, cv2.COLOR_BAYER_GR2BGR)
            elif pixel_type == PixelType_Gvsp_BayerGB8:
                img = cv2.cvtColor(img, cv2.COLOR_BAYER_GB2BGR)
            elif pixel_type == PixelType_Gvsp_BayerBG8:
                img = cv2.cvtColor(img, cv2.COLOR_BAYER_BG2BGR)
            else:
                # 其他格式默认作为灰度处理
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            # 未知格式：当作灰度图像处理
            img = img_data.reshape((height, width))
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        return img
    except Exception as e:
        print(f"Convert buffer error: {e}")
        return None


class CameraGrabThread(QThread):
    """
    相机图像采集线程类 - 继承PyQt5的QThread
    
    功能：
    - 在独立线程中持续从相机获取图像
    - 将图像转换为OpenCV格式
    - 通过PyQt信号发送给主线程进行显示
    
    信号：
        frame_ready: 当新帧准备好时发射，携带图像数据、宽、高、帧号
    
    线程安全：
        - 使用Lock保护共享缓冲区
        - 使用Flag标记线程运行状态
    """
    
    # PyQt5信号：发送图像数据 (numpy数组)
    frame_ready = pyqtSignal(np.ndarray, int, int, int)  # 图像、宽、高、帧号

    def __init__(self, camera_op):
        """
        初始化取图线程
        
        参数:
            camera_op: CameraOperation对象引用
        """
        super().__init__()
        self.camera_op = camera_op
        self.running = True  # 线程运行标志

    def run(self):
        """
        线程主循环 - 持续从相机获取并处理图像
        
        流程：
        1. 从相机SDK获取原始图像缓冲区
        2. 复制帧信息和图像数据到本地缓冲区
        3. 将图像缓冲转换为OpenCV格式
        4. 发射PyQt信号通知主线程
        5. 释放相机SDK的缓冲区
        6. 超时无数据时短暂睡眠后重试
        """
        stOutFrame = MV_FRAME_OUT()
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))

        while self.running:
            # 从相机获取图像缓冲，超时设置为1000ms
            ret = self.camera_op.obj_cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
            if 0 == ret:
                # 成功获取图像，获取缓存锁进行线程安全访问
                self.camera_op.buf_lock.acquire()

                # 检查并重新分配缓冲区（如果需要更大的空间）
                if self.camera_op.buf_save_image_len < stOutFrame.stFrameInfo.nFrameLen:
                    if self.camera_op.buf_save_image is not None:
                        del self.camera_op.buf_save_image
                        self.camera_op.buf_save_image = None
                    # 分配足够的缓冲区存储图像数据
                    self.camera_op.buf_save_image = (c_ubyte * stOutFrame.stFrameInfo.nFrameLen)()
                    self.camera_op.buf_save_image_len = stOutFrame.stFrameInfo.nFrameLen

                # 复制帧头信息到本地结构体
                cdll.msvcrt.memcpy(byref(self.camera_op.st_frame_info), byref(stOutFrame.stFrameInfo),
                                   sizeof(MV_FRAME_OUT_INFO_EX))
                # 复制图像数据到本地缓冲区
                cdll.msvcrt.memcpy(byref(self.camera_op.buf_save_image), stOutFrame.pBufAddr,
                                   self.camera_op.st_frame_info.nFrameLen)

                # 将相机缓冲转换为OpenCV可用的numpy数组
                img = Convert_buffer_to_ndarray(
                    self.camera_op.buf_save_image,
                    self.camera_op.st_frame_info.nFrameLen,
                    self.camera_op.st_frame_info.nWidth,
                    self.camera_op.st_frame_info.nHeight,
                    self.camera_op.st_frame_info.enPixelType
                )

                self.camera_op.buf_lock.release()

                # 如果转换成功，发送PyQt信号通知主线程显示图像
                if img is not None:
                    self.frame_ready.emit(
                        img,
                        self.camera_op.st_frame_info.nWidth,
                        self.camera_op.st_frame_info.nHeight,
                        self.camera_op.st_frame_info.nFrameNum
                    )

                print("get one frame: Width[%d], Height[%d], nFrameNum[%d]"
                      % (self.camera_op.st_frame_info.nWidth, self.camera_op.st_frame_info.nHeight,
                         self.camera_op.st_frame_info.nFrameNum))

                # 释放相机SDK的缓冲区
                self.camera_op.obj_cam.MV_CC_FreeImageBuffer(stOutFrame)
            else:
                # 未获取到数据，短暂睡眠后重试
                print("无数据，返回值: " + To_hex_str(ret))
                time.sleep(0.01)

    def stop(self):
        """
        优雅地停止线程
        
        流程：
        1. 设置运行标志为False
        2. 等待线程自然结束
        3. 释放所有资源
        """
        self.running = False
        self.wait()  # 等待线程结束


class CameraOperation:
    """
    相机操作类 - 提供相机控制和图像处理的完整接口
    
    主要功能：
    - 设备连接和初始化
    - 图像采集（连续模式和触发模式）
    - 参数读写（曝光、增益、帧率等）
    - 图像保存（BMP、JPEG格式）
    
    属性：
        obj_cam: 相机对象
        st_device_list: 设备列表
        b_open_device: 设备是否已打开
        b_start_grabbing: 是否正在采集
        buf_lock: 缓冲区线程锁
        st_frame_info: 当前帧信息
    """

    def __init__(self, obj_cam, st_device_list, n_connect_num=0, b_open_device=False, b_start_grabbing=False,
                 h_thread_handle=None,
                 b_thread_closed=False, st_frame_info=None, b_exit=False, b_save_bmp=False, b_save_jpg=False,
                 buf_save_image=None,
                 n_save_image_size=0, n_win_gui_id=0, frame_rate=0, exposure_time=0, gain=0):
        """
        初始化相机操作对象
        
        参数:
            obj_cam: MvCamera对象引用
            st_device_list: 设备列表
            n_connect_num: 要连接的设备索引
            其他参数: 初始化各项状态标志和参数
        """
        self.obj_cam = obj_cam
        self.st_device_list = st_device_list
        self.n_connect_num = n_connect_num
        self.b_open_device = b_open_device
        self.b_start_grabbing = b_start_grabbing
        self.b_thread_closed = b_thread_closed
        self.st_frame_info = MV_FRAME_OUT_INFO_EX()
        self.b_exit = b_exit
        self.b_save_bmp = b_save_bmp
        self.b_save_jpg = b_save_jpg
        self.buf_save_image = buf_save_image
        self.buf_save_image_len = 0
        self.n_save_image_size = n_save_image_size
        self.h_thread_handle = h_thread_handle
        self.b_thread_closed
        self.frame_rate = frame_rate
        self.exposure_time = exposure_time
        self.gain = gain
        self.buf_lock = threading.Lock()  # 取图和存图的缓冲区线程锁

    def Open_device(self):
        """
        打开相机设备并初始化
        
        功能：
        - 根据设备索引创建相机句柄
        - 打开物理设备连接
        - 检测网络最优包大小（GigE相机）
        - 设置初始参数（禁用触发模式等）
        
        返回:
            0: 成功，非0: 失败错误码
        """
        if not self.b_open_device:
            if self.n_connect_num < 0:
                return MV_E_CALLORDER

            # ch:选择设备并创建句柄 | en:Select device and create handle
            nConnectionNum = int(self.n_connect_num)
            stDeviceList = cast(self.st_device_list.pDeviceInfo[int(nConnectionNum)],
                                POINTER(MV_CC_DEVICE_INFO)).contents
            self.obj_cam = MvCamera()
            ret = self.obj_cam.MV_CC_CreateHandle(stDeviceList)
            if ret != 0:
                self.obj_cam.MV_CC_DestroyHandle()
                return ret

            ret = self.obj_cam.MV_CC_OpenDevice()
            if ret != 0:
                return ret
            print("open device successfully!")
            self.b_open_device = True
            self.b_thread_closed = False

            # ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
            if stDeviceList.nTLayerType == MV_GIGE_DEVICE or stDeviceList.nTLayerType == MV_GENTL_GIGE_DEVICE:
                nPacketSize = self.obj_cam.MV_CC_GetOptimalPacketSize()
                if int(nPacketSize) > 0:
                    ret = self.obj_cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
                    if ret != 0:
                        print("warning: set packet size fail! ret[0x%x]" % ret)
                else:
                    print("warning: set packet size fail! ret[0x%x]" % nPacketSize)

            stBool = c_bool(False)
            ret = self.obj_cam.MV_CC_GetBoolValue("AcquisitionFrameRateEnable", stBool)
            if ret != 0:
                print("get acquisition frame rate enable fail! ret[0x%x]" % ret)

            # ch:设置触发模式为off | en:Set trigger mode as off
            ret = self.obj_cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            if ret != 0:
                print("set trigger mode fail! ret[0x%x]" % ret)
            return MV_OK

    def Start_grabbing(self, frame_ready_callback=None):
        """
        开始采集图像
        
        功能：
        - 启动相机采集
        - 创建并启动图像采集线程
        - 连接图像就绪信号到回调函数
        
        参数:
            frame_ready_callback: 图像就绪时的回调函数（可选）
        
        返回:
            0: 成功，非0: 失败错误码
        """
        if not self.b_start_grabbing and self.b_open_device:
            self.b_exit = False
            ret = self.obj_cam.MV_CC_StartGrabbing()
            if ret != 0:
                return ret
            self.b_start_grabbing = True
            print("start grabbing successfully!")

            # 创建并启动 QThread
            self.h_thread_handle = CameraGrabThread(self)
            if frame_ready_callback:
                self.h_thread_handle.frame_ready.connect(frame_ready_callback)
            self.h_thread_handle.start()
            self.b_thread_closed = True
            return MV_OK

        return MV_E_CALLORDER

    def Stop_grabbing(self):
        """
        停止采集图像
        
        功能：
        - 停止采集线程
        - 停止相机采集
        - 清理资源
        
        返回:
            0: 成功，非0: 失败错误码
        """
        if self.b_start_grabbing and self.b_open_device:
            # 停止采集线程
            if self.b_thread_closed and self.h_thread_handle:
                self.h_thread_handle.stop()
                self.b_thread_closed = False
            ret = self.obj_cam.MV_CC_StopGrabbing()
            if ret != 0:
                return ret
            print("stop grabbing successfully!")
            self.b_start_grabbing = False
            self.b_exit = True
            return MV_OK
        else:
            return MV_E_CALLORDER

    def Close_device(self):
        """
        关闭相机设备
        
        功能：
        - 停止采集（如果正在采集）
        - 关闭物理连接
        - 销毁SDK句柄
        - 释放所有资源
        
        返回:
            0: 成功，非0: 失败错误码
        """
        if self.b_open_device:
            # 停止采集线程
            if self.b_thread_closed and self.h_thread_handle:
                self.h_thread_handle.stop()
                self.b_thread_closed = False
            ret = self.obj_cam.MV_CC_CloseDevice()
            if ret != 0:
                return ret

        # 销毁SDK句柄
        self.obj_cam.MV_CC_DestroyHandle()
        self.b_open_device = False
        self.b_start_grabbing = False
        self.b_exit = True
        print("close device successfully!")

        return MV_OK

    def Set_trigger_mode(self, is_trigger_mode):
        """
        设置相机触发模式
        
        功能：
        - 禁用触发模式：相机连续采集图像
        - 启用触发模式：相机等待触发信号，支持软件触发
        
        参数:
            is_trigger_mode: True=触发模式，False=连续模式
        
        返回:
            0: 成功，非0: 失败错误码
        """
        if not self.b_open_device:
            return MV_E_CALLORDER

        if not is_trigger_mode:
            ret = self.obj_cam.MV_CC_SetEnumValue("TriggerMode", 0)
            if ret != 0:
                return ret
        else:
            ret = self.obj_cam.MV_CC_SetEnumValue("TriggerMode", 1)
            if ret != 0:
                return ret
            ret = self.obj_cam.MV_CC_SetEnumValue("TriggerSource", 7)
            if ret != 0:
                return ret

        return MV_OK

    def Trigger_once(self):
        """
        执行一次软件触发
        
        功能：
        - 在触发模式下发送触发命令
        - 相机采集一帧图像
        
        返回:
            0: 成功，非0: 失败错误码
        """
        if self.b_open_device:
            return self.obj_cam.MV_CC_SetCommandValue("TriggerSoftware")

    def Get_parameter(self):
        """
        读取相机工作参数
        
        功能：
        - 从相机读取曝光时间
        - 读取模拟增益值
        - 读取帧率设置
        - 存储到本地属性供UI显示
        
        返回:
            0: 成功，非0: 失败错误码
        """
        if self.b_open_device:
            stFloatParam_FrameRate = MVCC_FLOATVALUE()
            memset(byref(stFloatParam_FrameRate), 0, sizeof(MVCC_FLOATVALUE))
            stFloatParam_exposureTime = MVCC_FLOATVALUE()
            memset(byref(stFloatParam_exposureTime), 0, sizeof(MVCC_FLOATVALUE))
            stFloatParam_gain = MVCC_FLOATVALUE()
            memset(byref(stFloatParam_gain), 0, sizeof(MVCC_FLOATVALUE))
            ret = self.obj_cam.MV_CC_GetFloatValue("AcquisitionFrameRate", stFloatParam_FrameRate)
            if ret != 0:
                return ret
            self.frame_rate = stFloatParam_FrameRate.fCurValue

            ret = self.obj_cam.MV_CC_GetFloatValue("ExposureTime", stFloatParam_exposureTime)
            if ret != 0:
                return ret
            self.exposure_time = stFloatParam_exposureTime.fCurValue

            ret = self.obj_cam.MV_CC_GetFloatValue("Gain", stFloatParam_gain)
            if ret != 0:
                return ret
            self.gain = stFloatParam_gain.fCurValue

            return MV_OK

    def Set_parameter(self, frameRate, exposureTime, gain):
        """
        设置相机工作参数
        
        功能：
        - 设置曝光模式为手动
        - 设置曝光时间（微秒）
        - 设置增益值
        - 设置帧率
        
        参数:
            frameRate: 帧率（fps）
            exposureTime: 曝光时间（μs）
            gain: 增益值（dB）
        
        返回:
            0: 成功，非0: 失败错误码
        """
        if '' == frameRate or '' == exposureTime or '' == gain:
            print('show info', 'please type in the text box !')
            return MV_E_PARAMETER
        if self.b_open_device:
            ret = self.obj_cam.MV_CC_SetEnumValue("ExposureAuto", 0)
            time.sleep(0.2)
            ret = self.obj_cam.MV_CC_SetFloatValue("ExposureTime", float(exposureTime))
            if ret != 0:
                print('show error', 'set exposure time fail! ret = ' + To_hex_str(ret))
                return ret

            ret = self.obj_cam.MV_CC_SetFloatValue("Gain", float(gain))
            if ret != 0:
                print('show error', 'set gain fail! ret = ' + To_hex_str(ret))
                return ret

            ret = self.obj_cam.MV_CC_SetFloatValue("AcquisitionFrameRate", float(frameRate))
            if ret != 0:
                print('show error', 'set acquistion frame rate fail! ret = ' + To_hex_str(ret))
                return ret

            print('show info', 'set parameter success!')

            return MV_OK

    def Save_jpg(self):
        """
        保存当前图像为JPEG文件
        
        功能：
        - 获取缓冲区锁以保证数据一致性
        - 使用SDK接口将图像保存为JPEG格式
        - 文件名为当前帧序号
        - JPEG质量设置为80
        
        返回:
            0: 成功，非0: 失败错误码
        
        说明：
            - 文件将保存到当前工作目录
            - 文件名格式：{帧号}.jpg
        """
        if self.buf_save_image is None:
            return

        # 获取缓冲区锁，保证采集线程不会修改数据
        self.buf_lock.acquire()

        file_path = str(self.st_frame_info.nFrameNum) + ".jpg"
        c_file_path = file_path.encode('ascii')
        stSaveParam = MV_SAVE_IMAGE_TO_FILE_PARAM_EX()
        stSaveParam.enPixelType = self.st_frame_info.enPixelType  # 相机像素格式
        stSaveParam.nWidth = self.st_frame_info.nWidth              # 图像宽度
        stSaveParam.nHeight = self.st_frame_info.nHeight            # 图像高度
        stSaveParam.nDataLen = self.st_frame_info.nFrameLen         # 数据长度
        stSaveParam.pData = cast(self.buf_save_image, POINTER(c_ubyte))
        stSaveParam.enImageType = MV_Image_Jpeg                     # 保存格式：JPEG
        stSaveParam.nQuality = 80                                   # JPEG质量（0-100）
        stSaveParam.pcImagePath = ctypes.create_string_buffer(c_file_path)
        stSaveParam.iMethodValue = 1
        ret = self.obj_cam.MV_CC_SaveImageToFileEx(stSaveParam)

        self.buf_lock.release()
        return ret

    def Save_Bmp(self):
        """
        保存当前图像为BMP文件
        
        功能：
        - 获取缓冲区锁以保证数据一致性
        - 使用SDK接口将图像保存为BMP格式
        - 文件名为当前帧序号
        - BMP为无损格式，保留完整图像质量
        
        返回:
            0: 成功，非0: 失败错误码
        
        说明：
            - 文件将保存到当前工作目录
            - 文件名格式：{帧号}.bmp
            - BMP文件通常较大（无压缩）
        """
        if 0 == self.buf_save_image:
            return

        # 获取缓冲区锁，保证采集线程不会修改数据
        self.buf_lock.acquire()

        file_path = str(self.st_frame_info.nFrameNum) + ".bmp"
        c_file_path = file_path.encode('ascii')

        stSaveParam = MV_SAVE_IMAGE_TO_FILE_PARAM_EX()
        stSaveParam.enPixelType = self.st_frame_info.enPixelType  # 相机像素格式
        stSaveParam.nWidth = self.st_frame_info.nWidth              # 图像宽度
        stSaveParam.nHeight = self.st_frame_info.nHeight            # 图像高度
        stSaveParam.nDataLen = self.st_frame_info.nFrameLen         # 数据长度
        stSaveParam.pData = cast(self.buf_save_image, POINTER(c_ubyte))
        stSaveParam.enImageType = MV_Image_Bmp                      # 保存格式：BMP
        stSaveParam.pcImagePath = ctypes.create_string_buffer(c_file_path)
        stSaveParam.iMethodValue = 1
        ret = self.obj_cam.MV_CC_SaveImageToFileEx(stSaveParam)

        self.buf_lock.release()

        return ret
