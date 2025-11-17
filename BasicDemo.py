# -*- coding: utf-8 -*-
"""
海康相机基础演示程序 - 主应用程序
用途：通过PyQt5 GUI界面控制相机的各项参数，实现实时图像采集和显示
主要功能：设备枚举、设备打开、图像采集、参数设置、图像保存等
"""

import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt
import cv2
import numpy as np
from CamOperation_class import CameraOperation
from hikvisionSDK.MvCameraControl_class import *
from hikvisionSDK.MvErrorDefine_const import *
from hikvisionSDK.CameraParams_header import *
from PyUICBasicDemo import Ui_MainWindow
import ctypes


def TxtWrapBy(start_str, end, all):
    """
    从字符串中提取特定位置的内容
    
    参数:
        start_str: 起始标识符（如"["）
        end: 结束标识符（如"]"）
        all: 完整字符串
    
    返回:
        提取的子字符串（已去除首尾空格），如果未找到则返回None
    
    示例:
        TxtWrapBy("[", "]", "[0]GigE: Camera1") -> "0"
    """
    start = all.find(start_str)
    if start >= 0:
        start += len(start_str)
        end = all.find(end, start)
        if end >= 0:
            return all[start:end].strip()


def ToHexStr(num):
    """
    将整数转换为十六进制字符串显示
    
    参数:
        num: 要转换的整数（可以是负数）
    
    返回:
        对应的十六进制字符串表示（小写字母）
    
    示例:
        ToHexStr(255) -> "ff"
        ToHexStr(-1) -> "ffffffff"
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


if __name__ == "__main__":
    """
    主程序入口 - 初始化SDK和应用程序
    """
    
    # 初始化SDK
    MvCamera.MV_CC_Initialize()

    # ===== 全局变量定义 =====
    global deviceList           # 设备列表信息
    deviceList = MV_CC_DEVICE_INFO_LIST()
    global cam                  # 相机对象
    cam = MvCamera()
    global nSelCamIndex         # 选中的相机设备索引
    nSelCamIndex = 0
    global obj_cam_operation    # 相机操作对象
    obj_cam_operation = 0
    global isOpen               # 相机是否已打开
    isOpen = False
    global isGrabbing           # 是否正在采集图像
    isGrabbing = False
    global isCalibMode          # 是否为标定模式（获取原始图像）
    isCalibMode = True

    # ===== 辅助回调函数 =====
    def xFunc(event):
        """
        设备下拉列表选择事件回调函数
        
        功能：当用户在下拉列表中选择不同的相机设备时，更新选中设备的索引
        """
        global nSelCamIndex
        nSelCamIndex = TxtWrapBy("[", "]", ui.ComboDevices.get())


    def decoding_char(c_ubyte_value):
        """
        解码字符编码工具函数
        
        功能：将C语言传来的字节数据转换为Python字符串，处理中文编码
        参数:
            c_ubyte_value: C语言的ubyte数据
        返回:
            解码后的字符串（GBK或其他格式）
        """
        c_char_p_value = ctypes.cast(c_ubyte_value, ctypes.c_char_p)
        try:
            decode_str = c_char_p_value.value.decode('gbk')  # 处理中文字符
        except UnicodeDecodeError:
            decode_str = str(c_char_p_value.value)
        return decode_str

    def enum_devices():
        """
        枚举所有可用的相机设备
        
        功能：
        - 扫描系统中连接的所有相机设备（GigE、USB、CameraLink等）
        - 获取每个设备的详细信息（型号、IP地址、序列号等）
        - 将设备信息显示在下拉列表中供用户选择
        
        返回:
            0: 成功，非0: 失败
        """
        global deviceList
        global obj_cam_operation

        deviceList = MV_CC_DEVICE_INFO_LIST()
        n_layer_type = (MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE
                        | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE)
        ret = MvCamera.MV_CC_EnumDevices(n_layer_type, deviceList)
        if ret != 0:
            strError = "Enum devices fail! ret = :" + ToHexStr(ret)
            QMessageBox.warning(mainWindow, "Error", strError, QMessageBox.Ok)
            return ret

        if deviceList.nDeviceNum == 0:
            QMessageBox.warning(mainWindow, "Info", "Find no device", QMessageBox.Ok)
            return ret
        print("Find %d devices!" % deviceList.nDeviceNum)

        devList = []
        for i in range(0, deviceList.nDeviceNum):
            mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE or mvcc_dev_info.nTLayerType == MV_GENTL_GIGE_DEVICE:
                print("\ngige device: [%d]" % i)
                user_defined_name = decoding_char(mvcc_dev_info.SpecialInfo.stGigEInfo.chUserDefinedName)
                model_name = decoding_char(mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName)
                print("device user define name: " + user_defined_name)
                print("device model name: " + model_name)

                nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
                nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
                nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
                nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
                print("current ip: %d.%d.%d.%d " % (nip1, nip2, nip3, nip4))
                devList.append(
                    "[" + str(i) + "]GigE: " + user_defined_name + " " + model_name + "(" + str(nip1) + "." + str(
                        nip2) + "." + str(nip3) + "." + str(nip4) + ")")
            elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
                print("\nu3v device: [%d]" % i)
                user_defined_name = decoding_char(mvcc_dev_info.SpecialInfo.stUsb3VInfo.chUserDefinedName)
                model_name = decoding_char(mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName)
                print("device user define name: " + user_defined_name)
                print("device model name: " + model_name)

                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                print("user serial number: " + strSerialNumber)
                devList.append("[" + str(i) + "]USB: " + user_defined_name + " " + model_name
                               + "(" + str(strSerialNumber) + ")")
            elif mvcc_dev_info.nTLayerType == MV_GENTL_CAMERALINK_DEVICE:
                print("\nCML device: [%d]" % i)
                user_defined_name = decoding_char(mvcc_dev_info.SpecialInfo.stCMLInfo.chUserDefinedName)
                model_name = decoding_char(mvcc_dev_info.SpecialInfo.stCMLInfo.chModelName)
                print("device user define name: " + user_defined_name)
                print("device model name: " + model_name)

                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stCMLInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                print("user serial number: " + strSerialNumber)
                devList.append("[" + str(i) + "]CML: " + user_defined_name + " " + model_name
                               + "(" + str(strSerialNumber) + ")")
            elif mvcc_dev_info.nTLayerType == MV_GENTL_CXP_DEVICE:
                print("\nCXP device: [%d]" % i)
                user_defined_name = decoding_char(mvcc_dev_info.SpecialInfo.stCXPInfo.chUserDefinedName)
                model_name = decoding_char(mvcc_dev_info.SpecialInfo.stCXPInfo.chModelName)
                print("device user define name: " + user_defined_name)
                print("device model name: " + model_name)

                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stCXPInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                print("user serial number: " + strSerialNumber)
                devList.append("[" + str(i) + "]CXP: " + user_defined_name + " " + model_name
                               + "(" + str(strSerialNumber) + ")")
            elif mvcc_dev_info.nTLayerType == MV_GENTL_XOF_DEVICE:
                print("\nXoF device: [%d]" % i)
                user_defined_name = decoding_char(mvcc_dev_info.SpecialInfo.stXoFInfo.chUserDefinedName)
                model_name = decoding_char(mvcc_dev_info.SpecialInfo.stXoFInfo.chModelName)
                print("device user define name: " + user_defined_name)
                print("device model name: " + model_name)

                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stXoFInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                print("user serial number: " + strSerialNumber)
                devList.append("[" + str(i) + "]XoF: " + user_defined_name + " " + model_name
                               + "(" + str(strSerialNumber) + ")")

        ui.ComboDevices.clear()
        ui.ComboDevices.addItems(devList)
        ui.ComboDevices.setCurrentIndex(0)


    def open_device():
        """
        打开选中的相机设备
        
        功能：
        - 验证是否已有相机在运行
        - 创建相机对象并建立与设备的连接
        - 初始化相机为连续模式（非触发模式）
        - 获取并显示相机的当前参数
        - 设置UI控件状态
        """
        global deviceList
        global nSelCamIndex
        global obj_cam_operation
        global isOpen
        if isOpen:
            QMessageBox.warning(mainWindow, "Error", 'Camera is Running!', QMessageBox.Ok)
            return MV_E_CALLORDER

        nSelCamIndex = ui.ComboDevices.currentIndex()
        if nSelCamIndex < 0:
            QMessageBox.warning(mainWindow, "Error", 'Please select a camera!', QMessageBox.Ok)
            return MV_E_CALLORDER

        obj_cam_operation = CameraOperation(cam, deviceList, nSelCamIndex)
        ret = obj_cam_operation.Open_device()
        if 0 != ret:
            strError = "Open device failed ret:" + ToHexStr(ret)
            QMessageBox.warning(mainWindow, "Error", strError, QMessageBox.Ok)
            isOpen = False
        else:
            set_continue_mode()

            get_param()

            isOpen = True
            enable_controls()


    def display_frame(img, width, height, frame_num):
        """
        在PyQt窗口中显示实时图像
        
        功能：
        - 接收OpenCV格式的图像数据
        - 转换为RGB格式（PyQt5需要）
        - 缩放图像适应显示窗口
        - 通过QLabel控件显示
        
        参数:
            img: numpy数组格式的图像（BGR格式）
            width: 图像宽度
            height: 图像高度
            frame_num: 帧序号
        """
        if img is None:
            return

        try:
            # 确保是BGR格式
            if len(img.shape) == 3:
                # 转换为RGB格式（PyQt5需要RGB）
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:
                img_rgb = img

            # 转换为PyQt5的QImage格式
            h, w = img_rgb.shape[:2]
            bytes_per_line = 3 * w if len(img_rgb.shape) == 3 else w
            q_img = QImage(img_rgb.data, w, h, bytes_per_line,
                           QImage.Format_RGB888 if len(img_rgb.shape) == 3 else QImage.Format_Grayscale8)

            # 缩放到显示窗口大小
            pixmap = QPixmap.fromImage(q_img)
            scaled_pixmap = pixmap.scaledToWidth(ui.widgetDisplay.width() - 4, Qt.SmoothTransformation)

            # 在QLabel上显示图像
            ui.labelDisplay.setPixmap(scaled_pixmap)
            ui.labelDisplay.adjustSize()
        except Exception as e:
            print(f"图像显示错误: {e}")


    def start_grabbing():
        """
        开始采集图像
        
        功能：
        - 启动取图线程
        - 连接图像显示回调函数
        - 更新UI控件状态
        """
        global obj_cam_operation
        global isGrabbing

        # 连接图像显示回调函数
        ret = obj_cam_operation.Start_grabbing(display_frame)
        if ret != 0:
            strError = "Start grabbing failed ret:" + ToHexStr(ret)
            QMessageBox.warning(mainWindow, "Error", strError, QMessageBox.Ok)
        else:
            isGrabbing = True
            enable_controls()

    def stop_grabbing():
        """
        停止采集图像
        
        功能：
        - 停止取图线程
        - 停止与相机的数据传输
        - 更新UI控件状态
        """
        global obj_cam_operation
        global isGrabbing
        ret = obj_cam_operation.Stop_grabbing()
        if ret != 0:
            strError = "Stop grabbing failed ret:" + ToHexStr(ret)
            QMessageBox.warning(mainWindow, "Error", strError, QMessageBox.Ok)
        else:
            isGrabbing = False
            enable_controls()

    def close_device():
        """
        关闭相机设备
        
        功能：
        - 停止采集（如果正在采集）
        - 关闭设备连接
        - 释放资源
        - 更新UI控件状态
        """
        global isOpen
        global isGrabbing
        global obj_cam_operation

        if isOpen:
            obj_cam_operation.Close_device()
            isOpen = False

        isGrabbing = False

        enable_controls()


    def set_continue_mode():
        """
        设置相机为连续采集模式
        
        功能：
        - 关闭触发模式
        - 设置相机连续自动采集图像
        - 禁用软触发按钮
        """
        ret = obj_cam_operation.Set_trigger_mode(False)
        if ret != 0:
            strError = "Set continue mode failed ret:" + ToHexStr(ret) + " mode is " + str(is_trigger_mode)
            QMessageBox.warning(mainWindow, "Error", strError, QMessageBox.Ok)
        else:
            ui.radioContinueMode.setChecked(True)
            ui.radioTriggerMode.setChecked(False)
            ui.bnSoftwareTrigger.setEnabled(False)

    def set_software_trigger_mode():
        """
        设置相机为软件触发模式
        
        功能：
        - 启用触发模式
        - 设置触发源为软件触发
        - 启用软触发按钮（仅在采集时）
        """
        ret = obj_cam_operation.Set_trigger_mode(True)
        if ret != 0:
            strError = "Set trigger mode failed ret:" + ToHexStr(ret)
            QMessageBox.warning(mainWindow, "Error", strError, QMessageBox.Ok)
        else:
            ui.radioContinueMode.setChecked(False)
            ui.radioTriggerMode.setChecked(True)
            ui.bnSoftwareTrigger.setEnabled(isGrabbing)

    def trigger_once():
        """
        执行一次软件触发
        
        功能：
        - 在触发模式下，发送触发命令给相机
        - 相机采集一帧图像后停止
        """
        ret = obj_cam_operation.Trigger_once()
        if ret != 0:
            strError = "TriggerSoftware failed ret:" + ToHexStr(ret)
            QMessageBox.warning(mainWindow, "Error", strError, QMessageBox.Ok)

    def save_bmp():
        """
        保存当前采集的图像为BMP文件
        
        功能：
        - 获取当前显示的图像数据
        - 以BMP格式保存到本地文件
        - 文件名为当前帧序号
        """
        ret = obj_cam_operation.Save_Bmp()
        if ret != MV_OK:
            strError = "Save BMP failed ret:" + ToHexStr(ret)
            QMessageBox.warning(mainWindow, "Error", strError, QMessageBox.Ok)
        else:
            print("Save image success")


    def is_float(str):
        """
        检查字符串是否为有效的浮点数
        
        参数:
            str: 待检查的字符串
        返回:
            True: 是有效浮点数，False: 否
        """
        try:
            float(str)
            return True
        except ValueError:
            return False

    def get_param():
        """
        读取相机当前参数
        
        功能：
        - 从相机读取曝光时间
        - 读取增益值
        - 读取帧率
        - 在UI界面显示这些参数
        """
        ret = obj_cam_operation.Get_parameter()
        if ret != MV_OK:
            strError = "Get param failed ret:" + ToHexStr(ret)
            QMessageBox.warning(mainWindow, "Error", strError, QMessageBox.Ok)
        else:
            # 以两位小数显示参数
            ui.edtExposureTime.setText("{0:.2f}".format(obj_cam_operation.exposure_time))
            ui.edtGain.setText("{0:.2f}".format(obj_cam_operation.gain))
            ui.edtFrameRate.setText("{0:.2f}".format(obj_cam_operation.frame_rate))

    def set_param():
        """
        设置相机工作参数
        
        功能：
        - 验证用户输入的参数格式
        - 将新参数写入相机
        - 包括曝光时间、增益、帧率
        
        返回:
            0: 成功，非0: 失败
        """
        frame_rate = ui.edtFrameRate.text()
        exposure = ui.edtExposureTime.text()
        gain = ui.edtGain.text()

        # 检查输入参数的有效性
        if is_float(frame_rate) != True or is_float(exposure) != True or is_float(gain) != True:
            strError = "Set param failed ret:" + ToHexStr(MV_E_PARAMETER)
            QMessageBox.warning(mainWindow, "Error", strError, QMessageBox.Ok)
            return MV_E_PARAMETER

        ret = obj_cam_operation.Set_parameter(frame_rate, exposure, gain)
        if ret != MV_OK:
            strError = "Set param failed ret:" + ToHexStr(ret)
            QMessageBox.warning(mainWindow, "Error", strError, QMessageBox.Ok)

        return MV_OK

    def enable_controls():
        """
        根据设备状态动态启用/禁用UI控件
        
        功能：
        - 根据相机是否打开启用/禁用相关控件
        - 根据是否正在采集启用/禁用采集控件
        - 管理模式选择和触发按钮的状态
        
        控件状态规则：
        - 打开按钮：仅在相机关闭时启用
        - 采集控件：仅在相机打开时启用
        - 软触发按钮：仅在触发模式且采集中时启用
        """
        global isGrabbing
        global isOpen

        # 先设置group的状态，再单独设置各控件状态
        ui.groupGrab.setEnabled(isOpen)
        ui.groupParam.setEnabled(isOpen)

        ui.bnOpen.setEnabled(not isOpen)
        ui.bnClose.setEnabled(isOpen)

        ui.bnStart.setEnabled(isOpen and (not isGrabbing))
        ui.bnStop.setEnabled(isOpen and isGrabbing)
        ui.bnSoftwareTrigger.setEnabled(isGrabbing and ui.radioTriggerMode.isChecked())

        ui.bnSaveImage.setEnabled(isOpen and isGrabbing)


    # ===== 初始化PyQt应用 =====
    """
    创建应用程序实例，加载UI界面，绑定事件处理函数
    """
    app = QApplication(sys.argv)
    mainWindow = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(mainWindow)
    
    # ===== 绑定设备管理按钮 =====
    ui.bnEnum.clicked.connect(enum_devices)           # 枚举设备
    ui.bnOpen.clicked.connect(open_device)            # 打开设备
    ui.bnClose.clicked.connect(close_device)          # 关闭设备
    
    # ===== 绑定采集控制按钮 =====
    ui.bnStart.clicked.connect(start_grabbing)        # 开始采集
    ui.bnStop.clicked.connect(stop_grabbing)          # 停止采集
    
    # ===== 绑定触发模式控制 =====
    ui.bnSoftwareTrigger.clicked.connect(trigger_once)        # 软触发按钮
    ui.radioTriggerMode.clicked.connect(set_software_trigger_mode)  # 触发模式
    ui.radioContinueMode.clicked.connect(set_continue_mode)         # 连续模式
    
    # ===== 绑定参数控制按钮 =====
    ui.bnGetParam.clicked.connect(get_param)          # 读取参数
    ui.bnSetParam.clicked.connect(set_param)          # 设置参数
    
    # ===== 绑定图像保存按钮 =====
    ui.bnSaveImage.clicked.connect(save_bmp)          # 保存图像

    # ===== 显示窗口并运行主事件循环 =====
    mainWindow.show()
    app.exec_()

    # ===== 程序退出时清理资源 =====
    close_device()                              # 关闭相机
    MvCamera.MV_CC_Finalize()                  # 反初始化SDK
    sys.exit()                                  # 退出程序
