#!/usr/bin/python
# -*- coding: utf-8 -*-

##############################################################################################
# 文件：main.py
# 作者：Zonesion wanghao 20220412
# 说明：aiarm 主应用程序
# 修改：20221206   初始版本  
#       
# 注释：主要实现与vnode进行通讯
##############################################################################################
import rospy
import time
import copy 
import sys   
import signal
import cvwin
import threading
import math
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float32MultiArray,Int16MultiArray,Int32,Int32MultiArray
from aiarm.srv import *
import yaml
this = sys.modules[__name__]

##############################################################################################
# 公共参数配置文件
##############################################################################################
this.config_path="/home/zonesion/catkin_ws/src/marm_controller/config/config.yaml"
with open(this.config_path, "r") as f:
    if sys.version_info < (3, 0):
        config = yaml.load(f.read())
    else:    
        config = yaml.load(f.read(), Loader=yaml.FullLoader)

##############################################################################################
# 应用配置参数
##############################################################################################
this.g_open=config["g_open"]                    # 机械臂夹具打开角度
this.gripper_ty= True                           # 夹具极性
this.g_range=[-130,130]                         # 底层夹具范围

from arm import Arm
class AiArm(Arm):
    def __init__(self,g_open):
        super(AiArm,self).__init__(g_open,gripper_ty=this.gripper_ty,arm_debug=False)          #定义为在arm（3399）端运行此程序。初始化Arm类,定义为"varm"是在虚拟机远程控制
        self.joint_posture_pub=rospy.Publisher('/aiarm/arm_joint', Float32MultiArray, queue_size=0, latch=True)
        self.space_posture_pub=rospy.Publisher('/aiarm/arm_space', Float32MultiArray, queue_size=0, latch=True)
        vnodeData_service = threading.Thread(target=self.VnodeData_to_app)
        vnodeData_service.setDaemon(True)
        vnodeData_service.start()
        vnodeData_publish = threading.Thread(target=self.app_to_VnodeData)
        vnodeData_publish.setDaemon(True)
        vnodeData_publish.start()

    def joint_target_handle(self,data):
        its = data.joint.split("/")
        its_rad =[math.radians(float(i)) for i in its]                  #将角度转换为弧度
        result=self.set_joint_value_target(its_rad)
        if result:
            return xarm_jointResponse(xarm_jointResponse.SUCCESS)       # 执行完成
        else:
            return xarm_jointResponse(xarm_jointResponse.ERROR)         # 执行失败

    def space_target_handle(self,data):
        its = data.space.split("/")
        its_rad =[float(i) for i in its]                       
        result=self.goPose_rpy(its_rad)
        if result:
            return xarm_spaceResponse(xarm_spaceResponse.SUCCESS)       # 执行完成
        else:
            return xarm_spaceResponse(xarm_spaceResponse.ERROR)         # 执行失败

    def fixture_stroke_handle(self,data):
        value = data.data  
        if value<this.g_range[0] or value>this.g_range[1]:
          return xarm_fix_strokeResponse(xarm_fix_strokeResponse.ERROR)         # 执行失败
        result=self.setGripperJoint(value)
        if result:
            return xarm_fix_strokeResponse(xarm_fix_strokeResponse.SUCCESS)       # 执行完成
        else:
            return xarm_fix_strokeResponse(xarm_fix_strokeResponse.ERROR)         # 执行失败

    def fixture_switch_handle(self,data):
        value = data.data  
        result=self.setGripper(value)
        if result:
            return xarm_fix_switchResponse(xarm_fix_switchResponse.SUCCESS)       # 执行完成
        else:
            return xarm_fix_switchResponse(xarm_fix_switchResponse.ERROR)         # 执行失败

    def preset_positions_handle(self,data):
        value = data.data  
        if value:
          result=self.arm_goStart()
        else:
          result=self.arm_goHome()
        if result:
            return xarm_pre_posResponse(xarm_pre_posResponse.SUCCESS)       # 执行完成
        else:
            return xarm_pre_posResponse(xarm_pre_posResponse.ERROR)         # 执行失败

    def VnodeData_to_app(self):
        rospy.Service('/vnode_xarm/joint_target', xarm_joint, self.joint_target_handle)    #V1
        rospy.Service('/vnode_xarm/space_target', xarm_space, self.space_target_handle)    #V2
        rospy.Service('/vnode_xarm/fixture_stroke', xarm_fix_stroke, self.fixture_stroke_handle)    #V3
        rospy.Service('/vnode_xarm/preset_positions', xarm_pre_pos, self.preset_positions_handle)    #D1.0 预置点
        rospy.spin()
    
    def app_to_VnodeData(self):
        while True:
            data=Float32MultiArray()
            # 关节
            joint=self.get_joints()
            # data.data=joint
            # 取6位小数
            data.data=[180/math.pi*i for i in joint]    #弧度转换角度
            self.joint_posture_pub.publish(data) 
            # 空间
            pose=self.getPose()
            rpy=self.getRpy()
            poserpy=[pose[0],pose[1],pose[2],rpy[0],rpy[1],rpy[2]]
            data.data=poserpy
            # 取6位小数
            # data.data=[float('%.6f'%i) for i in poserpy] 
            self.space_posture_pub.publish(data)
            time.sleep(1)

if __name__ == '__main__':
    def quit(signum, frame):
        print('EXIT APP') 
        sys.exit()

    signal.signal(signal.SIGINT, quit)                          
    signal.signal(signal.SIGTERM, quit)
    rospy.init_node("AIARM_NODE", log_level=rospy.INFO)         #初始化节点
    aiarm=AiArm(this.g_open)
    while True:
      # aiarm.setGripper(True)
      # time.sleep(1)
      # aiarm.setGripper(False)
      # time.sleep(1)
      time.sleep(5)
      print('run')


