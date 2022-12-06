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
import yaml
this = sys.modules[__name__]

##############################################################################################
# 公共参数配置文件
##############################################################################################
this.config_path="/home/zonesion/catkin_ws/src/marm_controller/config/config.yaml"
with open(this.config_path, "r") as f:
    config = yaml.load(f.read())

##############################################################################################
# 应用配置参数
##############################################################################################
g_open=config["g_open"]                 #机械臂夹具打开角度
this.arm_mode="varm"                    #


from arm import Arm
class AiArm(Arm):
    def __init__(self,g_open):
        super(AiArm,self).__init__(g_open,xarm="xarm")          #定义为在arm（3399）端运行此程序。初始化Arm类,定义为"varm"是在虚拟机远程控制


def VnodeData_Subscriber():
    rospy.Subscriber('/arm_controller/follow_joint_trajectory/cancel',GoalID,cmdfu)    #订阅控制板命令,实现机械臂紧急停止
    rospy.spin()

if __name__ == '__main__':
    def quit(signum, frame):
        print('EXIT APP') 
        sys.exit()

    signal.signal(signal.SIGINT, quit)                          
    signal.signal(signal.SIGTERM, quit)
    rospy.init_node("AIARM_NODE", log_level=rospy.INFO)         #初始化节点

    vnodeData_subscriber = threading.Thread(target=VnodeData_Subscriber)
    vnodeData_subscriber.setDaemon(True)
    vnodeData_subscriber.start()


