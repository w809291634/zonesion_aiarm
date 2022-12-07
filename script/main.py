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
from std_msgs.msg import String
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
this.arm_mode="xarm"                            # xarm:真实机械臂   varm:虚拟机械臂

from arm import Arm
class AiArm(Arm):
    def __init__(self,g_open):
        super(AiArm,self).__init__(g_open,xarm=this.arm_mode,arm_debug=False)          #定义为在arm（3399）端运行此程序。初始化Arm类,定义为"varm"是在虚拟机远程控制
        vnodeData_subscriber = threading.Thread(target=self.VnodeData_to_app)
        vnodeData_subscriber.setDaemon(True)
        vnodeData_subscriber.start()

    def joint_target_handle(self,data):
        its = data.joint.split("/")
        its_rad =[math.radians(float(i)) for i in its]                  #将角度转换为弧度
        result=self.set_joint_value_target(its_rad)
        # print("arm joint result:",result)
        if result:
            return xarm_jointResponse(xarm_jointResponse.SUCCESS)       # 执行完成
        else:
            return xarm_jointResponse(xarm_jointResponse.ERROR)         # 执行失败

    def VnodeData_to_app(self):
        service_gripper = rospy.Service('/vnode_xarm/joint_target', xarm_joint, self.joint_target_handle)    # 建立服务 等待客户端进行连接

        rospy.spin()

if __name__ == '__main__':
    def quit(signum, frame):
        print('EXIT APP') 
        sys.exit()

    signal.signal(signal.SIGINT, quit)                          
    signal.signal(signal.SIGTERM, quit)
    rospy.init_node("AIARM_NODE", log_level=rospy.INFO)         #初始化节点
    aiarm=AiArm(this.g_open)
    while True:
        time.sleep(5)


