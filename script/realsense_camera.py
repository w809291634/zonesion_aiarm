#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import sys
import cvwin
import rospy
import cv2
import threading
import time
import numpy as np
import copy
from collections import OrderedDict
from aiarm.srv import *
import pyrealsense2 as rs
from obj_detection_pb import detection      #插入检测模块
from std_msgs.msg import String
import yaml
this = sys.modules[__name__]
this.config_path="/home/zonesion/catkin_ws/src/marm_controller/config/config.yaml"

this.dir_f = os.path.abspath(os.path.dirname(__file__))

with open(this.config_path, "r") as f:
  if sys.version_info < (3, 0):
    config = yaml.load(f.read())
  else:    
    config = yaml.load(f.read(), Loader=yaml.FullLoader)

def max_word(lt):
  # 定义一个字典，用于保存每个元素及出现的次数
  d = {}
  # 记录做大的元素(字典的键)
  max_key = None
  for w in lt:
    if w not in d:
      # 统计该元素在列表中出现的次数
      count = lt.count(w)
      # 以元素作为键，次数作为值，保存到字典中
      d[w] = count
      # 记录最大元素
      if d.get(max_key, 0) < count:
        max_key = w
  return max_key,d

def stability_check(lt,tolerance=0.003):        
    '''
    lt 列表
    tolerance 公差 单位m,初始值3mm
    '''
    lt.sort()
    max=lt[len(lt)-1]
    min=lt[0]
    if abs(max-min)>tolerance :
      print(abs(max-min)) 
      return -1
    else:
      return 1

class RealsenseCamera(object):
    def __init__(self, findObjCall, MARGIN_PIX=7):
      self.findObj = findObjCall
      self.MARGIN_PIX = MARGIN_PIX
      self.window_name='camera'
      self.open_wins=[]
      self.__camera_enable=False
      self.camera_init(424,240)
      try:
          rospy.wait_for_service('/vnode_xarm/visual_grasping',timeout=5)
      except (rospy.ServiceException, rospy.ROSException ,Exception) as e:
          print(e)
          sys.exit()
      self.visual_grasping  = rospy.ServiceProxy('/vnode_xarm/visual_grasping', xarm_vis_grasp)

    def camera_init(self,WIDTH,HEIGHT):
      self.pipeline = rs.pipeline()
      config = rs.config()
      config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 30)     #使能深度相机
      config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30)    #使能彩色相机
      try:
        # if self.__camera_enable!=True:
          self.profile = self.pipeline.start(config)
          # self.__camera_enable=True

          # 保存相机内参
          frames = self.pipeline.wait_for_frames()            #等待相机坐标系生成
          color_frame = frames.get_color_frame()         #获取彩色相机坐标系
          self.intr = color_frame.profile.as_video_stream_profile().intrinsics
          camera_parameters = {'fx': self.intr.fx, 'fy': self.intr.fy,
                              'ppx': self.intr.ppx, 'ppy': self.intr.ppy,
                              'height': self.intr.height, 'width': self.intr.width,
                              'depth_scale': self.profile.get_device().first_depth_sensor().get_depth_scale()
                              }
          # rospy.logwarn(camera_parameters)
          # 保存深度参数
          align_to = rs.stream.color                              #统一对齐到彩色相机
          align = rs.align(align_to)
          aligned_frames = align.process(frames)                  #对齐后的相机坐标系
          aligned_depth_frame = aligned_frames.get_depth_frame()  #对齐到彩色相机后的深度相机坐标系
          self.depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
      except RuntimeError as e:
        # if "UVC device is already opened!" in e.args[0]:
        #   self.stopCamera()
        print(e)
        sys.exit()
      # rospy.logwarn(self.depth_intrin)

    def camera_data(self):
      # 图像对齐
      frames = self.pipeline.wait_for_frames()            #等待相机坐标系生成

      self.align_to = rs.stream.color                     #统一对齐到彩色相机
      self.align = rs.align(self.align_to)
      self.aligned_frames = self.align.process(frames)    #对齐后的相机坐标系

      self.aligned_depth_frame = self.aligned_frames.get_depth_frame()        #对齐到彩色相机后的深度相机坐标系
      self.color_frame = self.aligned_frames.get_color_frame()                #彩色相机坐标系

      if self.aligned_depth_frame and self.color_frame:
          self.color_data = np.asanyarray(self.color_frame.get_data())        #/camera/color/image_raw
          self.dep_data= np.asanyarray(self.aligned_depth_frame.get_data())   #/camera/aligned_depth_to_color/image_raw

    def Depth_data(self,pos,hp,wp,minDeep=135,maxDeep=1000,range_key=2,grade=0.9):  #获取位置坐标系下的深度数据
        '''
        pos[0]  x坐标，单位pixels
        pos[1]  y坐标，单位pixels
        hp      纵向公差
        wp      横向公差
        minDeep 最小深度
        '''
        depimg = self.dep_data
        xx= pos[0]      #x坐标转存，单位pixels
        yy= pos[1]      #y坐标转存，单位pixels
        sumx = sumy = sumz = num =0
        list_deep=[]
        # dis = aligned_depth_frame.get_distance(x, y)      # 获取深度的接口
        for m in range(int(yy-hp), int(yy+hp)):             # 以yy中心，hp为公差的范围数组
            for n in range(int(xx-wp), int(xx+wp)):
                if depimg[m][n] < minDeep:
                    continue
                if depimg[m][n] > maxDeep:
                    continue
                list_deep.append(depimg[m][n])
        if(list_deep!=[]):
            max_length=(2*hp)*(2*wp)
            length=len(list_deep)               #获取深度数据的长度，长度不一定
            max_key,d=max_word(list_deep)
            # print 'max_key,d:',max_key,d
            m=0
            for i in range(int(max_key)-range_key, int(max_key)+range_key+1):
                # print 'i:',i
                if d.get(i)!=None:
                    m+=d.get(i)
            point=float(m)/length               #深度列表数据最多的总长度比值
            point1=float(length)/max_length     #深度数据的有效长度
            # print 'point:',point,'point1:',point1
            if point>grade and point1>grade:
                return int(max_key)
            else :
                return -1 #深度数据不对  
        else :
            return -1 #深度数据不对

    def camera_coord_pos_api(self, x, y, deep):
        '''
        x   物体某点的x坐标 pixel 
        y   物体某点的y坐标 pixel
        deep    该点对应的深度数据 单位mm
        转换相机坐标系
        转换方法2
        [22.698266983032227, 29.696226119995117, 178.0]
        '''
        camera_coordinate = rs.rs2_deproject_pixel_to_point(intrin=self.depth_intrin, pixel=[x, y], depth=deep) #单位mm
        camera_factor = 1000.0 
        Zc= camera_coordinate[2]/ camera_factor     
        Xc= camera_coordinate[0]/ camera_factor 
        Yc= camera_coordinate[1]/ camera_factor 
        return (Xc,Yc,Zc)                       #返回相机坐标系下的坐标点

    def __win_is_open(self,name):
        for i in self.open_wins:
            if i==name:
                return True
            else:
                return False
        return False

    def __open_win(self,img):
        if self.__win_is_open(self.window_name)==False:    
            self.open_wins.append(self.window_name)
        cvwin.imshow(self.window_name,img)

    def __close_win(self):
        if self.__win_is_open(self.window_name)==True:
            cvwin.destroyWindow(self.window_name)
            self.open_wins.remove(self.window_name)

    def object_detect(self):
        _, box, type = self.findObj(self.color_data)   #_:处理后的图像，带有红色方框.与cv_image同一内存    #box:  返回左上角的坐标,和物体的宽度\高度，红色方框 #type:标签
        if len(box)>0 :
            rect = box                          #取box数据，返回左上角的坐标,和物体的宽度\高度，
            if rect[2]>self.MARGIN_PIX*2 and rect[3] > self.MARGIN_PIX*2:   #宽度大于两倍的边缘和长度大于两倍边缘
                rect2 = (rect[0]+self.MARGIN_PIX, rect[1]+self.MARGIN_PIX, rect[2]-self.MARGIN_PIX*2, rect[3]-self.MARGIN_PIX*2)    #rect2为处理后的方框坐标，绿色方框            
                cv2.rectangle(self.color_data, (rect2[0], rect2[1]),(rect2[0] + rect2[2], rect2[1] + rect2[3]), (0, 255, 0), 2)  #显示绿色方框
            else:
                rect2 = np.array([])        
        else:
            rect2 = np.array([])            
        self.__open_win(self.color_data)
        return rect2,type   #绿色方框，左上角的坐标,和物体的宽度\高度，单位pixels

    def locObject(self, wait=None): #等待目标识别，并获取目标三维坐标，wait为等待超时时间
        while True:
            self.camera_data()      #刷新相机数据
            #目标检测  
            pix_pos,_=self.object_detect() #绿色方框，左上角的坐标,和物体的宽度\高度，单位pixels
            #深度处理
            if len(pix_pos) > 0 and pix_pos[0]-30 > 0:      #物体宽度大于30像素
                pos_center=(pix_pos[0] + pix_pos[2] / 2, pix_pos[1] + pix_pos[3] / 2 )         #中心点
                depth_center= self.Depth_data(pos_center,self.MARGIN_PIX,self.MARGIN_PIX)      #pos像素坐标(x,y)
                pos=(pix_pos[0]+self.MARGIN_PIX*2 , pix_pos[1]+self.MARGIN_PIX*2 )      #左上角
                depth_diagonal_1= self.Depth_data(pos,self.MARGIN_PIX,self.MARGIN_PIX)  #pos像素坐标(x,y)
                pos=(pix_pos[0]+pix_pos[2]-self.MARGIN_PIX*2 , pix_pos[1]+pix_pos[3]-self.MARGIN_PIX*2 )      #右下角
                depth_diagonal_2= self.Depth_data(pos,self.MARGIN_PIX,self.MARGIN_PIX)  #pos像素坐标(x,y)
            else:
                rospy.logwarn("Target detection error!")
                continue
            if depth_center!=-1 and depth_diagonal_1!=-1 and depth_diagonal_2!=-1 :
                print("Center pixel:",pos_center,"center depth:",depth_center) 

            #转换相机坐标系
                camera_pos=self.camera_coord_pos_api(pos_center[0],pos_center[1],depth_center) 
                return camera_pos     
            else:
                rospy.logwarn("Incomplete object depth data!")
                continue               

    def arm_LocObject(self):                            #目标识别及稳定性确认
      sumx = sumy = sumz = num =0
      lt_x = []
      lt_y = []
      lt_z = []
      num_numbers=3
      while num<num_numbers:
        ret = self.locObject()                    #相机坐标系
        if ret != -1 :
          if num==0:
            ret1=ret
          if abs(ret1[0]-ret[0])>0.003 or abs(ret1[1]-ret[1])>0.003 or abs(ret1[2]-ret[2])>0.003:
            sumx = sumy = sumz = num =0
            lt_x = []
            lt_y = []
            lt_z = []
            rospy.logwarn("The target is moving! Restart!")
          lt_x.append(ret[0])
          lt_y.append(ret[1])
          lt_z.append(ret[2])
          ret1=ret
          sumx+=ret[0]
          sumy+=ret[1]
          sumz+=ret[2]
          num+=1
          if num==num_numbers :
            if stability_check(lt_x)!=1 or  stability_check(lt_y)!=1 or stability_check(lt_z)!=1:
              sumx = sumy = sumz = num =0
              lt_x = []
              lt_y = []
              lt_z = []
              rospy.logwarn("The target is moving! Restart!") 
      camera_pos=(sumx/num,sumy/num,sumz/num)         #稳定的相机坐标系下的目标
      pos="%.3f/%.3f/%.3f"%(camera_pos[0],camera_pos[1],camera_pos[2])
      result=self.visual_grasping(pos)
      print(result)

    def stopCamera(self):
      self.pipeline.stop()

if __name__ == '__main__':
  cam=RealsenseCamera(detection.pilldetect)
  while True:
    cam.arm_LocObject()
    time.sleep(1)

    
