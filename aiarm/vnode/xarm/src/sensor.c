/*********************************************************************************************
 * 文件：sensor.c
 * 作者：zonesion
 * 说明：通用节点传感器程序
 * 修改：
 * 注释：
 *********************************************************************************************/

/*********************************************************************************************
 * 头文件
 *********************************************************************************************/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <unistd.h>
#include "../../common/app/zxbee.h"
#include "../../common/app/zxbee-inf.h"
#include "../../common/app/util.h"
#include "sensor.h"
#include "ros-sensor.h"
#include "ros-service.h"

#include <sys/msg.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <errno.h>

/*********************************************************************************************
 * 全局变量
 *********************************************************************************************/
#define ITEM_SIZE 64

static uint8_t D0 = 0x9F;  // 默认打开主动上报功能
static uint8_t D1 = 0;     // 控制位
static char A0[ITEM_SIZE]; // 关节实时位姿  {A0=0/0/0/0/0}               返回5个关节值
static char A1[ITEM_SIZE]; // 空间实时位姿  {A1=x/y/z/R/P/Y}
static uint16_t V0 = 1;    // A0~A7上传时间间隔，默认1S
static char V1[ITEM_SIZE]; // 查询或者设置机械臂的5个关节值，不含夹具。{V1=0/0/0/0/0}
static char V2[ITEM_SIZE]; // 查询或者设置机械臂的位姿，RPY表示旋转方向 {V2=x/y/z/R/P/Y}
static char V3[ITEM_SIZE]; // 查询或者设置机械臂的夹具 {V3=夹具值}
static char V4[ITEM_SIZE]; // 设置深度相机识别的物体空间坐标，进行分析并抓取 {V4=x/y/z}

struct
{
  /* 机械臂当前关节位姿信息 */
  float joint1; //轴1，夹具旋转轴
  float joint2; //轴2
  float joint3;
  float joint4;
  float joint5;
  /* 机械臂当前空间位置信息  */
  float x;
  float y;
  float z;
  float R;
  float P;
  float Y;
  /* 关节目标*/
  char joint_target[ITEM_SIZE];
  /* 空间目标*/
  char space_target[ITEM_SIZE];
} xarm_info;

#define property_set(i, exp)   \
  do                           \
  {                            \
    char *p = getItem(msg, i); \
    if (p != NULL)             \
    {                          \
      exp;                     \
    }                          \
  } while (0)

char *getItem(char *str, int i)
{
  int j;
  char *p = str;
  for (j = 0; j < i; j++)
  {
    if (p == NULL)
      break;
    p = strchr(p, ',');
    if (p != NULL)
      p++;
  }
  return p;
}

/*********************************************************************************************
 * 名称：on_target_joint_msg_cb
 * 功能：机械臂关节位置
 * 参数：
 * 返回：
 * 修改：
 * 注释：
 *********************************************************************************************/
static void on_target_joint_msg_cb(unsigned long long tm, char *msg)
{
  // printf("joint_msg_cb %lld, %s\r\n", tm, msg);
  if (tm != 0) {
    property_set(1, xarm_info.joint1 = atof(p));
    property_set(2, xarm_info.joint2 = atof(p));
    property_set(3, xarm_info.joint3 = atof(p));
    property_set(4, xarm_info.joint4 = atof(p));
    property_set(5, xarm_info.joint5 = atof(p));
    // printf("joint:%f,%f,%f\r\n",xarm_info.joint1,xarm_info.joint3,xarm_info.joint5);
  }
}

/*********************************************************************************************
* 名称： on_target_space_msg_cb
* 功能： 机械臂空间位置
* 参数：
* 返回：
* 修改：
* 注释：
*********************************************************************************************/
static void  on_target_space_msg_cb (unsigned long long tm, char *msg)
{
  // printf("space_msg_cb %lld, %s\r\n", tm, msg);
  if (tm != 0) {
    property_set(1, xarm_info.x = atof(p));
    property_set(2, xarm_info.y = atof(p));
    property_set(3, xarm_info.z = atof(p));
    property_set(4, xarm_info.R = atof(p));
    property_set(5, xarm_info.P = atof(p));
    property_set(6, xarm_info.Y = atof(p));
    // printf("space_x:%f,%f,%f\r\n",xarm_info.x,xarm_info.y ,xarm_info.z);
    // printf("space_R:%f,%f,%f\r\n",xarm_info.R,xarm_info.P ,xarm_info.Y);
  }
}
/*********************************************************************************************
 * 名称：updateV0()
 * 功能：更新V0的值
 * 参数：*val -- 待更新的变量
 * 返回：
 * 修改：
 * 注释：
 *********************************************************************************************/
void updateV0(char *val)
{
  //将字符串变量val解析转换为整型变量赋值
  V0 = atoi(val); // 获取数据上报时间更改值
}
/*********************************************************************************************
 * 名称：updateA0()
 * 功能：更新A0的值
 * 参数：
 * 返回：
 * 修改：
 * 注释：
 *********************************************************************************************/
void updateA0(void)
{
  //机械臂关节值
  sprintf(A0, "%.3f/%.3f/%.3f/%.3f/%.3f",
      xarm_info.joint1,xarm_info.joint2,
      xarm_info.joint3,xarm_info.joint4,
      xarm_info.joint5);
}
// /*********************************************************************************************
// * 名称：updateA1()
// * 功能：更新A1的值
// * 参数：
// * 返回：
// * 修改：
// * 注释：
// *********************************************************************************************/
void updateA1(void)
{
  //机械臂空间值
  sprintf(A1, "%.3f/%.3f/%.3f/%.3f/%.3f/%.3f",
      xarm_info.x,xarm_info.y,
      xarm_info.z,xarm_info.R,
      xarm_info.P,xarm_info.Y);
}
// /*********************************************************************************************
// * 名称：updateA2()
// * 功能：更新A2的值
// * 参数：
// * 返回：
// * 修改：
// * 注释：
// *********************************************************************************************/
// void updateA2(void)
// {
//      //车牌信息，字符串
//   strcpy(A2, LicensePlate);
// }
// /*********************************************************************************************
// * 名称：updateA3()
// * 功能：更新A3的值
// * 参数：
// * 返回：
// * 修改：
// * 注释：
// *********************************************************************************************/
// void updateA3(void)
// {
//     //前后左右的距离，格式为a&b&c&d 单位m
//     sprintf(A3, "%.2f&%.2f&%.2f&%.2f", (xarm_info.sonar1),
//         (xarm_info.sonar3), (xarm_info.sonar2),
//         (xarm_info.sonar4));
// }

//定义：服务器错误码
static const char* err_code[]={
  "0",        //成功，任务结束
  "-1",       //错误
  "-2",       //服务请求超时
};
//定义：服务名，消息队列路径，名称，超时，消息类型
static service_obj joint_cmd={"/vnode_xarm/joint_target","/home/zonesion/catkin_ws/src/aiarm/tmp",'a'
,.timeout=10, .result=V1, .msg_st.msg_type=1,};
static service_obj space_cmd={"/vnode_xarm/space_target","/home/zonesion/catkin_ws/src/aiarm/tmp",'b'
,.timeout=10, .result=V2, .msg_st.msg_type=1,};
static service_obj fixture_stroke={"/vnode_xarm/fixture_stroke","/home/zonesion/catkin_ws/src/aiarm/tmp",'c'
,.timeout=10, .result=V3, .msg_st.msg_type=1,};
static service_obj vis_grasp={"/vnode_xarm/visual_grasping","/home/zonesion/catkin_ws/src/aiarm/tmp",'d'
,.timeout=10, .result=V4, .msg_st.msg_type=1,};
static service_obj preset_positions={"/vnode_xarm/preset_positions","/home/zonesion/catkin_ws/src/aiarm/tmp",'e'
,.timeout=10, .msg_st.msg_type=1,};
/*********************************************************************************************
 * 名称：sensorInit()
 * 功能：传感器硬件初始化
 * 参数：无
 * 返回：无
 * 修改：
 * 注释：
 *********************************************************************************************/
void sensorInit(void)
{
  char aiarm_path[128]={0};
  // 初始化消息接收线程
  ros_topic_register("/aiarm/arm_joint", on_target_joint_msg_cb, 256);
  ros_topic_register("/aiarm/arm_space", on_target_space_msg_cb, 256);
  // 消息队列路径
  if(getcwd(aiarm_path, 128)!=NULL){
    char* p = strstr(aiarm_path, "aiarm");
    p[5]=0;
    strcat(aiarm_path,"/tmp");
    strcpy(joint_cmd.pathname,aiarm_path);
    strcpy(space_cmd.pathname,aiarm_path);
    strcpy(fixture_stroke.pathname,aiarm_path);
    strcpy(vis_grasp.pathname,aiarm_path);
    strcpy(preset_positions.pathname,aiarm_path);
    printf("INFO:msg queue path:%s\r\n",aiarm_path);
  }else{
    printf("ERROR:get path fail!");
  }
  // 注册服务类型，错误码，服务对象，命令接收缓存区大小
  ros_service_register("V1",(char **)err_code,&joint_cmd,128);
  ros_service_register("V2",(char **)err_code,&space_cmd,128);
  ros_service_register("V3",(char **)err_code,&fixture_stroke,128);
  ros_service_register("V4",(char **)err_code,&vis_grasp,128);
  ros_service_register("D1",(char **)err_code,&preset_positions,128);
}
/*********************************************************************************************
 * 名称：sensorUpdate()
 * 功能：处理主动上报的数据
 * 参数：无
 * 返回：无
 * 修改：
 * 注释：
 *********************************************************************************************/
void sensorUpdate(void)
{
  ZXBeeBegin(); // 智云数据帧格式包头

  // 根据D0的位状态判定需要主动上报的数值
  if ((D0 & 0x01) == 0x01)
  {
    updateA0();
    ZXBeeAdd("A0", A0);
  }
  if ((D0 & 0x02) == 0x02)
  {
    updateA1();
    ZXBeeAdd("A1", A1);
  }
  if ((D0 & 0x04) == 0x04)
  {
    // updateA2();
    // ZXBeeAdd("A2", A2);
  }
  if ((D0 & 0x08) == 0x08)
  {
    // updateA3();
    // ZXBeeAdd("A3", A3);
  }
  if ((D0 & 0x10) == 0x10)
  {
    // updateA4();
    // ZXBeeAdd("A4", A4);
  }
  if ((D0 & 0x20) == 0x20)
  {
    // ZXBeeAdd("A5", A5);
  }
  if ((D0 & 0x40) == 0x40)
  {
    // updateA6();
    // ZXBeeAdd("A6", A6);
  }
  if ((D0 & 0x80) == 0x80)
  {
    // updateA7();
    // ZXBeeAdd("A7", A7);
  }

  char *p = ZXBeeEnd(); // 智云数据帧格式包尾
  if (p != NULL)
  {
    ZXBeeInfSend(p, strlen(p)); // 将需要上传的数据进行打包操作，并通过zb_SendDataRequest()发送到协调器
  }
}
/*********************************************************************************************
 * 名称：sensorCheck()
 * 功能：传感器监测
 * 参数：无
 * 返回：无
 * 修改：
 * 注释：
 *********************************************************************************************/
void sensorCheck(void)
{
  // ZXBeeBegin();
  // // 根据D0的位来决定是否上报
  // if (strcmp(V1, xarm_info.target)) {
  //     strcpy(V1, xarm_info.target);
  //     ZXBeeAdd("V1", V1);
  // }
  // if ((D0 & (1<<3)) != 0) {
  //     char a3[ITEM_SIZE];
  //     strcpy(a3, A3);
  //     updateA3();
  //     if (strcmp(a3, A3)){
  //         ZXBeeAdd("A3", A3);
  //     }
  // }
  // if ((D0 & (1<<4)) != 0) {
  //     char a4[ITEM_SIZE];
  //     strcpy(a4, A4);
  //     updateA4();
  //     if (strcmp(a4, A4)){
  //         ZXBeeAdd("A4", A4);
  //     }
  // }
  // if ((D0 & (1<<7)) != 0) {
  //     char a7[ITEM_SIZE];
  //     strcpy(a7, A7);
  //     updateA7();
  //     if (strcmp(a7, A7)){
  //         ZXBeeAdd("A7", A7);
  //     }
  // }
  // char *p = ZXBeeEnd();
  // if ((p != NULL)) {
  //     int len = strlen(p);
  //     ZXBeeInfSend(p, len);
  // }
}
/*********************************************************************************************
 * 名称：sensorControl()
 * 功能：传感器控制
 * 参数：cmd - 控制命令
 * 返回：无
 * 修改：
 * 注释：
 *********************************************************************************************/
void sensorControl(uint8_t cmd)
{
  // 根据cmd参数处理对应的控制程序
  char data;
  // 处理位1
  if(cmd& 0x01)data='1';
  else data='0';
  sprintf(preset_positions.msg_st.text,"rosservice call %s \"data: %c\"",preset_positions.service,data);
  if(msgsnd(preset_positions.msg_id,(void *)&preset_positions.msg_st,strlen(preset_positions.msg_st.text),IPC_NOWAIT)== -1){
    fprintf ( stderr, "preset_positions msgsnd failed\r\n" );
  }; 
}
/*********************************************************************************************
 * 名称：ZXBeeUserProcess()
 * 功能：解析收到的控制命令
 * 参数：*ptag -- 控制命令名称
 *       *pval -- 控制命令参数
 * 返回：ret -- pout字符串长度
 * 修改：
 * 注释：
 *********************************************************************************************/
int ZXBeeUserProcess(char *ptag, char *pval)
{
  int val;
  int ret = 0;
  char p[64];

  // 将字符串变量pval解析转换为整型变量赋值
  val = atoi(pval);
  // 控制命令解析
  if (0 == strcmp("CD0", ptag))
  { // 对D0的位进行操作，CD0表示位清零操作
    D0 &= ~val;
  }
  if (0 == strcmp("OD0", ptag))
  { // 对D0的位进行操作，OD0表示位置一操作
    D0 |= val;
  }
  if (0 == strcmp("D0", ptag))
  { // 查询上报使能编码
    if (0 == strcmp("?", pval))
    {
      sprintf(p, "%u", D0);
      ZXBeeAdd("D0", p);
    }
  }
  if (0 == strcmp("CD1", ptag))
  { // 对D1的位进行操作，CD1表示位清零操作
    D1 &= ~val;
    sensorControl(D1); // 处理执行命令
    sprintf(p, "%u", D1);
    ZXBeeAdd("D1", p);
  }
  if (0 == strcmp("OD1", ptag))
  { // 对D1的位进行操作，OD1表示位置一操作
    D1 |= val;
    sensorControl(D1); // 处理执行命令
    sprintf(p, "%u", D1);
    ZXBeeAdd("D1", p);
  }
  if (0 == strcmp("D1", ptag))
  { // 查询执行器命令编码
    if (0 == strcmp("?", pval))
    {
      sprintf(p, "%u", D1);
      ZXBeeAdd("D1", p);
    }
  }
  if (0 == strcmp("A0", ptag))
  {
    if (0 == strcmp("?", pval))
    {
      updateA0();
      ZXBeeAdd("A0", A0);
    }
  }
  if (0 == strcmp("A1", ptag))
  {
    if (0 == strcmp("?", pval))
    {
      updateA1();
      ZXBeeAdd("A1", A1);
    }
  }
  if (0 == strcmp("V0", ptag))
  {
    if (0 == strcmp("?", pval))
    {
      ret = sprintf(p, "%u", V0); // 上报时间间隔
      ZXBeeAdd("V0", p);
    }
    else
    {
      updateV0(pval);
    }
  }
  if (0 == strcmp("V1", ptag))
  {
    if (0 == strcmp("?", pval))
    {
      ZXBeeAdd("V1", V1);
    }
    else
    { 
      strcpy(V1, pval);
      sprintf(joint_cmd.msg_st.text,"rosservice call %s \"joint: '%s'\"",joint_cmd.service,pval);
      if(msgsnd(joint_cmd.msg_id,(void *)&joint_cmd.msg_st,strlen(joint_cmd.msg_st.text),IPC_NOWAIT)== -1){
        fprintf ( stderr, "msgsnd failed\r\n" );
      };
    }
  }
  if (0 == strcmp("V2", ptag))
  {
    if (0 == strcmp("?", pval))
    {
      ZXBeeAdd("V2", V2);
    }
    else
    {        
      strcpy(V2, pval);      
      sprintf(space_cmd.msg_st.text,"rosservice call %s \"space: '%s'\"",space_cmd.service,pval);
      if(msgsnd(space_cmd.msg_id,(void *)&space_cmd.msg_st,strlen(space_cmd.msg_st.text),IPC_NOWAIT)== -1){
        fprintf ( stderr, "msgsnd failed\r\n" );
      };
    }
  }
  if (0 == strcmp("V3", ptag))
  {
    if (0 == strcmp("?", pval))
    {
      ZXBeeAdd("V3", V3);
    }
    else
    {        
      strcpy(V3, pval);      
      sprintf(fixture_stroke.msg_st.text,"rosservice call %s \"data: %s\"",fixture_stroke.service,pval);
      if(msgsnd(fixture_stroke.msg_id,(void *)&fixture_stroke.msg_st,strlen(fixture_stroke.msg_st.text),IPC_NOWAIT)== -1){
        fprintf ( stderr, "msgsnd failed\r\n" );
      };
    }
  }
  if (0 == strcmp("V4", ptag))
  {
    if (0 == strcmp("?", pval))
    {
      ZXBeeAdd("V4", V4);
    }
    else
    {        
      strcpy(V4, pval);      
      sprintf(vis_grasp.msg_st.text,"rosservice call %s \"pos: '%s'\"",vis_grasp.service,pval);
      if(msgsnd(vis_grasp.msg_id,(void *)&vis_grasp.msg_st,strlen(vis_grasp.msg_st.text),IPC_NOWAIT)== -1){
        fprintf ( stderr, "msgsnd failed\r\n" );
      };
    }
  }
  return ret;
}

/*********************************************************************************************
 * 名称：sensor()
 * 功能：传感器采集线程
 * 参数：
 * 返回：
 * 修改：
 * 注释：
 *********************************************************************************************/
void sensorPoll(void)
{
  static uint32_t lastUpdateTm, sensorCheckTm;

  if (timeoutMs(sensorCheckTm))
  {
    sensorCheck();
    sensorCheckTm = millis() + 1000;
  }
  if (timeoutMs(lastUpdateTm))
  {
    sensorUpdate();
    lastUpdateTm = millis() + 1000 * V0;
  }
}
