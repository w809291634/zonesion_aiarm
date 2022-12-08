/*********************************************************************************************
* 文件：ros-sensor.c
* 作者：zonesion
* 说明：ros接口传感器数据获取
* 修改：
* 注释：
*********************************************************************************************/

/*********************************************************************************************
* 头文件
*********************************************************************************************/
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/types.h>

#include "ros-service.h"
#include "../../common/app/zxbee.h"
#include "../../common/app/zxbee-inf.h"
#include "../../common/app/util.h"


#define debug           0x80                                    // 0x80-仅打印无线数据 0x01-打印所有 0x00-不打印

#if (debug & 0x80)
#define ERR printf
#define DEBUG(...)
#else
#if (debug & 0x01)
#define ERR printf
#define DEBUG printf
#else
#define ERR(...)
#define DEBUG(...)
#endif
#endif

static void* thread_service_proc(void *args)
{
  service_proc_args_t* argv = (service_proc_args_t*)args;
  char* p=NULL;
  while(1){
    if (msgrcv(argv->service->msg_id, (void *)&argv->service->msg_st, MSGBUFSIZE, 0, 0) == -1) {
      fprintf(stderr, "msgrcv failed width erro: %d\r\n", errno);
      sleep(1);
    }
    if(memcmp(argv->service->msg_st.text, "rosservice call", 14) == 0){
      argv->fp = popen(argv->service->msg_st.text, "r");
      // argv->fp = popen("ls", "r");
      if (argv->fp == NULL){
        fprintf(stderr, "popen failed width erro: %d\r\n", errno);
      }
      DEBUG("cmd: %s\r\n",argv->service->msg_st.text);
      memset(argv->buffer,0,argv->buflen);
      // uint32_t st=millis();
      // printf("st:%u\r\n",st);
      if(NULL!=fgets(argv->buffer, argv->buflen, argv->fp))
      {
        DEBUG("argv->buffer %s\r\n",argv->buffer);
        if(memcmp(argv->buffer, "result: 0", 9) == 0){
          // 执行成功
          ZXBeeBegin();
          ZXBeeAdd(argv->tag,argv->e_list[0]);        //成功码
          p = ZXBeeEnd();
          ZXBeeInfSend(p, strlen(p));
        }else if(memcmp(argv->buffer, "result: 1", 9) == 0){
          // 执行错误,这里可以定义协议区分具体错误并返回
          ZXBeeBegin();
          ZXBeeAdd(argv->tag,argv->e_list[1]);        //错误码
          p = ZXBeeEnd();
          ZXBeeInfSend(p, strlen(p));
        }
      }else{
        // 执行错误,这里可以定义协议区分具体错误并返回
        ZXBeeBegin();
        ZXBeeAdd(argv->tag,argv->e_list[1]);        //错误码
        p = ZXBeeEnd();
        ZXBeeInfSend(p, strlen(p));
      }
      pclose(argv->fp);
    }
  }
  fclose(argv->fp);
  free(argv->buffer);
  free(argv);
  return NULL;
}

// 处理与机械臂应用程序之间的服务通讯，并向服务器返回处理结果
// key: 消息队列key值
// bufsize：命令返回字符缓存大小
// ptag: 命令对应的关键字
int ros_service_register(char* ptag,char** list,service_obj_t srv, int bufsize)
{
    service_proc_args_t *pa=NULL;
    
    srv->key=ftok( srv->pathname, srv->proj);
    if (srv->key  == -1) {
        fprintf ( stderr, "ftok failed width erro: %d\r\n", errno );
        return -1;
    }
    srv->msg_id = msgget(srv->key, IPC_CREAT|0666);    // 创建消息队列
    if (srv->msg_id  == -1) {
        fprintf ( stderr, "msgget failed width erro: %d\r\n", errno );
        return -1;
    }
    void *pb = malloc(bufsize);
    if (pb == NULL) {
        return -2;
    }
    pa = malloc(sizeof (service_proc_args_t));
    if (pa == NULL){
        free(pb);
        return -3;
    }
    pa->buffer = pb;
    pa->buflen = bufsize;
    pa->tag = ptag;
    pa->e_list = list;
    pa->service = srv;
    pthread_t *pt = malloc(sizeof (pthread_t));
    if (pt == NULL) {
        free(pb);
        free(pa);
        return -4;
    }
    pthread_create(pt, NULL, thread_service_proc, pa);      //pt 没有回收
    return 0;
}


