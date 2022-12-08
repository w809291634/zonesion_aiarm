#ifndef __ROS_SERVICE_H__
#define __ROS_SERVICE_H__
#include "config.h"
#include <sys/msg.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <errno.h>

typedef struct {
    char service[50];               // 服务名
    char pathname[50];              // 消息队列
    char proj;
    unsigned char timeout;          // 超时时间，单位s
    key_t key;
    int msg_id;
    struct {
        long int msg_type;
        char text[MSGBUFSIZE];
    }msg_st;                        //消息缓存区
}service_obj;
typedef service_obj* service_obj_t;

typedef struct {
    FILE * fp;
    char * buffer;
    unsigned int buflen;
    char * tag;
    char** e_list;

    service_obj_t service;
} service_proc_args_t;

int ros_service_register(char* ptag,char** list,service_obj_t msg, int bufsize);

#endif
