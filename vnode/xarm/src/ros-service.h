#ifndef __ROS_SERVICE_H__
#define __ROS_SERVICE_H__
#include "config.h"
#include <sys/msg.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <errno.h>

typedef struct {
    char pathname[50];
    char proj;
    key_t key;
    int msg_id;
    struct {
        long int msg_type;
        char text[MSGBUFSIZE];
    }msg_st;
}msg_queue;
typedef msg_queue* msg_queue_t;

typedef struct {
    FILE * fp;
    char * buffer;
    unsigned int buflen;
    char * tag;
    char** e_list;

    msg_queue_t msg_queue;
} service_proc_args_t;

int ros_service_register(char* ptag,char** list,msg_queue_t msg, int bufsize);

#endif
