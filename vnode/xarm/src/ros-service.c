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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h> 
#include <pthread.h>

#include "ros-service.h"
static void* thread_service_proc(void *args)
{
    service_proc_args_t* argv = (service_proc_args_t*)args;
    long int msgtype = 0;

    while(1){
        if ( msgrcv ( argv->msg_queue->msg_id, (void *)&argv->msg_queue->msg_st, MSGBUFSIZE, msgtype, 0 ) == -1 ) {
            fprintf ( stderr, "msgrcv failed width erro: %d\r\n", errno );
            sleep(1);
        }
        printf ( "You wrote: %s\r\n", argv->msg_queue->msg_st.text );
        if(memcmp(argv->msg_queue->msg_st.text, "rosservice call", 14) == 0){
            printf ( "argv->fp1: %d\r\n", argv->fp );
            argv->fp = popen(argv->msg_queue->msg_st.text, "r");
            printf ( "argv->fp2: %d\r\n", argv->fp );
            if (argv->fp == NULL){
                fprintf ( stderr, "popen failed width erro: %d\r\n", errno );

            }
            // while((fgets(argv->buffer, argv->buflen, argv->fp))!=NULL){
            //     printf("argv->buffer %s\r\n",argv->buffer);



            // }
            pclose(argv->fp);

        }

    }
    fclose(argv->fp);
    free(argv->buffer);
    free(argv);
    return NULL;
}

// 处理与机械臂应用程序之间的服务通讯，并向服务器返回处理结果
// key:消息队列key值
// bufsize：命令返回字符缓存
int ros_service_register(msg_queue_t msg,int bufsize)
{
    service_proc_args_t *pa=NULL;


    FILE *fp = NULL;
 
     char buf[128] = { 0 };
 
     if ((fp = popen("ls .", "r")) == NULL){
        perror("popen error");
        return -1;
     }
 
     while (fgets(buf, 128, fp) != NULL){
        printf("%s", buf);
     }
 
     pclose(fp);










    msg->key=ftok( msg->pathname, msg->proj);
    if (msg->key  == -1) {
        fprintf ( stderr, "ftok failed width erro: %d\r\n", errno );
        return -1;
    }
    msg->msg_id = msgget(msg->key, IPC_CREAT|0666);    // 创建消息队列
    if (msg->msg_id  == -1) {
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
    pa->msg_queue = msg;
    pthread_t *pt = malloc(sizeof (pthread_t));
    if (pt == NULL) {
        free(pb);
        free(pa);
        return -4;
    }
    pthread_create(pt, NULL, thread_service_proc, pa);      //pt 没有回收
    return 0;
}


