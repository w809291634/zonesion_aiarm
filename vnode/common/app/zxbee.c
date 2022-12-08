/*********************************************************************************************
* �ļ���zxbee.c
* ���ߣ�xuzhy 
* ˵����
*       
*       
*      
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "zxbee.h"
#include "zxbee-inf.h"
#include <pthread.h>
 
static char wbuf[1024];
static pthread_mutex_t ZXBeemutex = PTHREAD_MUTEX_INITIALIZER;

/*********************************************************************************************
* ���ƣ�ZXBeeBegin
* ���ܣ�ZXBeeָ��ϳɿ�ʼ
* ������
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void ZXBeeBegin(void)
{
  pthread_mutex_lock ( &ZXBeemutex );
  wbuf[0] = '{';
  wbuf[1] = '\0';
}
/*********************************************************************************************
* ���ƣ�ZXBeeAdd
* ���ܣ�����һ�����ݵ�zxbeeָ��
* ������tag����������ʶ
        val��������ֵ
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
int ZXBeeAdd(char* tag, char* val)
{
  int offset = strlen(wbuf);
  sprintf(&wbuf[offset],"%s=%s,", tag, val);
  return 0;
}
/*********************************************************************************************
* ���ƣ�ZXBeeEnd
* ���ܣ�zxbeeָ�����ӽ���
* ������
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
char* ZXBeeEnd(void)
{
  int offset = strlen(wbuf);
  wbuf[offset-1] = '}';
  wbuf[offset] = '\0';
  if (offset > 2) {
    pthread_mutex_unlock ( &ZXBeemutex );
    return wbuf;
  }
  pthread_mutex_unlock ( &ZXBeemutex );
  return NULL;
}

/*********************************************************************************************
* ���ƣ�ZXBeeDecodePackage
* ���ܣ�zxbeeָ�����
* ������
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
char* ZXBeeDecodePackage(char *pkg, int len)
{
   char *p;
  char *ptag = NULL;
  char *pval = NULL;
  //int len = strlen(pkg);

  if (pkg[0] != '{' || pkg[len-1] != '}') return NULL;
  
  ZXBeeBegin();

  pkg[len-1] = 0;
  p = pkg+1;
  do {
    ptag = p;
    p = strchr(p, '=');
    if (p != NULL) {
      *p++ = 0;
      pval = p;
      p = strchr(p, ',');
      if (p != NULL) *p++ = 0;
        int ret;
        ret = ZXBeeSysCommandProc(ptag, pval);
        if (ret < 0) {
           ret = ZXBeeUserProcess(ptag, pval);
        }
      }
  } while (p != NULL);
  p = ZXBeeEnd();

  return p;
}
