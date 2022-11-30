/*********************************************************************************************
* �ļ���zxbee-sys-command.c
* ���ߣ�xuzhy 2018.1.12
* ˵����lte ģ�鹫��zxbeeָ���ģ��
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



/*********************************************************************************************
* ���ƣ�ZXBeeSysCommandProc
* ���ܣ�lteģ�鹫��zxbeeָ���
* ������
* ���أ�
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
int ZXBeeSysCommandProc(char* ptag, char* pval)
{
  int ret = -1;
  if (memcmp(ptag, "ECHO", 4) == 0) {
    ZXBeeAdd(ptag, pval);
    return 1;
  }
  if (memcmp(ptag, "TYPE", 4) == 0) {
    if (pval[0] == '?') {
      char buf[16];
      ret = sprintf(buf, "32%03d", SENSOR_TYPE);
      ZXBeeAdd("TYPE", buf);
      return 1;
    }
  }
 
  return ret;
}


