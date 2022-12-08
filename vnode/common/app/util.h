#ifndef __UTILS_H__
#define __UTILS_H__
#include <sys/time.h>
#include <stdint.h>
#include <stdio.h>

uint32_t millis(void);
int timeoutMs(uint32_t ms);
void msleep(int ms);
#endif