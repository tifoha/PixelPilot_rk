#ifndef MVLINK_H
#define MVLINK_H

extern int mavlink_port;
extern bool mavlink_dvr_on_arm;
extern int mavlink_thread_signal;

void* __MAVLINK_THREAD__(void* arg);

// DVR arm/disarm control (implemented in main.cpp, called from mavlink.c)
void dvr_arm_start(void);
void dvr_arm_stop(void);

size_t numOfChars(const char s[]);

char* insertString(char s1[], const char s2[], size_t pos);

#endif