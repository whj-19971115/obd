#ifndef NXPLIB_TIMER_H_
#define NXPLIB_TIMER_H_

void LPTMR_init(void);//2ms
int  TimerCreate(unsigned int time_2ms);
int  TimerDelete(unsigned int time_2ms);
int TimerOutGet(unsigned int time_2ms);

void LPIT0_init (void);//1ms
void LPTMR_Reset();
int  TimerLpitCreate(unsigned int time_ms);
int  TimerLpitDelete(unsigned int time_ms);
int TimerLpitOutGet(unsigned int time_ms);

void LPTMR_noIrq_init(void);
int LPTMR_COUNT();

void LPIT0_noIrq_init (void);
int LPIT0_COUNT();
#endif
