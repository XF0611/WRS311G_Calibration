#ifndef _INIT_H_
#define _INIT_H_

extern uint8_t update_test[8];

void can_uds_init(void);
void timer_expired_callback(void *params);
void timer_expired_app_callback(void *params);
#endif
