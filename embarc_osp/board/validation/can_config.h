#ifndef _CAN_CONFIG_H_
#define _CAN_CONFIG_H_



void *canif_get_txpdu_cfg(uint32_t *cnt);
void *canif_get_rxpdu_cfg(uint32_t *cnt);
void *cantp_get_nsdu(uint32_t *cnt);
void *can_config_get(void);
#endif
