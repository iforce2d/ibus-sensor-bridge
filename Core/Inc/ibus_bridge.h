#ifndef IBUS_BRIDGE_H
#define IBUS_BRIDGE_H

#include <stdint.h>

void ibus_bridge_init(void);
void ibus_bridge_service(void);
uint8_t ibus_bridge_fc_link_healthy(void);
void ibus_bridge_usart1_irq(void);
void ibus_bridge_usart2_irq(void);

#endif
