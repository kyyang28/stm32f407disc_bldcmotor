#ifndef __RUNTIME_CONFIG_H
#define __RUNTIME_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

bool sensors(uint32_t mask);
void sensorSet(uint32_t mask);
void sensorClear(uint32_t mask);
uint32_t sensorsMask(void);

#endif	// __RUNTIME_CONFIG_H
