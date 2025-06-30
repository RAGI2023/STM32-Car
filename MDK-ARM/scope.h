#ifndef __SCOPE_H
#define __SCOPE_H

#include "main.h"

void Scope_SendNames(const char* names);
void Scope_SendData(float* data, uint8_t num_channels);

#endif /* __SCOPE_H */
