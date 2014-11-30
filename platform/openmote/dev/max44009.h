#ifndef _MAX44009_H
#define _MAX44009_H

int enable_max44009(void);
int rst_max44009(void);
int readReg(uint8_t addr, uint8_t *data);
int readLux(void);
float getLux(void);

#endif
