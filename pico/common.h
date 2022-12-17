#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"

int state;                      /* 内部状態 */

float omegaA;
float omegaB;

extern void eyes_init(void);
extern void eyePattern(void);

