#ifndef JOSTLE_DETECT_H
#define JOSTLE_DETECT_H

#include "stdint.h"
#include "boards.h"

void jostle_detect_init();
bool jostle_detect_get_flag();
void jostle_detect_clear_flag();


#endif // JOSTLE_DETECT_H
