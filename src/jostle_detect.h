#ifndef JOSTLE_DETECT_H
#define JOSTLE_DETECT_H

#include "boards.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

void jostle_detect_init();
bool jostle_detect_get_flag();
void jostle_detect_clear_flag();

#ifdef __cplusplus
}
#endif

#endif // JOSTLE_DETECT_H
