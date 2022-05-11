
#ifndef __RGB_MATRIX_USER_H__
#define __RGB_MATRIX_USER_H__

#include "quantum/color.h"

void    rgb_matrix_user_init(void);
void    rgb_matrix_user_tick(void);
uint8_t rgb_matrix_user_numLED(void);
void    rgb_matrix_user_render(int ledIdx);
RGB     rgb_matrix_user_getColor(int ledIdx);

void    rgb_matrix_user_process(uint8_t row, uint8_t col, bool pressed);
void    rgb_matrix_user_sleep(void);

#endif