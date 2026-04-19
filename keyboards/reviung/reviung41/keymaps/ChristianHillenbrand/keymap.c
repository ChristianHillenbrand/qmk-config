#include QMK_KEYBOARD_H

/**********
 * STATUS *
 **********/

void keyboard_pre_init_user(void) {
  gpio_set_pin_output(17);
  gpio_write_pin_low(17);
}

/**********
 * KEYMAP *
 **********/

#define X_LT KC_ESC,
#define X_LM KC_TAB,
#define X_LB CW_TOGG,

#define LH1 KC_LGUI
#define LH0 LLS(L_NAV)
#define X_CH LT(L_FUN_SYM, KC_SPC),
#define RH0 LRS(L_NUM)
#define RH1 KC_RGUI

#define X_RT KC_BSPC,
#define X_RM KC_ENT,
#define X_RB KC_DEL,

#include "base.h"

bool is_left_key(keyrecord_t* record)
{
  return record->event.key.row < (MATRIX_ROWS - 1) / 2 ||
    (is_bottom_key(record) && record->event.key.col < (MATRIX_COLS - 1) / 2);
}

bool is_bottom_key(keyrecord_t* record)
{
  return record->event.key.row == MATRIX_ROWS - 1;
}
