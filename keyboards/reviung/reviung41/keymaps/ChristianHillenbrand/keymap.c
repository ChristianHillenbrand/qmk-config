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
#define X_LM CW_TOGG,
#define X_LB KC_LSFT,

#define LH1 MO(L_FUN)
#define LH0 LLS(L_NAV)
#define X_CH LT(L_FUN_SYM, KC_SPC),
#define RH0 LRS(L_NUM)
#define RH1 MO(L_SYM)

#define X_RT KC_BSPC,
#define X_RM KC_ENT,
#define X_RB KC_RSFT,

#include "base.h"
