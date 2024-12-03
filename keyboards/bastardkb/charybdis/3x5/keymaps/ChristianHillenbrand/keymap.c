#include QMK_KEYBOARD_H

#define LH0 LT(L_FUN, KC_BSPC)
#define LH1 LT(L_NAV, KC_SPC)
#define X_LH KC_ESC,

const key_override_t shift_bspc = ko_make_basic(MOD_MASK_SHIFT, LT(L_FUN, KC_BSPC), KC_DEL);
const key_override_t shift_spc = ko_make_basic(MOD_MASK_SHIFT, LT(L_NAV, KC_SPC), KC_TAB);

#define EXTRA_KEY_OVERRIDES &shift_bspc, &shift_spc,

#include "base.h"

/*********
 * MOUSE *
 *********/

void pointing_device_init_user(void) {
  set_auto_mouse_enable(true);
}

bool is_mouse_record_user(uint16_t keycode, keyrecord_t* record) {
  switch (keycode) {
    case DRGSCRL:
    case SNIPING:
      return true;

    case QK_ONE_SHOT_MOD ... QK_ONE_SHOT_MOD_MAX:
      return true;

    case C(US_Y):
    case C(US_X):
    case C(US_C):
    case C(US_V):
    case C(US_Z):
      return true;

    default:
      return false;
  }
}
