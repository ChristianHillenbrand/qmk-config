#include QMK_KEYBOARD_H

#define X_LH KC_LCTL,

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

    case C(US_X):
    case C(US_C):
    case C(US_V):
      return true;

    default:
      return false;
  }
}
