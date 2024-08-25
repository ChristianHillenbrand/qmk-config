#include QMK_KEYBOARD_H

#define X_LH KC_LCTL,

#include "base.h"

/*********
 * MOUSE *
 *********/

void pointing_device_init_user(void) {
  set_auto_mouse_enable(true);
}
