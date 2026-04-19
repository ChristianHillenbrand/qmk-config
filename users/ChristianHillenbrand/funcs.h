#pragma once

#include QMK_KEYBOARD_H

bool is_left_key(keyrecord_t* record);
bool is_right_key(keyrecord_t* record);
bool is_bottom_key(keyrecord_t* record);

bool process_record_user_special(uint16_t keycode, keyrecord_t* record);
