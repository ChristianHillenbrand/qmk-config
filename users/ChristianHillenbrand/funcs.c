#include "funcs.h"

__attribute__((weak)) bool is_left_key(keyrecord_t* record) {
  return record->event.key.row < MATRIX_ROWS / 2;
}

__attribute__((weak)) bool is_right_key(keyrecord_t* record) {
  return !is_left_key(record);
}

__attribute__((weak)) bool is_bottom_key(keyrecord_t* record) {
  return record->event.key.row == MATRIX_ROWS / 2 - 1 ||
    record->event.key.row == MATRIX_ROWS - 1;
}

__attribute__((weak)) bool process_record_user_special(uint16_t keycode, keyrecord_t* record){
  return true;
}
