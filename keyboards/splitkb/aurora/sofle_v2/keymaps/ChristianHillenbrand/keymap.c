#include QMK_KEYBOARD_H

#include "oled.h"

#include <transactions.h>

#define IDLE_MS 100

/**********
 * STATUS *
 **********/

void keyboard_pre_init_user(void) {
  setPinOutput(24);
  writePinHigh(24);
}

void rpc_caps_word_slave_handler(uint8_t in_buflen, const void* in_data, uint8_t out_buflen, void* out_data) {
  const bool* caps_word_state = (const bool*)(in_data);

  if (*caps_word_state) {
    caps_word_on();
  } else {
    caps_word_off();
  }
}

void rpc_space_pressed_slave_handler(uint8_t in_buflen, const void* in_data, uint8_t out_buflen, void* out_data) {
  trigger_jump();
}

void keyboard_post_init_user(void) {
  transaction_register_rpc(RPC_CAPS_WORD, rpc_caps_word_slave_handler);
  transaction_register_rpc(RPC_SPACE_PRESSED, rpc_space_pressed_slave_handler);
}

void caps_word_set_user(bool active) {
  if (is_keyboard_left()) {
    if (active) {
      writePinLow(24);
    } else {
      writePinHigh(24);
    }
  }

  if (is_keyboard_master()) {
    transaction_rpc_send(RPC_CAPS_WORD, sizeof(active), &active);
  }
}

bool led_update_user(led_t led_state) {
  if (is_keyboard_left()) {
    return true;
  }

  static bool caps_lock_state = false;
  if (led_state.caps_lock && !caps_lock_state) {
    writePinLow(24);
    caps_lock_state = true;
  } else {
    writePinHigh(24);
    caps_lock_state = false;
  }

  return true;
}

/**********
 * KEYMAP *
 **********/

#define X_NR KC_ESC, US_1, US_2, US_3, US_4, US_5, US_6, US_7, US_8, US_9, US_0, KC_BSPC,

#define X_LT KC_TAB,
#define X_LM CW_TOGG,
#define X_LB KC_LSFT,
#define X_LH KC_LGUI, KC_LALT, KC_LCTL,

#define X_CB XXXXXXX, KC_MUTE,

#define X_RT KC_DEL,
#define X_RM KC_QUOT,
#define X_RB KC_RSFT,
#define X_RH ,KC_RCTL, KC_LALT, KC_RGUI

#include "base.h"

bool process_record_user_special(uint16_t keycode, keyrecord_t* record){
  const uint8_t tap_keycode = QK_LAYER_TAP_GET_TAP_KEYCODE(keycode);

  if (tap_keycode == KC_SPC && record->event.pressed) {
    trigger_jump();
    transaction_rpc_send(RPC_SPACE_PRESSED, 0, NULL);
  }

  return true;
}

/***********
 * ENCODER *
 ***********/

bool encoder_update_user(uint8_t index, bool clockwise) {
  if (index == 0) {
    if (clockwise) {
      tap_code(KC_WH_D);
    } else {
      tap_code(KC_WH_U);
    }
  } else if (index == 1) {
    if (clockwise) {
      tap_code(KC_VOLU);
    } else {
      tap_code(KC_VOLD);
    }
  }
  return false;
}

/*******
 * RGB *
 *******/

bool rgb_matrix_indicators_advanced_user(uint8_t led_min, uint8_t led_max) {
  uint8_t highest_layer = get_highest_layer(layer_state);

  if (highest_layer > 0) {
    for (uint8_t row = 0; row < MATRIX_ROWS; ++row) {
      for (uint8_t col = 0; col < MATRIX_COLS; ++col) {
        uint8_t index = g_led_config.matrix_co[row][col];

        if (index >= led_min && index < led_max && index != NO_LED) {
          uint16_t base_layer_code = keymap_key_to_keycode(L_QWRTY, (keypos_t){col,row});
          uint16_t highest_layer_code = keymap_key_to_keycode(highest_layer, (keypos_t){col,row});
          if (highest_layer_code == base_layer_code || highest_layer_code == KC_TRNS) {
            rgb_matrix_set_color(index, RGB_BLACK);
          }
        }
      }
    }
  }

  return false;
}

/********
 * OLED *
 ********/

bool render_left_display(void) {
  oled_clear();
  render_logo();
  render_logo_text();
  render_line();
  render_space();
  render_layer();
  render_space();
  render_line();
  render_mod_status_shift_ctrl(get_mods() | get_oneshot_mods());
  render_mod_status_alt_gui(get_mods() | get_oneshot_mods());
  return false;
}

bool render_right_display(void) {
  oled_clear();
  render_logo();
  render_logo_text();
  render_line();
  if (layer_state_is(L_MEDIA)) {
    render_rgb_data();
  } else {
    render_space();
    render_wpm();
    render_space();
    render_line();
    render_luna();
  }
  return false;
}

bool oled_task_user(void) {
  if (handle_oled_timeout()) {
    return false;
  }

  if (is_keyboard_left()) {
    return render_left_display();
  } else {
    return render_right_display();
  }
}
