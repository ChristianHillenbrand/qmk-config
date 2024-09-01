#pragma once

#include "config.h"
#include QMK_KEYBOARD_H

#include <keymap_us_extended.h>

#include "features/achordion.h"

#include "extra_keys.h"
#include "funcs.h"
#include "layers.h"

#define HRL (MATRIX_ROWS / 2 - 3) // home row left
#define HRR (MATRIX_ROWS - 3)     // home row right

#define LAYOUT_wrapper(...) LAYOUT(__VA_ARGS__)

/*********************
 * GENERAL FUNCTIONS *
 *********************/

bool IS_HRM(uint16_t keycode) {
  return IS_QK_MOD_TAP(keycode) &&
    QK_MOD_TAP_GET_MODS(keycode) != MOD_RALT;
}

bool IS_ALPHA(uint16_t keycode) {
  return (keycode & 0xFF) >= US_A && (keycode & 0xFF) <= US_Z;
}

bool IS_SPACE(uint16_t keycode) {
  return (keycode & 0xFF) == KC_SPC;
}

bool IS_TYPING(uint16_t keycode)
{
  return (IS_ALPHA(keycode) || IS_SPACE(keycode)) &&
    last_input_activity_elapsed() < REQUIRE_PRIOR_IDLE_MS;
}

bool SHIFT_PRESSED(void) {
  return (get_mods() | get_oneshot_mods()) & MOD_MASK_SHIFT;
}

/**********************
 * CAPS WORD SETTINGS *
 **********************/

bool caps_word_press_user(uint16_t keycode) {
  switch (keycode) {
    case US_A ... US_Z:
    case US_ADIA:
    case US_ODIA:
    case US_UDIA:
    case US_MINS:
      add_weak_mods(MOD_BIT(KC_LSFT));
      return true;

    case CW_TOGG:
    case US_1 ... US_0:
    case US_UNDS:
    case KC_BSPC:
    case KC_DEL:
      return true;

    default:
      return false;
  }
}

/*******************
 * CUSTOM KEYCODES *
 *******************/

enum custom_keycodes {
  KC_LOWER_ = SAFE_RANGE,
  KC_RAISE_,
  KC_MAX,
};

#define KC_LOWER LT(0, KC_LOWER_)
#define KC_RAISE LT(0, KC_RAISE_)

#define LT_MEDIA_SPC LT(L_MEDIA, KC_SPC)
#define LT_FUN_ENT LT(L_FUN, KC_ENT)

bool lower_pressed = false;
bool raise_pressed = false;

bool pre_process_record_user(uint16_t keycode, keyrecord_t* record) {
  static bool is_pressed[UINT8_MAX];
  static uint16_t prev_keycode;

  const uint8_t tap_keycode = keycode & 0xFF;

  if (keycode == KC_LOWER) {
    lower_pressed = record->event.pressed;
  } else if (keycode == KC_RAISE) {
    raise_pressed = record->event.pressed;
  }

  if (record->event.pressed) {
    if ((IS_HRM(keycode, record) || IS_SPACE(keycode)) && IS_TYPING(prev_keycode)) {
      record->keycode = tap_keycode;
      is_pressed[tap_keycode] = true;
    }
    prev_keycode = keycode;
  }
  else if (is_pressed[tap_keycode]) {
    record->keycode = tap_keycode;
    is_pressed[tap_keycode] = false;
  }

  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t* record) {
  if (!process_achordion(keycode, record)) {
    return false;
  }

  if (!process_record_user_special(keycode, record)) {
    return false;
  }

  switch (keycode) {
    case KC_LOWER:
      if (record->tap.count) {
        // tap -> one shot shift
        if (record->event.pressed) {
          if (get_oneshot_mods() & MOD_BIT(KC_LSFT)) {
            del_oneshot_mods(MOD_BIT(KC_LSFT));
          } else {
            add_oneshot_mods(MOD_BIT(KC_LSFT));
          }
        }
        return false;
      } else{
        // hold -> nav layer
        if (record->event.pressed) {
          del_oneshot_mods(MOD_BIT(KC_LSFT));
          layer_on(L_NAV);
        } else {
          layer_off(L_NAV);
        }
        return true;
      }

    case KC_RAISE:
      if (record->tap.count) {
        // tap -> one shot shift
        if (record->event.pressed) {
          if (get_oneshot_mods() & MOD_BIT(KC_RSFT)) {
            del_oneshot_mods(MOD_BIT(KC_RSFT));
          } else {
            add_oneshot_mods(MOD_BIT(KC_RSFT));
          }
        }
        return false;
      } else {
        // hold -> num layer
        if (record->event.pressed) {
          del_oneshot_mods(MOD_BIT(KC_RSFT));
          layer_on(L_NUM);
        } else {
          layer_off(L_NUM);
        }
        return true;
      }

    case MT(MOD_LCTL | MOD_LSFT, US_LPRN):
      if (record->tap.count && record->event.pressed) {
        if (SHIFT_PRESSED()) {
          tap_code16(US_LABK);
        } else {
          tap_code16(US_LPRN);
        }
        return false;
      }
      break;

    case MT(MOD_RCTL | MOD_RSFT, US_RPRN):
      if (record->tap.count && record->event.pressed) {
        if (SHIFT_PRESSED()) {
          tap_code16(US_RABK);
        } else {
          tap_code16(US_RPRN);
        }
        return false;
      }
      break;

    default:
      break;
  }

  return true;
}

void matrix_scan_user(void) {
  achordion_task();
}

bool achordion_chord(uint16_t tap_hold_keycode, keyrecord_t* tap_hold_record, uint16_t other_keycode, keyrecord_t* other_record) {
  if (IS_HRM(tap_hold_keycode)) {
    return achordion_opposite_hands(tap_hold_record, other_record);
  }
  return true;
}

uint16_t achordion_timeout(uint16_t tap_hold_keycode) {
  if (IS_QK_LAYER_TAP(tap_hold_keycode)) {
    return 0;
  }
  return ACHORDION_TIMEOUT;
}

bool achordion_eager_mod(uint8_t mod) {
  return true;
}

/*****************
 * KEY OVERRIDES *
 *****************/

const key_override_t ralt_a = ko_make_basic(MOD_BIT_RALT, US_A, US_ADIA);
const key_override_t ralt_a_ = ko_make_basic(MOD_BIT_RALT, LGUI_T(US_A), US_ADIA);
const key_override_t ralt_o = ko_make_basic(MOD_BIT_RALT, US_O, US_ODIA);
const key_override_t ralt_u = ko_make_basic(MOD_BIT_RALT, US_U, US_UDIA);
const key_override_t ralt_e = ko_make_basic(MOD_BIT_RALT, US_E, US_EURO);
const key_override_t shift_esc = ko_make_basic(MOD_MASK_SHIFT, KC_ESC, KC_TILD);

const key_override_t** key_overrides = (const key_override_t* []){
  &ralt_a,
  &ralt_a_,
  &ralt_o,
  &ralt_u,
  &ralt_e,
  &shift_esc,
  NULL
};

/**********
 * COMBOS *
 **********/

enum combos {
  COMBO_LBRC,
  COMBO_LPRN,

  COMBO_RPRN,
  COMBO_RBRC,

  COMBO_CAPS_WORD
};

// left half combos
const uint16_t PROGMEM combo_lbrc[] = {LALT_T(US_S), LCTL_T(US_D), COMBO_END};
const uint16_t PROGMEM combo_lprn[] = {LCTL_T(US_D), LSFT_T(US_F), COMBO_END};

// right half combos
const uint16_t PROGMEM combo_rprn[] = {RSFT_T(US_J), RCTL_T(US_K), COMBO_END};
const uint16_t PROGMEM combo_rbrc[] = {RCTL_T(US_K), LALT_T(US_L), COMBO_END};

// mixed combos
const uint16_t PROGMEM combo_caps_word[] = {KC_LOWER, KC_RAISE, COMBO_END};

combo_t key_combos[] = {
  // left half combos
  [COMBO_LBRC] = COMBO(combo_lbrc, MT(MOD_LALT | MOD_LCTL, US_LBRC)),
  [COMBO_LPRN] = COMBO(combo_lprn, MT(MOD_LCTL | MOD_LSFT, US_LPRN)),

  // right half combos
  [COMBO_RPRN] = COMBO(combo_rprn, MT(MOD_RCTL | MOD_RSFT, US_RPRN)),
  [COMBO_RBRC] = COMBO(combo_rbrc, MT(MOD_LALT | MOD_RCTL, US_RBRC)),

  // mixed combos
  [COMBO_CAPS_WORD] = COMBO(combo_caps_word, CW_TOGG)
};

uint8_t combo_ref_from_layer(uint8_t layer){
  switch (layer){
    case L_COLMK: return L_QWRTY;
    default: return layer;
  }
}

bool combo_should_trigger(uint16_t combo_index, combo_t *combo, uint16_t keycode, keyrecord_t *record) {
  switch (combo_index) {
    case COMBO_LBRC:
    case COMBO_LPRN:
      return !lower_pressed;

    case COMBO_RPRN:
    case COMBO_RBRC:
      return !raise_pressed;

    default:
      return true;
  }
}

/**************
 * TAP DANCES *
 **************/

enum {
  TD_BOOT_,
  TD_RBT_,
  TD_QWRTY_,
  TD_COLMK_
};

void td_boot_fn(tap_dance_state_t *state, void *user_data) {
  if (state->count == 2) {
    reset_keyboard();
  }
}

void td_reset_fn(tap_dance_state_t *state, void *user_data) {
  if (state->count == 2) {
    soft_reset_keyboard();
  }
}

void td_qwrty_fn(tap_dance_state_t *state, void *user_data) {
  if (state->count == 2) {
    set_single_persistent_default_layer(L_QWRTY);
  }
}

void td_colmk_fn(tap_dance_state_t *state, void *user_data) {
  if (state->count == 2) {
    set_single_persistent_default_layer(L_COLMK);
  }
}

tap_dance_action_t tap_dance_actions[] = {
  [TD_BOOT_] = ACTION_TAP_DANCE_FN(td_boot_fn),
  [TD_RBT_] = ACTION_TAP_DANCE_FN(td_reset_fn),
  [TD_QWRTY_] = ACTION_TAP_DANCE_FN(td_qwrty_fn),
  [TD_COLMK_] = ACTION_TAP_DANCE_FN(td_colmk_fn)
};

#define TD_BOOT TD(TD_BOOT_)
#define TD_RBT TD(TD_RBT_)
#define TD_QWRTY TD(TD_QWRTY_)
#define TD_COLMK TD(TD_COLMK_)

/**********
 * KEYMAP *
 **********/

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

  [L_QWRTY] = LAYOUT_wrapper(
    //                                                                                                     ╭──────╮
                                                                                                             X_NR
    // ╭──────╮   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ├──────┤   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ╭──────╮
         X_LT       US_Q,            US_W,            US_E,            US_R,            US_T,                X_CT       US_Z,            US_U,            US_I,            US_O,            US_P,                X_RT
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LM       LGUI_T(US_A),    LALT_T(US_S),    LCTL_T(US_D),    LSFT_T(US_F),    US_G,                X_CM       US_H,            RSFT_T(US_J),    RCTL_T(US_K),    LALT_T(US_L),    RGUI_T(US_SCLN),     X_RM
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LB       RALT_T(US_Y),    US_X,            US_C,            US_V,            US_B,                X_CB       US_N,            US_M,            US_COMM,         US_DOT,          RALT_T(US_SLSH),     X_RB
    // ├──────┤   ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯   ├──────┤
         X_LH                                                          KC_LOWER,        LT_MEDIA_SPC,        X_CH       LT_FUN_ENT,      KC_RAISE                                                                X_RH
    // ╰──────╯                                                      ╰────────────────┴────────────────╯   ╰──────╯   ╰────────────────┴────────────────╯                                                      ╰──────╯

  ),

  [L_COLMK] = LAYOUT_wrapper(

    //                                                                                                     ╭──────╮
                                                                                                             X_NR
    // ╭──────╮   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ├──────┤   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ╭──────╮
         X_LT       US_Q,            US_W,            US_F,            US_P,            US_B,                X_CT       US_J,            US_L,            US_U,            US_Z,            US_SCLN,             X_RT
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LM       LGUI_T(US_A),    LALT_T(US_R),    LCTL_T(US_S),    LSFT_T(US_T),    US_G,                X_CM       US_M,            RSFT_T(US_N),    RCTL_T(US_E),    LALT_T(US_I),    RGUI_T(US_O),        X_RM
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LB       RALT_T(US_Y),    US_X,            US_C,            US_D,            US_V,                X_CB       US_K,            US_H,            US_COMM,         US_DOT,          RALT_T(US_SLSH),     X_RB
    // ├──────┤   ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯   ├──────┤
         X_LH                                                          KC_LOWER,        LT_MEDIA_SPC,        X_CH       LT_FUN_ENT,      KC_RAISE                                                                X_RH
    // ╰──────╯                                                      ╰────────────────┴────────────────╯   ╰──────╯   ╰────────────────┴────────────────╯                                                      ╰──────╯

  ),

  [L_NAV] = LAYOUT_wrapper(

    //                                                                                                     ╭──────╮
                                                                                                             X_NR
    // ╭──────╮   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ├──────┤   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ╭──────╮
         X_LT       TD_BOOT,         TD_RBT,          TD_QWRTY,        TD_COLMK,        _______,             X_CT       KC_ESC,          KC_HOME,         KC_UP,           KC_END,          KC_BSPC,             X_RT
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LM       OSM(MOD_LGUI),   OSM(MOD_LALT),   OSM(MOD_LCTL),   OSM(MOD_LSFT),   _______,             X_CM       KC_TAB,          KC_LEFT,         KC_DOWN,         KC_RGHT,         KC_DEL,              X_RM
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LB       TG(L_MOUSE),     _______,         _______,         _______,         _______,             X_CB       C(US_Z),         C(US_V),         C(US_C),         C(US_X),         C(US_Y),             X_RB
    // ├──────┤   ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯   ├──────┤
         X_LH                                                          _______,         _______,             X_CH       _______,         _______                                                                 X_RH
    // ╰──────╯                                                      ╰────────────────┴────────────────╯   ╰──────╯   ╰────────────────┴────────────────╯                                                      ╰──────╯

  ),

  [L_MEDIA] = LAYOUT_wrapper(

    //                                                                                                     ╭──────╮
                                                                                                             X_NR
    // ╭──────╮   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ├──────┤   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ╭──────╮
         X_LT       TD_BOOT,         TD_RBT,          TD_QWRTY,        TD_COLMK,        _______,             X_CT       RGB_TOG,         _______,         KC_VOLU,         _______,         _______,             X_RT
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LM       OSM(MOD_LGUI),   OSM(MOD_LALT),   OSM(MOD_LCTL),   OSM(MOD_LSFT),   _______,             X_CM       _______,         KC_MPRV,         KC_VOLD,         KC_MNXT,         _______,             X_RM
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LB       TG(L_MOUSE),     _______,         _______,         _______,         _______,             X_CB       RGB_MOD,         RGB_HUI,         RGB_SAI,         RGB_VAI,         RGB_SPI,             X_RB
    // ├──────┤   ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯   ├──────┤
         X_LH                                                          _______,         _______,             X_CH       KC_MPLY,         KC_MUTE                                                                 X_RH
    // ╰──────╯                                                      ╰────────────────┴────────────────╯   ╰──────╯   ╰────────────────┴────────────────╯                                                      ╰──────╯

  ),

  [L_NUM] = LAYOUT_wrapper(

    //                                                                                                     ╭──────╮
                                                                                                             X_NR
    // ╭──────╮   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ├──────┤   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ╭──────╮
         X_LT       US_MINS,         US_7,            US_8,            US_9,            US_EQL,              X_CT       _______,         TD_COLMK,        TD_QWRTY,        TD_RBT,          TD_BOOT,             X_RT
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LM       US_LBRC,         US_4,            US_5,            US_6,            US_RBRC,             X_CM       _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),       X_RM
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LB       US_BSLS,         US_1,            US_2,            US_3,            US_QUOT,             X_CB       _______,         _______,         _______,         _______,         TG(L_MOUSE),         X_RB
    // ├──────┤   ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯   ├──────┤
         X_LH                                                          US_0,            _______,             X_CH       _______,         _______                                                                 X_RH
    // ╰──────╯                                                      ╰────────────────┴────────────────╯   ╰──────╯   ╰────────────────┴────────────────╯                                                      ╰──────╯

  ),

  [L_SYM] = LAYOUT_wrapper(

    //                                                                                                     ╭──────╮
                                                                                                             X_NR
    // ╭──────╮   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ├──────┤   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ╭──────╮
         X_LT       US_UNDS,         US_AMPR,         US_ASTR,         US_LPRN,         US_PLUS,             X_CT       _______,         TD_COLMK,        TD_QWRTY,        TD_RBT,          TD_BOOT,             X_RT
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LM       US_LCBR,         US_DLR,          US_PERC,         US_CIRC,         US_RCBR,             X_CM       _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),       X_RM
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LB       US_PIPE,         US_EXLM,         US_AT,           US_HASH,         US_DQUO,             X_CB       _______,         _______,         _______,         _______,         TG(L_MOUSE),         X_RB
    // ├──────┤   ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯   ├──────┤
         X_LH                                                          US_RPRN,         _______,             X_CH       _______,         _______                                                                 X_RH
    // ╰──────╯                                                      ╰────────────────┴────────────────╯   ╰──────╯   ╰────────────────┴────────────────╯                                                      ╰──────╯

  ),

  [L_FUN] = LAYOUT_wrapper(

    //                                                                                                     ╭──────╮
                                                                                                             X_NR
    // ╭──────╮   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ├──────┤   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ╭──────╮
         X_LT       KC_F12,          KC_F7,           KC_F8,           KC_F9,           _______,             X_CT       _______,         TD_COLMK,        TD_QWRTY,        TD_RBT,          TD_BOOT,             X_RT
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LM       KC_F11,          KC_F4,           KC_F5,           KC_F6,           _______,             X_CM       _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),       X_RM
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LB       KC_F10,          KC_F1,           KC_F2,           KC_F3,           _______,             X_CB       _______,         _______,         _______,         _______,         TG(L_MOUSE),         X_RB
    // ├──────┤   ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯   ├──────┤
         X_LH                                                          _______,         _______,             X_CH       _______,         _______                                                                 X_RH
    // ╰──────╯                                                      ╰────────────────┴────────────────╯   ╰──────╯   ╰────────────────┴────────────────╯                                                      ╰──────╯

  ),

  [L_MOUSE] = LAYOUT_wrapper(

    //                                                                                                     ╭──────╮
                                                                                                             X_NR
    // ╭──────╮   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ├──────┤   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ╭──────╮
         X_LT       TG(L_MOUSE),     DRGSCRL,         SNIPING,         _______,         _______,             X_CT       _______,         _______,         SNIPING,         DRGSCRL,         TG(L_MOUSE),         X_RT
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LM       OSM(MOD_LGUI),   OSM(MOD_LALT),   OSM(MOD_LCTL),   OSM(MOD_LSFT),   _______,             X_CM       _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),       X_RM
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LB       C(US_Y),         C(US_X),         C(US_C),         C(US_V),         C(US_Z),             X_CB       C(US_Z),         C(US_V),         C(US_C),         C(US_X),         C(US_Y),             X_RB
    // ├──────┤   ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯   ├──────┤
         X_LH                                                          KC_BTN2,         KC_BTN1,             X_CH       KC_BTN1,         KC_BTN2                                                                 X_RH
    // ╰──────╯                                                      ╰────────────────┴────────────────╯   ╰──────╯   ╰────────────────┴────────────────╯                                                      ╰──────╯

  )
};
