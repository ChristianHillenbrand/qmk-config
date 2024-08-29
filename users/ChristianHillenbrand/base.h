#pragma once

#include QMK_KEYBOARD_H

#include "extra_keys.h"
#include "funcs.h"
#include "layers.h"

#include <keymap_us_extended.h>

#define IDLE_MS 100

#define LAYOUT_wrapper(...) LAYOUT(__VA_ARGS__)

#define GET_TAP_KEYCODE(kc) ((kc)&0xFF)
#define IS_TYPING(kc) ( \
  ((kc) <= KC_Z || (kc) == KC_SPC) && \
  (last_input_activity_elapsed() < IDLE_MS))

/**********************
 * CAPS WORD SETTINGS *
 **********************/

bool caps_word_press_user(uint16_t keycode) {
  switch (keycode) {
    case KC_A ... KC_Z:
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

#ifndef DRGSCRL
  #define DRGSCRL KC_TRNS
#endif

#ifndef SNIPING
  #define SNIPING KC_TRNS
#endif

#define HRML(A, B, C, D) MT(MOD_LGUI, A), MT(MOD_LALT, B), MT(MOD_LCTL, C), MT(MOD_LSFT, D)
#define HRMR(A, B, C, D) MT(MOD_RSFT, A), MT(MOD_RCTL, B), MT(MOD_LALT, C), MT(MOD_RGUI, D)

#define MT_RALT_Y MT(MOD_RALT, US_Y)
#define MT_RALT_SLSH MT(MOD_RALT, US_SLSH)

#define KC_LOWER LT(0, KC_LOWER_)
#define KC_RAISE LT(0, KC_RAISE_)

#define LT_FUN_SPC LT(L_FUN, KC_SPC)
#define LT_SYM_ENT LT(L_SYM, KC_ENT)

bool lower_pressed = false;
bool raise_pressed = false;

bool pre_process_record_user(uint16_t keycode, keyrecord_t *record) {
  static uint16_t prev_keycode;
  static bool is_pressed[UINT8_MAX];

  const uint8_t tap_keycode = GET_TAP_KEYCODE(keycode);

  switch (keycode) {
    case KC_LOWER:
      lower_pressed = record->event.pressed;
      break;

    case KC_RAISE:
      raise_pressed = record->event.pressed;
      break;

    case LT_FUN_SPC:
    {
      if (record->event.pressed && IS_TYPING(prev_keycode)) {
        record->keycode = tap_keycode;
        is_pressed[tap_keycode] = true;
      }
      else if (!record->event.pressed && is_pressed[tap_keycode]) {
        record->keycode = tap_keycode;
        is_pressed[tap_keycode] = false;
      }
    }
    break;

    default:
      break;
  }

  if (record->event.pressed) {
    prev_keycode = tap_keycode;
  }

  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t* record) {
  if (!process_record_user_special(keycode, record))
    return false;

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

    default:
      break;
  }

  return true;
}

/*********************
 * TAP HOLD SETTINGS *
 *********************/

bool get_permissive_hold(uint16_t keycode, keyrecord_t *record) {
  return IS_QK_MOD_TAP(keycode) || IS_QK_LAYER_TAP(keycode);
}

/*****************
 * KEY OVERRIDES *
 *****************/

const key_override_t shift_9  = ko_make_basic(MOD_MASK_SHIFT, US_9, US_TILD);
const key_override_t shift_lprn = ko_make_basic(MOD_MASK_SHIFT, US_LPRN, US_LABK);
const key_override_t shift_rprn = ko_make_basic(MOD_MASK_SHIFT, KC_RPRN, US_RABK);

const key_override_t ralt_a = ko_make_basic(MOD_BIT_RALT, US_A, US_ADIA);
const key_override_t ralt_o = ko_make_basic(MOD_BIT_RALT, US_O, US_ODIA);
const key_override_t ralt_u = ko_make_basic(MOD_BIT_RALT, US_U, US_UDIA);
const key_override_t ralt_s = ko_make_basic(MOD_BIT_RALT, US_S, US_SS);
const key_override_t ralt_e = ko_make_basic(MOD_BIT_RALT, US_E, US_EURO);

const key_override_t** key_overrides = (const key_override_t* []){
  &shift_9,
  &shift_lprn,
  &shift_rprn,

  &ralt_a,
  &ralt_o,
  &ralt_u,
  &ralt_s,
  &ralt_e,
  NULL
};

/**********
 * COMBOS *
 **********/

enum combos {
  COMBO_LBRC,
  COMBO_LPRN,

  COMBO_BSPC,
  COMBO_DEL,
  COMBO_RPRN,
  COMBO_RBRC,

  COMBO_CAPS_WORD,
  COMBO_MEDIA
};

// left half combos
const uint16_t PROGMEM combo_lbrc[] = {US_S, US_D, COMBO_END};
const uint16_t PROGMEM combo_lprn[] = {US_D, US_F, COMBO_END};

// right half combos
const uint16_t PROGMEM combo_rprn[] = {US_J, US_K, COMBO_END};
const uint16_t PROGMEM combo_rbrc[] = {US_K, US_L, COMBO_END};

// mixed combos
const uint16_t PROGMEM combo_caps_word[] = {US_F, US_J, COMBO_END};
const uint16_t PROGMEM combo_media[] = {KC_LOWER, KC_RAISE, COMBO_END};

combo_t key_combos[] = {
  // left half combos
  [COMBO_LBRC] = COMBO(combo_lbrc, US_LBRC),
  [COMBO_LPRN] = COMBO(combo_lprn, US_LPRN),

  // right half combos
  [COMBO_RPRN] = COMBO(combo_rprn, US_RPRN),
  [COMBO_RBRC] = COMBO(combo_rbrc, US_RBRC),

  // mixed combos
  [COMBO_CAPS_WORD] = COMBO(combo_caps_word, CW_TOGG),
  [COMBO_MEDIA] = COMBO(combo_media, MO(L_MEDIA))
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
         X_LM       HRML(US_A,       US_S,            US_D,            US_F),           US_G,                X_CM       US_H,            HRMR(US_J,       US_K,            US_L,            US_SCLN),            X_RM
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LB       MT_RALT_Y,       US_X,            US_C,            US_V,            US_B,                X_CB       US_N,            US_M,            US_COMM,         US_DOT,          MT_RALT_SLSH,        X_RB
    // ├──────┤   ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯   ├──────┤
         X_LH                                                          KC_LOWER,        LT_FUN_SPC,          X_CH       LT_SYM_ENT,      KC_RAISE                                                                X_RH
    // ╰──────╯                                                      ╰────────────────┴────────────────╯   ╰──────╯   ╰────────────────┴────────────────╯                                                      ╰──────╯

  ),

  [L_COLMK] = LAYOUT_wrapper(

    //                                                                                                     ╭──────╮
                                                                                                             X_NR
    // ╭──────╮   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ├──────┤   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ╭──────╮
         X_LT       US_Q,            US_W,            US_F,            US_P,            US_B,                X_CT       US_J,            US_L,            US_U,            US_Z,            US_SCLN,             X_RT
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LM       HRML(US_A,       US_R,            US_S,            US_T),           US_G,                X_CM       US_M,            HRMR(US_N,       US_E,            US_I,            US_O),               X_RM
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LB       MT_RALT_Y,       US_X,            US_C,            US_D,            US_V,                X_CB       US_K,            US_H,            US_COMM,         US_DOT,          MT_RALT_SLSH,        X_RB
    // ├──────┤   ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯   ├──────┤
         X_LH                                                          KC_LOWER,        LT_FUN_SPC,          X_CH       LT_SYM_ENT,      KC_RAISE                                                                X_RH
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

  [L_FUN] = LAYOUT_wrapper(

    //                                                                                                     ╭──────╮
                                                                                                             X_NR
    // ╭──────╮   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ├──────┤   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ╭──────╮
         X_LT       TD_BOOT,         TD_RBT,          TD_QWRTY,        TD_COLMK,        _______,             X_CT       _______,         KC_F7,           KC_F8,           KC_F9,           KC_F12,              X_RT
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LM       OSM(MOD_LGUI),   OSM(MOD_LALT),   OSM(MOD_LCTL),   OSM(MOD_LSFT),   _______,             X_CM       _______,         KC_F4,           KC_F5,           KC_F6,           KC_F11,              X_RM
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LB       TG(L_MOUSE),     _______,         _______,         _______,         _______,             X_CB       _______,         KC_F1,           KC_F2,           KC_F3,           KC_F10,              X_RB
    // ├──────┤   ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯   ├──────┤
         X_LH                                                          _______,         _______,             X_CH       _______,         _______                                                                 X_RH
    // ╰──────╯                                                      ╰────────────────┴────────────────╯   ╰──────╯   ╰────────────────┴────────────────╯                                                      ╰──────╯

  ),

  [L_NUM] = LAYOUT_wrapper(

    //                                                                                                     ╭──────╮
                                                                                                             X_NR
    // ╭──────╮   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ├──────┤   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ╭──────╮
         X_LT       US_LBRC,         US_7,            US_8,            US_9,            US_RBRC,             X_CT       _______,         TD_COLMK,        TD_QWRTY,        TD_RBT,          TD_BOOT,             X_RT
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LM       US_MINS,         US_4,            US_5,            US_6,            US_EQL,              X_CM       _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),       X_RM
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
         X_LT       US_LCBR,         US_AMPR,         US_ASTR,         US_TILD,         US_RCBR,             X_CT       _______,         TD_COLMK,        TD_QWRTY,        TD_RBT,          TD_BOOT,             X_RT
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LM       US_UNDS,         US_DLR,          US_PERC,         US_CIRC,         US_PLUS,             X_CM       _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),       X_RM
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LB       US_PIPE,         US_EXLM,         US_AT,           US_HASH,         US_DQUO,             X_CB       _______,         _______,         _______,         _______,         TG(L_MOUSE),         X_RB
    // ├──────┤   ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯   ├──────┤
         X_LH                                                          _______,         _______,             X_CH       _______,         _______                                                                 X_RH
    // ╰──────╯                                                      ╰────────────────┴────────────────╯   ╰──────╯   ╰────────────────┴────────────────╯                                                      ╰──────╯

  ),

  [L_MEDIA] = LAYOUT_wrapper(

    //                                                                                                     ╭──────╮
                                                                                                             X_NR
    // ╭──────╮   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ├──────┤   ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮   ╭──────╮
         X_LT       RGB_TOG,         _______,         _______,         _______,         _______,             X_CT       _______,         _______,         _______,         _______,         _______,             X_RT
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LM       RGB_SPI,         RGB_VAI,         RGB_SAI,         RGB_HUI,         RGB_MOD,             X_CM       _______,         KC_VOLD,         KC_MUTE,         KC_VOLU,         _______,             X_RM
    // ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤   ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤   ├──────┤
         X_LB       RGB_SPD,         RGB_VAD,         RGB_SAD,         RGB_HUD,         RGB_RMOD,            X_CB       _______,         KC_MPRV,         KC_MPLY,         KC_MNXT,         _______,             X_RB
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
