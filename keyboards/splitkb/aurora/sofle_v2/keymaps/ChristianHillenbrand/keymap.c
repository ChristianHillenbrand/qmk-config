#include "config.h"
#include QMK_KEYBOARD_H

#include "keymap_us_extended.h"
#include "transactions.h"

#include "layers.h"
#include "oled.h"

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

/*******************
 * CUSTOM KEYCODES *
 *******************/

enum custom_keycodes {
  KC_LOWER_ = SAFE_RANGE,
  KC_RAISE_
};

#define MT_RALT_Y MT(MOD_RALT, US_Y)
#define MT_RALT_SLSH MT(MOD_RALT, US_SLSH)

#define KC_LOWER LT(0, KC_LOWER_)
#define KC_RAISE LT(0, KC_RAISE_)
#define LT_NUM_ENT LT(L_NUM, KC_ENT)

#define MOD_BIT_LSFT MOD_BIT(KC_LSFT)
#define MOD_BIT_RSFT MOD_BIT(KC_RSFT)

bool process_record_user(uint16_t keycode, keyrecord_t* record) {
  switch (keycode) {
    case KC_SPC:
      if (record->event.pressed) {
        trigger_jump();
        transaction_rpc_send(RPC_SPACE_PRESSED, 0, NULL);
      }
      return true;

    case KC_LOWER:
      if (record->tap.count) {
        // tap -> one shot shift
        if (record->event.pressed) {
          if (get_oneshot_mods() & MOD_BIT_LSFT) {
            del_oneshot_mods(MOD_BIT_LSFT);
          } else {
            add_oneshot_mods(MOD_BIT_LSFT);
          }
        }
        return false;
      } else{
        // hold -> different layers depending on tap
        static uint8_t layer = 0;
        if (record->event.pressed) {
          layer = (get_oneshot_mods() & MOD_BIT_LSFT) ? L_MEDIA : L_NAV;
          del_oneshot_mods(MOD_BIT_LSFT);
          layer_on(layer);
        } else {
          layer_off(layer);
        }
        return true;
      }

    case KC_RAISE:
      if (record->tap.count) {
        // tap -> one shot shift
        if (record->event.pressed) {
          if (get_oneshot_mods() & MOD_BIT_RSFT) {
            del_oneshot_mods(MOD_BIT_RSFT);
          } else {
            add_oneshot_mods(MOD_BIT_RSFT);
          }
        }
        return false;
      } else {
        // hold -> fun layer
        if (record->event.pressed) {
          del_oneshot_mods(MOD_BIT_RSFT);
          layer_on(L_FUN);
        } else {
          layer_off(L_FUN);
        }
        return true;
      }

    default:
      break;
  }

  return true;
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

/*****************
 * KEY OVERRIDES *
 *****************/

const key_override_t shift_esc  = ko_make_basic(MOD_MASK_SHIFT, KC_ESC, US_TILD);
const key_override_t shift_lprn = ko_make_basic(MOD_MASK_SHIFT, US_LPRN, US_LABK);
const key_override_t shift_rprn = ko_make_basic(MOD_MASK_SHIFT, KC_RPRN, US_RABK);

const key_override_t ralt_a = ko_make_basic(MOD_BIT_RALT, US_A, US_ADIA);
const key_override_t ralt_o = ko_make_basic(MOD_BIT_RALT, US_O, US_ODIA);
const key_override_t ralt_u = ko_make_basic(MOD_BIT_RALT, US_U, US_UDIA);
const key_override_t ralt_s = ko_make_basic(MOD_BIT_RALT, US_S, US_SS);
const key_override_t ralt_e = ko_make_basic(MOD_BIT_RALT, US_E, US_EURO);

const key_override_t **key_overrides = (const key_override_t *[]){
  &shift_esc,
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

// left half combos
const uint16_t PROGMEM combo_esc[] = {US_W, US_E, COMBO_END};
const uint16_t PROGMEM combo_tab[] = {US_E, US_R, COMBO_END};
const uint16_t PROGMEM combo_lbrc[] = {US_S, US_D, COMBO_END};
const uint16_t PROGMEM combo_lprn[] = {US_D, US_F, COMBO_END};

// right half combos
const uint16_t PROGMEM combo_bspc[] = {US_U, US_I, COMBO_END};
const uint16_t PROGMEM combo_del[] = {US_I, US_O, COMBO_END};
const uint16_t PROGMEM combo_rprn[] = {US_J, US_K, COMBO_END};
const uint16_t PROGMEM combo_rbrc[] = {US_K, US_L, COMBO_END};

// mixed combos
const uint16_t PROGMEM combo_gui[] = {US_F, US_J, COMBO_END};
const uint16_t PROGMEM combo_caps_word[] = {KC_LOWER, KC_RAISE, COMBO_END};

combo_t key_combos[] = {
  // left half combos
  COMBO(combo_esc, KC_ESC),
  COMBO(combo_tab, KC_TAB),
  COMBO(combo_lbrc, US_LBRC),
  COMBO(combo_lprn, US_LPRN),

  // right half combos
  COMBO(combo_bspc, KC_BSPC),
  COMBO(combo_del, KC_DEL),
  COMBO(combo_rprn, US_RPRN),
  COMBO(combo_rbrc, US_RBRC),

  // mixed combos
  COMBO(combo_gui, KC_LGUI),
  COMBO(combo_caps_word, CW_TOGG)
};

uint8_t combo_ref_from_layer(uint8_t layer){
  switch (layer){
    case L_COLMK: return L_QWRTY;
    default: return layer;
  }
}

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

/**********
 * KEYMAP *
 **********/

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

  [L_QWRTY] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         KC_ESC,          US_1,            US_2,            US_3,            US_4,            US_5,                                                 US_6,            US_7,            US_8,            US_9,            US_0,            KC_BSPC,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         KC_TAB,          US_Q,            US_W,            US_E,            US_R,            US_T,                                                 US_Z,            US_U,            US_I,            US_O,            US_P,            KC_DEL,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         CW_TOGG,         US_A,            US_S,            US_D,            US_F,            US_G,                                                 US_H,            US_J,            US_K,            US_L,            US_SCLN,         US_QUOT,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         KC_LSFT,         MT_RALT_Y,       US_X,            US_C,            US_V,            US_B,            XXXXXXX,            KC_MUTE,         US_N,            US_M,            US_COMM,         US_DOT,          MT_RALT_SLSH,    KC_RSFT,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           KC_LGUI,         KC_LALT,         KC_LCTL,         KC_LOWER,        KC_SPC,             LT_NUM_ENT,      KC_RAISE,        KC_RCTL,         KC_LALT,         KC_RGUI
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_COLMK] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         KC_ESC,          US_1,            US_2,            US_3,            US_4,            US_5,                                                 US_6,            US_7,            US_8,            US_9,            US_0,            KC_BSPC,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         KC_TAB,          US_Q,            US_W,            US_F,            US_P,            US_B,                                                 US_J,            US_L,            US_U,            US_Z,            US_SCLN,         KC_DEL,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         CW_TOGG,         US_A,            US_R,            US_S,            US_T,            US_G,                                                 US_M,            US_N,            US_E,            US_I,            US_O,            US_QUOT,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         KC_LSFT,         MT_RALT_Y,       US_X,            US_C,            US_D,            US_V,            XXXXXXX,            KC_MUTE,         US_K,            US_H,            US_COMM,         US_DOT,          MT_RALT_SLSH,    KC_RSFT,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           KC_LGUI,         KC_LALT,         KC_LCTL,         KC_LOWER,        KC_SPC,             LT_NUM_ENT,      KC_RAISE,        KC_RCTL,         KC_LALT,         KC_RGUI
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_NAV] = LAYOUT(
    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         TD_BOOT,         TD_RBT,          TD_QWRTY,        TD_COLMK,        _______,                                              KC_ESC,          KC_HOME,         KC_UP,           KC_END,          KC_BSPC,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         OSM(MOD_LGUI),   OSM(MOD_LALT),   OSM(MOD_LCTL),   OSM(MOD_LSFT),   _______,                                              KC_TAB,          KC_LEFT,         KC_DOWN,         KC_RGHT,         KC_DEL,          _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         TG(L_MOUSE),     _______,         _______,         _______,         _______,         _______,            _______,         C(US_Z),         C(US_V),         C(US_C),         C(US_X),         C(US_Y),         _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_MEDIA] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         TD_BOOT,         TD_RBT,          TD_QWRTY,        TD_COLMK,        _______,                                              RGB_TOG,         _______,         KC_VOLU,         _______,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         OSM(MOD_LGUI),   OSM(MOD_LALT),   OSM(MOD_LCTL),   OSM(MOD_LSFT),   _______,                                              _______,         KC_MPRV,         KC_VOLD,         KC_MNXT,         KC_MNXT,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         TG(L_MOUSE),     _______,         _______,         _______,         _______,         _______,            _______,         RGB_MOD,         RGB_HUI,         RGB_SAI,         RGB_VAI,         RGB_SPI,         _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            KC_MPLY,         KC_MSTP,         KC_MUTE,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  ),

  [L_NUM] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         US_DGRV,         US_7,            US_8,            US_9,            XXXXXXX,                                              _______,         TD_COLMK,        TD_QWRTY,        TD_RBT,          TD_BOOT,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         US_MINS,         US_4,            US_5,            US_6,            US_EQL,                                               _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),   _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         US_BSLS,         US_1,            US_2,            US_3,            US_QUOT,         _______,            _______,         _______,         _______,         _______,         _______,         TG(L_MOUSE),     _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         US_0,            _______,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  ),

  [L_FUN] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         KC_F12,          KC_F7,           KC_F8,           KC_F9,           _______,                                              _______,         TD_COLMK,        TD_QWRTY,        TD_RBT,          TD_BOOT,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         KC_F11,          KC_F4,           KC_F5,           KC_F6,           _______,                                              _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),   _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         KC_F10,          KC_F1,           KC_F2,           KC_F3,           _______,         _______,            _______,         _______,         _______,         _______,         _______,         TG(L_MOUSE),     _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  ),

  [L_MOUSE] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         _______,         KC_BTN3,         KC_BTN2,         KC_BTN1,         _______,                                              _______,         KC_BTN1,         KC_BTN2,         KC_BTN3,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         C(US_Y),         C(US_X),         C(US_C),         C(US_V),         C(US_Z),         _______,            _______,         C(US_Z),         C(US_V),         C(US_C),         C(US_X),         C(US_Y),         _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         TG(L_MOUSE),     _______,            _______,         TG(L_MOUSE),     _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  )
};

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

        if (index >= led_min && index < led_max && index != NO_LED &&
          keymap_key_to_keycode(highest_layer, (keypos_t){col,row}) == KC_TRNS) {
            rgb_matrix_set_color(index, RGB_BLACK);
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
