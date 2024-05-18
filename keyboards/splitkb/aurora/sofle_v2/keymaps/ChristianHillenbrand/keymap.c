#include QMK_KEYBOARD_H

#include "keymap_us_international.h"

enum layers {
  L_BASE = 0,
  L_MOUSE_L,
  L_MOUSE_R,
  L_NAV,
  L_NUM,
  L_FUN
};

/*************
 * POWER LED *
 *************/

void keyboard_pre_init_user(void) {
  setPinOutput(24);
  writePinHigh(24);
}

/*************
 * CAPS WORD *
 *************/

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
    case US_SS:
    case US_UNDS:
    case KC_BSPC:
    case KC_DEL:
      return true;

    default:
      return false;
  }
}

void caps_word_set_user(bool active) {
  if (active) {
    writePinLow(24);
  } else {
    writePinHigh(24);
  }
}

/*************
 * TAP HOLDS *
 *************/ 

enum custom_keycodes {
  US_A_AE_ = SAFE_RANGE,
  US_O_OE_,
  US_U_UE_,
  US_S_SS_,
  US_E_EURO_,

  MT_SFT_MOUSE_L_,
  MT_SFT_MOUSE_R_,

  LT_NAV_SFT_ ,
  LT_FUN_SFT_,
};

#define US_A_AE LT(0, US_A_AE_)
#define US_O_OE LT(0, US_O_OE_)
#define US_U_UE LT(0, US_U_UE_)
#define US_S_SS LT(0, US_S_SS_)
#define US_E_EURO LT(0, US_E_EURO_)

#define MT_SFT_MOUSE_L LT(0, MT_SFT_MOUSE_L_)
#define MT_SFT_MOUSE_R LT(0, MT_SFT_MOUSE_R_)

#define LT_NAV_SFT LT(0, LT_NAV_SFT_)
#define LT_NUM_ENT LT(L_NUM, KC_ENT)
#define LT_FUN_SFT LT(0, LT_FUN_SFT_)

uint16_t fast_tap_hold_keycode = 0;
keyrecord_t fast_tap_hold_record;

uint16_t fast_tap_hold_timer = 0;
bool fast_tap_hold_pressed = false;

bool fast_tap_hold(uint16_t tap_keycode, uint16_t hold_keycode, keyrecord_t* record)
{
  if (record->event.pressed) {
    uint8_t mods = get_mods();
    uint8_t oneshot_mods = get_oneshot_mods();

    process_caps_word(tap_keycode, record);
    tap_code16(tap_keycode);

    // only decide for hold if no other mod then shift is active
    if (((mods | oneshot_mods) & ~MOD_MASK_SHIFT) == 0) {
      fast_tap_hold_keycode = hold_keycode;
      if (oneshot_mods & MOD_MASK_SHIFT) {
        fast_tap_hold_keycode = S(fast_tap_hold_keycode); 
      }
      fast_tap_hold_record = *record;

      fast_tap_hold_timer = record->event.time + TAPPING_TERM;
      fast_tap_hold_pressed = true;
    }
  } else {
    fast_tap_hold_pressed = false;
  }

  return false;
}

bool layer_shift(uint16_t layer, keyrecord_t* record)
{
  if (record->event.pressed) {
    if (record->tap.count) {
      if (is_caps_word_on()) {
        caps_word_off();
      } else if (get_oneshot_mods() & MOD_MASK_SHIFT) {
        del_oneshot_mods(MOD_MASK_SHIFT);
        caps_word_toggle();
      } else {
        add_oneshot_mods(MOD_MASK_SHIFT);
      }
    } else if (get_oneshot_mods() & MOD_MASK_SHIFT){
      del_oneshot_mods(MOD_MASK_SHIFT);
      register_mods(MOD_BIT(KC_LSFT));
    } else {
      layer_on(layer);
    }
  } else if (!record->tap.count) {
    if (layer_state_is(layer)) {
      layer_off(layer);
    } else {
      unregister_mods(MOD_BIT(KC_LSFT));
    }
  }

  return false;
}

bool process_record_user(uint16_t keycode, keyrecord_t* record) {
  switch (keycode) {
    case US_A_AE:
      return fast_tap_hold(US_A, US_ADIA, record);

    case US_O_OE:
      return fast_tap_hold(US_O, US_ODIA, record);

    case US_U_UE:
      return fast_tap_hold(US_U, US_UDIA, record);

    case US_S_SS:
      return fast_tap_hold(US_S, US_SS, record);

    case US_E_EURO:
      return fast_tap_hold(US_E, US_EURO, record);

    case MT_SFT_MOUSE_L:
      if (record->event.pressed) {
        if (record->tap.count) {
          layer_invert(L_MOUSE_L);
        } else {
          register_mods(MOD_BIT(KC_LSFT));
        }
      } else if (!record->tap.count) {
        unregister_mods(MOD_BIT(KC_LSFT));
      }
      return false;

    case MT_SFT_MOUSE_R:
      if (record->event.pressed) {
        if (record->tap.count) {
          layer_invert(L_MOUSE_R);
        } else {
          register_mods(MOD_BIT(KC_LSFT));
        }
      } else if (!record->tap.count) {
        unregister_mods(MOD_BIT(KC_LSFT));
      }
      return false;

    case LT_NAV_SFT:
      return layer_shift(L_NAV, record);

    case LT_FUN_SFT:
      return layer_shift(L_FUN, record);
    
    default: 
      fast_tap_hold_pressed = false;
      return true;
  }
}

void matrix_scan_user(void) {
  if (fast_tap_hold_pressed && timer_expired(timer_read(), fast_tap_hold_timer)) {
    tap_code16(KC_BSPC);
    process_caps_word(fast_tap_hold_keycode, &fast_tap_hold_record);
    tap_code16(fast_tap_hold_keycode);
    fast_tap_hold_pressed = false;
  }
}

/*********************
 * TAP HOLD SETTINGS *
 *********************/

uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case US_A_AE:
    case US_O_OE:
    case US_U_UE:
    case US_S_SS:
    case US_E_EURO:
      return 0;

    default:
      return TAPPING_TERM;
  }
}

bool get_hold_on_other_key_press(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case MT_SFT_MOUSE_L:
    case MT_SFT_MOUSE_R:
      return true;

    default:
      return false;
  }
}

bool get_permissive_hold(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case LT_NAV_SFT:
    case LT_NUM_ENT:
    case LT_FUN_SFT:
      return true;

    default:
      return false;
  }
}

/**************
 * TAP DANCES *
 **************/

void td_fn_boot(tap_dance_state_t *state, void *user_data) {
  if (state->count == 2) {
    reset_keyboard();
  }
}

enum tap_dances{
  TD_BOOT_ = 0
};

tap_dance_action_t tap_dance_actions[] = { 
  [TD_BOOT_] = ACTION_TAP_DANCE_FN(td_fn_boot)
};

#define TD_BOOT TD(TD_BOOT_)

/*****************
 * KEY OVERRIDES *
 *****************/

const key_override_t shift_ss = ko_make_basic(MOD_MASK_SHIFT, US_SS, US_SS);
const key_override_t shift_euro = ko_make_basic(MOD_MASK_SHIFT, US_EURO, US_EURO);
const key_override_t shift_bspc = ko_make_basic(MOD_MASK_SHIFT, KC_BSPC, KC_DEL);
const key_override_t shift_lprn = ko_make_basic(MOD_MASK_SHIFT, US_LPRN, US_LABK);
const key_override_t shift_rprn = ko_make_basic(MOD_MASK_SHIFT, US_RPRN, US_RABK);

const key_override_t **key_overrides = (const key_override_t *[]){
  &shift_ss,
  &shift_euro,
  &shift_bspc,
  &shift_lprn,
  &shift_rprn,
  NULL
};

/**********
 * COMBOS *
 **********/

const uint16_t PROGMEM capsword_combo[] = {LT_NAV_SFT, LT_FUN_SFT, COMBO_END};

const uint16_t PROGMEM esc_combo[] = {US_W, US_E_EURO, COMBO_END};
const uint16_t PROGMEM tab_combo[]  = {US_E_EURO, US_R, COMBO_END};
const uint16_t PROGMEM lbrc_combo[] = {US_S_SS, US_D, COMBO_END};
const uint16_t PROGMEM lprn_combo[] = {US_D, US_F, COMBO_END};

const uint16_t PROGMEM bspc_combo[] = {US_U_UE, US_I, COMBO_END};
const uint16_t PROGMEM del_combo[]  = {US_I, US_O_OE, COMBO_END};
const uint16_t PROGMEM rprn_combo[] = {US_J, US_K, COMBO_END};
const uint16_t PROGMEM rbrc_combo[] = {US_K, US_L, COMBO_END};

combo_t key_combos[] = {
  COMBO(capsword_combo, CW_TOGG),

  COMBO(esc_combo, KC_ESC),
  COMBO(tab_combo, KC_TAB),
  COMBO(lbrc_combo, US_LBRC),
  COMBO(lprn_combo, US_LPRN),

  COMBO(bspc_combo, KC_BSPC),
  COMBO(del_combo, KC_DEL),
  COMBO(rprn_combo, US_RPRN),
  COMBO(rbrc_combo, US_RBRC),
};

/**********
 * KEYMAP *
 **********/

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

  [L_BASE] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         US_1,            US_2,            US_3,            US_4,            US_5,            US_6,                                                 US_7,            US_8,            US_9,            US_0,            US_MINS,         US_EQL,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         KC_ESC,          US_Q,            US_W,            US_E_EURO,       US_R,            US_T,                                                 US_Z,            US_U_UE,         US_I,            US_O_OE,         US_P,            KC_BSPC, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         KC_TAB,          US_A_AE,         US_S_SS,         US_D,            US_F,            US_G,                                                 US_H,            US_J,            US_K,            US_L,            US_SCLN,         US_ACUT, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         MT_SFT_MOUSE_L,  US_Y,            US_X,            US_C,            US_V,            US_B,            XXXXXXX,            KC_MUTE,         US_N,            US_M,            US_COMM,         US_DOT,          US_SLSH,         MT_SFT_MOUSE_R, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           KC_LGUI,         OSM(MOD_LALT),   OSM(MOD_LCTL),   LT_NAV_SFT,      KC_SPC,             LT_NUM_ENT,      LT_FUN_SFT,      OSM(MOD_RCTL),   OSM(MOD_LALT),   KC_RGUI
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_MOUSE_L] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         _______,         KC_BTN2,         KC_WH_U,         KC_BTN1,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         _______,         KC_WH_L,         KC_WH_D,         KC_WH_R,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         C(US_Y),         C(US_X),         C(US_C),         C(US_V),         C(US_Z),         _______,            _______,         _______,         _______,         _______,         _______,         _______,         _______, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           KC_BTN5,         KC_BTN4,         KC_BTN3,         KC_BTN2,         KC_BTN1,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_MOUSE_R] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         KC_BTN1,         KC_WH_U,         KC_BTN2,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         KC_WH_L,         KC_WH_D,         KC_WH_R,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         _______,         _______,         _______,         _______,         _______,         _______,            _______,         C(US_Z),         C(US_V),         C(US_C),         C(US_X),         C(US_Y),         _______, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            KC_BTN1,         KC_BTN2,         KC_BTN3,         KC_BTN4,         KC_BTN5
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),
  
  [L_NAV] = LAYOUT(
    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              RGB_TOG,         RGB_MOD,         RGB_HUI,         RGB_SAI,         RGB_VAI,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         TD_BOOT,         _______,         _______,         _______,         _______,                                              _______,         KC_HOME,         KC_UP,           KC_END,          _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         KC_LGUI,        OSM(MOD_LALT),   OSM(MOD_LCTL),   OSM(MOD_LSFT),   _______,                                              _______,         KC_LEFT,         KC_DOWN,         KC_RGHT,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         _______,         _______,         _______,         _______,         _______,         _______,            _______,         C(US_Z),         C(US_V),         C(US_C),         C(US_X),         C(US_Y),         _______, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_NUM] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         KC_MPLY,         KC_MSTP,         KC_MPRV,         KC_MNXT,         KC_MUTE,                                              _______,         _______,         _______,         _______,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         _______,         US_7,            US_8,            US_9,            _______,                                              _______,         _______,         _______,         _______,         TD_BOOT,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         _______,         US_4,            US_5,            US_6,            _______,                                              _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   KC_RGUI,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         _______,         US_1,            US_2,            US_3,            _______,         _______,            _______,         _______,         _______,         _______,         _______,         _______,         _______, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         US_COMM,         US_DOT,          US_0,               _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  ),

  [L_FUN] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         KC_MPLY,         KC_MSTP,         KC_MPRV,         KC_MNXT,         KC_MUTE,                                               _______,         _______,         _______,         _______,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         KC_F12,          KC_F7,           KC_F8,           KC_F9,           _______,                                              _______,         _______,         _______,         _______,         TD_BOOT,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         KC_F11,          KC_F4,           KC_F5,           KC_F6,           _______,                                              _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   KC_RGUI,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         KC_F10,          KC_F1,           KC_F2,           KC_F3,           _______,         _______,            _______,         _______,         _______,         _______,         _______,         _______,         _______, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  )
};

/***********
 * ENCODER *
 ***********/

bool encoder_update_user(uint8_t index, bool clockwise) {
  if (index == 0) {
    if (clockwise) {
      tap_code(KC_PGDN);
    } else {
      tap_code(KC_PGUP);
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

    for (uint8_t index = led_min; index < led_max; ++index) {
      if (g_led_config.flags[index] & LED_FLAG_UNDERGLOW) {
        rgb_matrix_set_color(index, RGB_BLACK);
      }
    }
       
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
