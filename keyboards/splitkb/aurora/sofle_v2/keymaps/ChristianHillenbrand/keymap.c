#include QMK_KEYBOARD_H

#include "keymap_german.h"

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
    case KC_A ... KC_Z:
    case DE_ADIA:
    case DE_ODIA:
    case DE_UDIA:
    case DE_MINS:
      add_weak_mods(MOD_BIT(KC_LSFT));
      return true;

    case CW_TOGG:
    case DE_1 ... DE_0:
    case DE_SS:
    case DE_UNDS:
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
  DE_A_AE_ = SAFE_RANGE,
  DE_O_OE_,
  DE_U_UE_,
  DE_S_SS_,
  DE_E_EURO_,

  MT_SFT_MOUSE_L_,
  MT_SFT_MOUSE_R_,

  LT_NAV_SFT_ ,
  LT_FUN_SFT_,
};

#define DE_A_AE LT(0, DE_A_AE_)
#define DE_O_OE LT(0, DE_O_OE_)
#define DE_U_UE LT(0, DE_U_UE_)
#define DE_S_SS LT(0, DE_S_SS_)
#define DE_E_EURO LT(0, DE_E_EURO_)

#define MT_SFT_MOUSE_L LT(0, MT_SFT_MOUSE_L_)
#define MT_SFT_MOUSE_R LT(0, MT_SFT_MOUSE_R_)

#define LT_NAV_SFT LT(0, LT_NAV_SFT_)
#define LT_NUM_ENT LT(L_NUM, KC_ENT)
#define LT_FUN_SFT LT(0, LT_FUN_SFT_)

uint8_t fast_tap_hold_mods = 0;
uint8_t fast_tap_hold_oneshot_mods = 0;
uint16_t fast_tap_hold_keycode = 0;
uint16_t fast_tap_hold_timer = 0;
bool fast_tap_hold_pressed = false;

void tap_code_caps_word(uint16_t keycode)
{
  if (is_caps_word_on() && !caps_word_press_user(keycode)) {
    caps_word_off();
  }
  tap_code16(keycode);
}

void fast_tap_hold(keyrecord_t* record, uint16_t tap_keycode, uint16_t hold_keycode)
{
  if (record->event.pressed) {
    fast_tap_hold_mods = get_mods(); 
    fast_tap_hold_oneshot_mods = get_oneshot_mods();

    tap_code_caps_word(tap_keycode);

    if ((fast_tap_hold_mods & ~MOD_MASK_SHIFT) == 0) {
      fast_tap_hold_keycode = hold_keycode;
      fast_tap_hold_timer = record->event.time + TAPPING_TERM;
      fast_tap_hold_pressed = true;
    }
  } else {
    fast_tap_hold_pressed = false;
  }
}

bool process_record_user(uint16_t keycode, keyrecord_t* record) {
  switch (keycode) {
    case DE_A_AE:
      fast_tap_hold(record, DE_A, DE_ADIA);
      return false;

    case DE_O_OE:
      fast_tap_hold(record, DE_O, DE_ODIA);
      return false;

    case DE_U_UE:
      fast_tap_hold(record, DE_U, DE_UDIA);
      return false;

    case DE_S_SS:
      fast_tap_hold(record, DE_S, DE_SS);
      fast_tap_hold_mods &= ~MOD_MASK_SHIFT;
      fast_tap_hold_oneshot_mods &= ~MOD_MASK_SHIFT;
      return false;

    case DE_E_EURO:
      fast_tap_hold(record, DE_E, DE_EURO);
      return false;

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
        } else {
          layer_on(L_NAV);
        }
      } else if (!record->tap.count) {
        layer_off(L_NAV);
      }
      return false;

    case LT_FUN_SFT:
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
        } else {
          layer_on(L_FUN);
        }
      } else if (!record->tap.count) {
        layer_off(L_FUN);
      }
      return false;
    
    default: 
      fast_tap_hold_pressed = false;
      return true;
  }
}

void matrix_scan_user(void) {
  if (fast_tap_hold_pressed && timer_expired(timer_read(), fast_tap_hold_timer)) {
    tap_code16(KC_BSPC);

    uint8_t cur_mods = get_mods();
    uint8_t cur_oneshot_mods = get_oneshot_mods();

    set_mods(fast_tap_hold_mods);
    set_oneshot_mods(fast_tap_hold_oneshot_mods);

    tap_code_caps_word(fast_tap_hold_keycode);

    set_mods(cur_mods);
    set_mods(cur_oneshot_mods);

    fast_tap_hold_pressed = false;
  }
}

/*********************
 * TAP HOLD SETTINGS *
 *********************/

uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case DE_A_AE:
    case DE_O_OE:
    case DE_U_UE:
    case DE_S_SS:
    case DE_E_EURO:
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

const key_override_t shift_bspc = ko_make_basic(MOD_MASK_SHIFT, KC_BSPC, DE_QUES);
const key_override_t ralt_bspc  = ko_make_basic(MOD_BIT(KC_RALT), KC_BSPC, DE_BSLS);
const key_override_t shift_lprn = ko_make_basic(MOD_MASK_SHIFT, LSFT(DE_8), DE_LABK);
const key_override_t shift_rprn = ko_make_basic(MOD_MASK_SHIFT, LSFT(DE_9), LSFT(DE_LABK));
const key_override_t shift_lbrc = ko_make_basic(MOD_MASK_SHIFT, DE_LBRC, DE_LCBR);
const key_override_t shift_rbrc = ko_make_basic(MOD_MASK_SHIFT, DE_RBRC, DE_RCBR);

const key_override_t **key_overrides = (const key_override_t *[]){
  &shift_bspc,
  &ralt_bspc,
  &shift_lprn,
  &shift_rprn,
  &shift_lbrc,
  &shift_rbrc,
  NULL
};

/**********
 * COMBOS *
 **********/

const uint16_t PROGMEM capsword_combo[] = {LT_NAV_SFT, LT_FUN_SFT, COMBO_END};

const uint16_t PROGMEM esc_combo[] = {DE_W, DE_E_EURO, COMBO_END};
const uint16_t PROGMEM tab_combo[]  = {DE_E_EURO, DE_R, COMBO_END};
const uint16_t PROGMEM lbrc_combo[] = {DE_S_SS, DE_D, COMBO_END};
const uint16_t PROGMEM lprn_combo[] = {DE_D, DE_F, COMBO_END};

const uint16_t PROGMEM bspc_combo[] = {DE_U_UE, DE_I, COMBO_END};
const uint16_t PROGMEM del_combo[]  = {DE_I, DE_O_OE, COMBO_END};
const uint16_t PROGMEM rprn_combo[] = {DE_J, DE_K, COMBO_END};
const uint16_t PROGMEM rbrc_combo[] = {DE_K, DE_L, COMBO_END};

combo_t key_combos[] = {
  COMBO(capsword_combo, CW_TOGG),

  COMBO(lbrc_combo, DE_LBRC),
  COMBO(lprn_combo, LSFT(DE_8)),
  COMBO(esc_combo, KC_ESC),
  COMBO(tab_combo, KC_TAB),

  COMBO(rprn_combo, LSFT(DE_9)),
  COMBO(rbrc_combo, DE_RBRC),
  COMBO(bspc_combo, KC_BSPC),
  COMBO(del_combo, KC_DEL)
};

/**********
 * KEYMAP *
 **********/

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

  [L_BASE] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         QK_GESC,         DE_1,            DE_2,            DE_3,            DE_4,            DE_5,                                                 DE_6,            DE_7,            DE_8,            DE_9,            DE_0,            KC_BSPC, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         KC_TAB,          DE_Q,            DE_W,            DE_E_EURO,       DE_R,            DE_T,                                                 DE_Z,            DE_U_UE,         DE_I,            DE_O_OE,         DE_P,            KC_DEL, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         DE_LABK,         DE_A_AE,         DE_S_SS,         DE_D,            DE_F,            DE_G,                                                 DE_H,            DE_J,            DE_K,            DE_L,            DE_PLUS,         DE_HASH, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         MT_SFT_MOUSE_L,  DE_Y,            DE_X,            DE_C,            DE_V,            DE_B,            XXXXXXX,            KC_MUTE,         DE_N,            DE_M,            DE_COMM,         DE_DOT,          DE_MINS,         MT_SFT_MOUSE_R, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           KC_LGUI,         KC_LALT,         KC_LCTL,         LT_NAV_SFT,      KC_SPC,             LT_NUM_ENT,      LT_FUN_SFT,      KC_RCTL,         KC_RALT,         KC_RGUI
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
         _______,         C(DE_Y),         C(DE_X),         C(DE_C),         C(DE_V),         C(DE_Z),         _______,            _______,         _______,         _______,         _______,         _______,         _______,         _______, 
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
         _______,         _______,         _______,         _______,         _______,         _______,         _______,            _______,         C(DE_Z),         C(DE_V),         C(DE_C),         C(DE_X),         C(DE_Y),         _______, 
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
         _______,         OSM(MOD_LGUI),   OSM(MOD_LALT),   OSM(MOD_LCTL),   OSM(MOD_LSFT),   _______,                                              _______,         KC_LEFT,         KC_DOWN,         KC_RGHT,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         _______,         _______,         _______,         _______,         _______,         _______,            _______,         C(DE_Z),         C(DE_V),         C(DE_C),         C(DE_X),         C(DE_Y),         _______, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_NUM] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         KC_MPLY,         KC_MSTP,         KC_MPRV,         KC_MNXT,         KC_MUTE,                                              _______,         _______,         _______,         _______,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         _______,         DE_7,            DE_8,            DE_9,            _______,                                              _______,         _______,         _______,         _______,         TD_BOOT,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         _______,         DE_4,            DE_5,            DE_6,            _______,                                              _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_RALT),   OSM(MOD_RGUI),   _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         _______,         DE_1,            DE_2,            DE_3,            _______,         _______,            _______,         _______,         _______,         _______,         _______,         _______,         _______, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         DE_COMM,         DE_DOT,          DE_0,               _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  ),

  [L_FUN] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         KC_MPLY,         KC_MSTP,         KC_MPRV,         KC_MNXT,         KC_MUTE,                                               _______,         _______,         _______,         _______,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         KC_F12,          KC_F7,           KC_F8,           KC_F9,           _______,                                              _______,         _______,         _______,         _______,         TD_BOOT,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         KC_F11,          KC_F4,           KC_F5,           KC_F6,           _______,                                              _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_RALT),   OSM(MOD_RGUI),   _______, 
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
