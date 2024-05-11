#include QMK_KEYBOARD_H

#include "keymap_german.h"

enum layers {
  L_BASE = 0,
  L_MOUSE,
  L_NAV,
  L_NUM,
  L_FUN
};

/************************
 * DEACTIVATE POWER LED *
 ************************/

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
      add_weak_mods(MOD_LSFT);
      return true;

    case CW_TOGG:
    case DE_1 ... DE_0:
    case KC_BSPC:
    case KC_DEL:
    case DE_UNDS:
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

#define DE_A_AE LT(0, DE_A)
#define DE_O_OE LT(0, DE_O)
#define DE_U_UE LT(0, DE_U)
#define DE_S_SS LT(0, DE_S)

bool tap_hold(keyrecord_t* record, uint16_t hold_keycode) {
  if (!record->tap.count && record->event.pressed) {
    tap_code16(hold_keycode);
    return false;
  }
  return true;
}

bool caps_word_tap_hold(keyrecord_t* record, uint16_t keycode) {
  if (!record->tap.count && record->event.pressed) {
    if (is_caps_word_on())
      add_weak_mods(MOD_BIT(KC_LSFT));
    tap_code16(keycode);
    return false;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t* record) {
  switch (keycode) {
    case DE_A_AE: return caps_word_tap_hold(record, DE_ADIA);
    case DE_O_OE: return caps_word_tap_hold(record, DE_ODIA);
    case DE_U_UE: return caps_word_tap_hold(record, DE_UDIA);
    case DE_S_SS: return tap_hold(record, DE_SS);

    default: 
      return true;
  }
}

/**************
 * TAP DANCES *
 **************/

typedef enum {
    TD_UNKNOWN,
    TD_SINGLE_TAP,
    TD_SINGLE_HOLD,
    TD_DOUBLE_TAP
} td_state_t;

static td_state_t td_state;

void td_fn_boot(tap_dance_state_t *state, void *user_data) {
  if (state->count == 2) {
    reset_keyboard();
  }
}

td_state_t cur_dance(tap_dance_state_t* state) {
  if (state->count == 1) {
    if (!state->pressed) 
      return TD_SINGLE_TAP;
    else 
      return TD_SINGLE_HOLD;
  }
  else if (state->count == 2) {
    return TD_DOUBLE_TAP;
  }
  return TD_UNKNOWN;
}

void td_nav_sft_finished(tap_dance_state_t *state, void *user_data) {
  td_state = cur_dance(state);
  switch (td_state) {
    case TD_SINGLE_TAP:
      add_oneshot_mods(MOD_MASK_SHIFT);
      break;

    case TD_SINGLE_HOLD:
      layer_on(L_NAV);
      break;

    case TD_DOUBLE_TAP:
      caps_word_toggle();
      break;

    default:
      break;
  }
}

void td_nav_sft_reset(tap_dance_state_t *state, void *user_data) {
  switch (td_state) {
    case TD_SINGLE_HOLD:
      layer_off(L_NAV);
      break;

    default:
      break;
  }
}

void td_fun_sft_finished(tap_dance_state_t *state, void *user_data) {
  td_state = cur_dance(state);
  switch (td_state) {
    case TD_SINGLE_TAP:
      add_oneshot_mods(MOD_MASK_SHIFT);
      break;

    case TD_SINGLE_HOLD:
      layer_on(L_FUN);
      break;

    case TD_DOUBLE_TAP:
      caps_word_toggle();
      break;

    default:
      break;
  }
}

void td_fun_sft_reset(tap_dance_state_t *state, void *user_data) {
  switch (td_state) {
    case TD_SINGLE_HOLD:
      layer_off(L_FUN);
      break;

    default:
      break;
  }
}

void td_ctrl_mouse_finished(tap_dance_state_t *state, void *user_data) {
  td_state = cur_dance(state);
  switch (td_state) {
    case TD_SINGLE_HOLD:
      add_mods(MOD_MASK_CTRL);
      break;

    case TD_SINGLE_TAP:
      IS_LAYER_OFF(L_MOUSE) ? layer_on(L_MOUSE) : layer_off(L_MOUSE);
      break;      

    default:
      break;
  }
}

void td_ctrl_mouse_reset(tap_dance_state_t *state, void *user_data) {
  switch (td_state) {
    case TD_SINGLE_HOLD:
      del_mods(MOD_MASK_CTRL);
      break;

    default:
      break;
  }
}

enum tap_dances{
  TD_BOOT_ = 0,
  TD_NAV_SFT_,
  TD_FUN_SFT_,
  TD_CTRL_MOUSE_
};

tap_dance_action_t tap_dance_actions[] = { 
  [TD_BOOT_] = ACTION_TAP_DANCE_FN(td_fn_boot),
  [TD_NAV_SFT_] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, td_nav_sft_finished, td_nav_sft_reset),
  [TD_FUN_SFT_] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, td_fun_sft_finished, td_fun_sft_reset),
  [TD_CTRL_MOUSE_] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, td_ctrl_mouse_finished, td_ctrl_mouse_reset)
};

#define TD_BOOT TD(TD_BOOT_)
#define TD_NAV_SFT TD(TD_NAV_SFT_)
#define TD_FUN_SFT TD(TD_FUN_SFT_)
#define TD_CTRL_MOUSE TD(TD_CTRL_MOUSE_)

/*********************
 * TAP HOLD SETTINGS *
 *********************/

#define LT_NUM_ENT LT(L_NUM, KC_ENT)

bool get_hold_on_other_key_press(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case SFT_T(DE_LABK):
    case SFT_T(DE_HASH):
    case TD_CTRL_MOUSE:
      return true;

    default:
      return false;
  }
}

bool get_permissive_hold(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case TD_NAV_SFT:
    case LT_NUM_ENT:
    case TD_FUN_SFT:
      return true;

    default:
      return false;
  }
}

/*****************
 * KEY OVERRIDES *
 *****************/

const key_override_t shift_bspc_override = ko_make_basic(MOD_MASK_SHIFT, KC_BSPC, DE_QUES);
const key_override_t ralt_bspc_override  = ko_make_basic(MOD_BIT(KC_RALT), KC_BSPC, DE_BSLS);
const key_override_t shift_lprn_override = ko_make_basic(MOD_MASK_SHIFT, LSFT(DE_8), DE_LABK);
const key_override_t shift_rprn_override = ko_make_basic(MOD_MASK_SHIFT, LSFT(DE_9), LSFT(DE_LABK));
const key_override_t shift_lbrc_override = ko_make_basic(MOD_MASK_SHIFT, DE_LBRC, DE_LCBR);
const key_override_t shift_rbrc_override = ko_make_basic(MOD_MASK_SHIFT, DE_RBRC, DE_RCBR);

const key_override_t **key_overrides = (const key_override_t *[]){
  &shift_bspc_override,
  &ralt_bspc_override,
  &shift_lprn_override,
  &shift_rprn_override,
  &shift_lbrc_override,
  &shift_rbrc_override,
  NULL
};

/**********
 * COMBOS *
 **********/

const uint16_t PROGMEM caps_word_combo[] = {TD_NAV_SFT, TD_FUN_SFT, COMBO_END};

const uint16_t PROGMEM esc_combo[] = {DE_W, DE_E, COMBO_END};
const uint16_t PROGMEM tab_combo[]  = {DE_E, DE_R, COMBO_END};
const uint16_t PROGMEM lbrc_combo[] = {DE_S_SS, DE_D, COMBO_END};
const uint16_t PROGMEM lprn_combo[] = {DE_D, DE_F, COMBO_END};

const uint16_t PROGMEM bspc_combo_base[] = {DE_U_UE, DE_I, COMBO_END};
const uint16_t PROGMEM del_combo_base[]  = {DE_I, DE_O_OE, COMBO_END};
const uint16_t PROGMEM rprn_combo[] = {DE_J, DE_K, COMBO_END};
const uint16_t PROGMEM rbrc_combo[] = {DE_K, DE_L, COMBO_END};

const uint16_t PROGMEM bspc_combo_num[] = {KC_HOME, KC_UP, COMBO_END};
const uint16_t PROGMEM del_combo_num[]  = {KC_UP, KC_END, COMBO_END};

combo_t key_combos[] = {
  COMBO(caps_word_combo, CW_TOGG),

  COMBO(esc_combo, KC_ESC),
  COMBO(tab_combo, KC_TAB),
  COMBO(lbrc_combo, DE_LBRC),
  COMBO(lprn_combo, LSFT(DE_8)),

  COMBO(bspc_combo_base, KC_BSPC),
  COMBO(del_combo_base, KC_DEL),
  COMBO(rprn_combo, LSFT(DE_9)),
  COMBO(rbrc_combo, DE_RBRC),

  COMBO(bspc_combo_num, KC_BSPC),
  COMBO(del_combo_num, KC_DEL)
};

/**********
 * KEYMAP *
 **********/

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

  [L_BASE] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         QK_GESC,         DE_1,            DE_2,            DE_3,            DE_4,            DE_5,                                                 DE_6,            DE_7,            DE_8,            DE_9,            DE_0,            KC_BSPC, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         KC_TAB,          DE_Q,            DE_W,            DE_E,            DE_R,            DE_T,                                                 DE_Z,            DE_U_UE,         DE_I,            DE_O_OE,         DE_P,            KC_DEL, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         SFT_T(DE_LABK),  DE_A_AE,         DE_S_SS,         DE_D,            DE_F,            DE_G,                                                 DE_H,            DE_J,            DE_K,            DE_L,            DE_PLUS,         SFT_T(DE_HASH), 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         TD_CTRL_MOUSE,   DE_Y,            DE_X,            DE_C,            DE_V,            DE_B,            XXXXXXX,            KC_MUTE,         DE_N,            DE_M,            DE_COMM,         DE_DOT,          DE_MINS,         TD_CTRL_MOUSE, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           KC_LGUI,         KC_LALT,         KC_LCTL,         TD_NAV_SFT,      KC_SPC,             LT_NUM_ENT,      TD_FUN_SFT,      KC_RCTL,         KC_RALT,         KC_RGUI
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_MOUSE] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         TG(L_MOUSE),     _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         TG(L_MOUSE), 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         KC_WH_L,         KC_WH_D,         KC_WH_U,         KC_WH_R,         _______,                                              _______,         KC_WH_L,         KC_WH_D,         KC_WH_U,         KC_WH_R,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         KC_MS_L,         KC_MS_D,         KC_MS_U,         KC_MS_R,         _______,                                              _______,         KC_MS_L,         KC_MS_D,         KC_MS_U,         KC_MS_R,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         C(DE_Y),         C(DE_X),         C(DE_C),         C(DE_V),         C(DE_Z),         _______,            _______,         C(DE_Z),         C(DE_V),         C(DE_C),         C(DE_X),         C(DE_Y),         _______, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           KC_BTN5,         KC_BTN4,         KC_BTN3,         KC_BTN2,         KC_BTN1,            KC_BTN1,         KC_BTN2,         KC_BTN3,         KC_BTN4,         KC_BTN5
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
