#include QMK_KEYBOARD_H

#include "keymap_german.h"

enum layers {
  L_BASE = 0,
  L_MOUSE,  
  L_NAV,
  L_MEDIA,
  L_NUM,
  L_FUN
};

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

    case DE_1 ... DE_0:
    case KC_BSPC:
    case KC_DEL:
    case DE_UNDS:
      return true;

    default:
      return false;
  }
}

/*************
 * TAP HOLDS *
 *************/

#define DE_A_AE LT(0, DE_A)
#define DE_O_OE LT(0, DE_O)
#define DE_U_UE LT(0, DE_U)
#define DE_S_SS LT(0, DE_S)

#define NAV_LEFT LT(0, KC_LEFT)
#define NAV_RGHT LT(0, KC_RGHT)
#define NAV_UP LT(0, KC_UP)
#define NAV_DOWN LT(0, KC_DOWN)
#define NAV_BSPC LT(0, KC_BSPC)
#define NAV_DEL LT(0, KC_DEL)

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

    case NAV_LEFT: return tap_hold(record, KC_HOME);
    case NAV_RGHT: return tap_hold(record, KC_END);
    case NAV_UP:   return tap_hold(record, C(KC_HOME));
    case NAV_DOWN: return tap_hold(record, C(KC_END));
    case NAV_BSPC: return tap_hold(record, C(KC_BSPC));
    case NAV_DEL:  return tap_hold(record, C(KC_DEL));

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

void lt_media_cw_finished(tap_dance_state_t *state, void *user_data) {
  td_state = cur_dance(state);
  switch (td_state) {
    case TD_SINGLE_TAP:
      caps_word_on();
      break;

    case TD_SINGLE_HOLD:
      layer_on(L_MEDIA);
      break;

    default:
      break;
  }
}

void lt_media_cw_reset(tap_dance_state_t *state, void *user_data) {
  switch (td_state) {
    case TD_SINGLE_HOLD:
      layer_off(L_MEDIA);
      break;

    default:
      break;
  }
}

void lt_fun_sft_finished(tap_dance_state_t *state, void *user_data) {
  td_state = cur_dance(state);
  switch (td_state) {
    case TD_SINGLE_TAP:
      add_oneshot_mods(MOD_BIT(KC_LSFT));
      break;

    case TD_SINGLE_HOLD:
      layer_on(L_FUN);
      break;

    default:
      break;
  }
}

void lt_fun_sft_reset(tap_dance_state_t *state, void *user_data) {
  switch (td_state) {
    case TD_SINGLE_HOLD:
      layer_off(L_FUN);
      break;

    default:
      break;
  }
}

void mt_mouse_ralt_finished(tap_dance_state_t *state, void *user_data) {
  td_state = cur_dance(state);
  switch (td_state) {
    case TD_SINGLE_TAP:
      if (IS_LAYER_OFF(L_MOUSE))
        layer_on(L_MOUSE);
      else
        layer_off(L_MOUSE);
      break;

    case TD_SINGLE_HOLD:
      add_mods(MOD_BIT(KC_RALT));
      break;

    default:
      break;
  }
}

void mt_mouse_ralt_reset(tap_dance_state_t *state, void *user_data) {
  switch (td_state) {
    case TD_SINGLE_HOLD:
      del_mods(MOD_BIT(KC_RALT));
      break;

    default:
      break;
  }
}

enum tap_dances{
  TD_BOOT = 0,
  TD_LT_MEDIA_CW,
  TD_LT_FUN_SFT,
  TD_MT_MOUSE_RALT
};

tap_dance_action_t tap_dance_actions[] = { 
  [TD_BOOT] = ACTION_TAP_DANCE_FN(td_fn_boot),
  [TD_LT_MEDIA_CW] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, lt_media_cw_finished, lt_media_cw_reset),
  [TD_LT_FUN_SFT] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, lt_fun_sft_finished, lt_fun_sft_reset),
  [TD_MT_MOUSE_RALT] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, mt_mouse_ralt_finished, mt_mouse_ralt_reset)
};

/*********************
 * TAP HOLD SETTINGS *
 *********************/

#define LT_NAV_SPC LT(L_NAV, KC_SPC)
#define LT_MEDIA_CW TD(TD_LT_MEDIA_CW)
#define LT_NUM_ENT LT(L_NUM, KC_ENT)
#define LT_FUN_SFT TD(TD_LT_FUN_SFT)
#define MT_MOUSE_RALT TD(TD_MT_MOUSE_RALT)

bool get_hold_on_other_key_press(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case SFT_T(DE_LABK):
    case SFT_T(DE_HASH):
      return true;

    default:
      return false;
  }
}

bool get_permissive_hold(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case LT_NAV_SPC:
    case LT_MEDIA_CW:
    case LT_NUM_ENT:
    case LT_FUN_SFT:
    case MT_MOUSE_RALT:
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

const uint16_t PROGMEM bspc_combo[] = {KC_HOME, KC_UP, COMBO_END};
const uint16_t PROGMEM del_combo[]  = {KC_UP, KC_END, COMBO_END};
const uint16_t PROGMEM lprn_combo[] = {DE_D, DE_F, COMBO_END};
const uint16_t PROGMEM rprn_combo[] = {DE_J, DE_K, COMBO_END};
const uint16_t PROGMEM lbrc_combo[] = {DE_S_SS, DE_D, COMBO_END};
const uint16_t PROGMEM rbrc_combo[] = {DE_K, DE_L, COMBO_END};

combo_t key_combos[] = {
  COMBO(bspc_combo, KC_BSPC),
  COMBO(del_combo, KC_DEL),
  COMBO(lprn_combo, LSFT(DE_8)),
  COMBO(rprn_combo, LSFT(DE_9)),
  COMBO(lbrc_combo, DE_LBRC),
  COMBO(rbrc_combo, DE_RBRC),
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
         KC_LCTL,         DE_Y,            DE_X,            DE_C,            DE_V,            DE_B,            KC_MUTE,            XXXXXXX,         DE_N,            DE_M,            DE_COMM,         DE_DOT,          DE_MINS      ,   KC_RCTL, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           KC_LGUI,         KC_LALT,         MT_MOUSE_RALT,   LT_MEDIA_CW,     LT_NAV_SPC,         LT_NUM_ENT,      LT_FUN_SFT,      MT_MOUSE_RALT,   KC_LALT,         KC_RGUI
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_MOUSE] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         TG(L_MOUSE),     _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         TG(L_MOUSE), 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         _______,         _______,         KC_WH_U,         _______,         _______,                                              _______,         _______,         KC_WH_U,         _______,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         _______,         KC_WH_L,         KC_WH_D,         KC_WH_R,         _______,                                              _______,         KC_WH_L,         KC_WH_D,         KC_WH_R,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         C(DE_Y),         C(DE_X),         C(DE_C),         C(DE_V),         C(DE_Z),         _______,            _______,         C(DE_Z),         C(DE_V),         C(DE_C),         C(DE_X),         C(DE_Y),         _______, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         KC_BTN2,         KC_BTN1,            KC_BTN1,         KC_BTN2,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ), 

  [L_NAV] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         TD(TD_BOOT),     _______,         _______,         _______,         _______,                                              _______,         KC_HOME,         KC_UP,           KC_END,          _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         KC_LGUI,         KC_LALT,         KC_LCTL,         KC_LSFT,         KC_RALT,                                              _______,         KC_LEFT,         KC_DOWN,         KC_RGHT,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         _______,         _______,         _______,         _______,         _______,         _______,            _______,         C(DE_Z),         C(DE_V),         C(DE_C),         C(DE_X),         C(DE_Y),         _______, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_MEDIA] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         TD(TD_BOOT),     _______,         _______,         _______,         _______,                                              RGB_TOG,         _______,         KC_VOLU,         _______,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         KC_LGUI,         KC_LALT,         KC_LCTL,         KC_LSFT,         KC_RALT,                                              _______,         KC_MPRV,         KC_VOLD,         KC_MNXT,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         _______,         _______,         _______,         _______,         _______,         _______,            _______,         RGB_MOD,         RGB_HUI,         RGB_SAI,         RGB_VAI,         RGB_SPI,         _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            KC_MPLY,         KC_MSTP,         KC_MUTE,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_NUM] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         _______,         DE_7,            DE_8,            DE_9,            _______,                                              _______,         _______,         _______,         _______,         TD(TD_BOOT),     _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         _______,         DE_4,            DE_5,            DE_6,            _______,                                              KC_RALT,         KC_RSFT,         KC_RCTL,         KC_LALT,         KC_RGUI,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         _______,         DE_1,            DE_2,            DE_3,            _______,         _______,            _______,         _______,         _______,         _______,         _______,         _______,         _______, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           DE_COMM,         DE_0,            DE_DOT,          _______,         _______,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  ),

  [L_FUN] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         KC_F12,          KC_F7,           KC_F8,           KC_F9,           _______,                                              _______,         _______,         _______,         _______,         TD(TD_BOOT),     _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         KC_F11,          KC_F4,           KC_F5,           KC_F6,           _______,                                              KC_RALT,         KC_RSFT,         KC_RCTL,         KC_LALT,         KC_RGUI,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         KC_F10,          KC_F1,           KC_F2,           KC_F3,           _______,         _______,            _______,         _______,         _______,         _______,         _______,         _______,         _______, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  )
};

const uint16_t PROGMEM encoder_map[][NUM_ENCODERS][NUM_DIRECTIONS] = {
  [L_BASE]  = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU), ENCODER_CCW_CW(KC_WH_U, KC_WH_D)},
  [L_MOUSE] = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU), ENCODER_CCW_CW(KC_WH_U, KC_WH_D)},   
  [L_NAV]   = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU), ENCODER_CCW_CW(KC_WH_U, KC_WH_D)},
  [L_MEDIA] = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU), ENCODER_CCW_CW(KC_WH_U, KC_WH_D)},  
  [L_NUM]   = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU), ENCODER_CCW_CW(KC_WH_U, KC_WH_D)},
  [L_FUN]   = {ENCODER_CCW_CW(KC_VOLD, KC_VOLU), ENCODER_CCW_CW(KC_WH_U, KC_WH_D)}
};
