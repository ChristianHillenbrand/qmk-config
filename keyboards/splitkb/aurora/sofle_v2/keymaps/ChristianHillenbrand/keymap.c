#include QMK_KEYBOARD_H

#include "keymap_german.h"
#include "transactions.h"

#include "features/layer_lock.h"

#include "layers.h"
#include "oled.h"

/*************
 * POWER LED *
 *************/

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

void keyboard_post_init_user(void) {
  transaction_register_rpc(RPC_CAPS_WORD, rpc_caps_word_slave_handler);
}

void caps_word_set_user(bool active) {
  if (!is_keyboard_master()) {
    return;
  }

  if (active) {
    writePinLow(24);
  } else {
    writePinHigh(24);
  }

  transaction_rpc_send(RPC_CAPS_WORD, sizeof(active), &active);
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
    case DE_UNDS:
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
  LLOCK = SAFE_RANGE,

  KC_DCLK,
  KC_HOLD,

  DE_A_AE,
  DE_O_OE,
  DE_U_UE,
  DE_S_SS,
  DE_E_EURO,



  LT_NAV_SFT_ ,
  LT_FUN_SFT_,
};

#define KC_LCLK KC_BTN1
#define KC_RCLK KC_BTN2

#define MT_SFT_BSLS MT(MOD_LSFT | MOD_RSFT, DE_BSLS)

#define LT_NAV_SFT LT(L_NAV, LT_NAV_SFT_)
#define LT_NUM_ENT LT(L_NUM, KC_ENT)
#define LT_FUN_SFT LT(L_FUN, LT_FUN_SFT_)

bool is_mod_active(uint8_t mods, uint8_t mask)
{
  return (mods & mask) != 0;
}

bool is_mod_inactive(uint8_t mods, uint8_t mask)
{
  return (mods & mask) == 0;
}

bool is_shift_active(void)
{
  uint8_t mods = get_mods() | get_oneshot_mods() | get_weak_mods();
  return is_mod_active(mods, MOD_MASK_SHIFT);
}

bool is_only_shift_active(void)
{
  uint8_t mods = get_mods() | get_oneshot_mods() | get_weak_mods();
  return is_mod_inactive(mods, ~MOD_MASK_SHIFT);
}

static uint16_t fast_tap_keycode = 0;
static uint16_t fast_hold_keycode = 0;
static uint16_t fast_tap_hold_timer = 0;
static bool fast_tap_hold_pressed = false;

bool fast_tap_hold(uint16_t tap_keycode, uint16_t hold_keycode, bool keep_shift, keyrecord_t* record) {
  if (record->event.pressed) {
    // get shift state before tapping to not loose oneshots
    process_caps_word(tap_keycode, record);
    bool shift_active = is_shift_active();

    tap_code16(tap_keycode);
    fast_tap_keycode = tap_keycode;

    if (is_only_shift_active()) {
      if (shift_active && keep_shift) {
        fast_hold_keycode = S(hold_keycode);
      } else {
        fast_hold_keycode = hold_keycode;
      }

      fast_tap_hold_timer = record->event.time + TAPPING_TERM;
      fast_tap_hold_pressed = true;
    }
  } else if (tap_keycode == fast_tap_keycode) {
    fast_tap_hold_pressed = false;
  }

  return false;
}

bool tap_shift(keyrecord_t* record) {
  if (record->event.pressed && record->tap.count) {
    if (is_mod_active(get_oneshot_mods(), MOD_MASK_SHIFT)) {
      del_oneshot_mods(MOD_MASK_SHIFT);
    } else {
      add_oneshot_mods(MOD_MASK_SHIFT);
    }
    return false;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t* record) {
  static bool is_hold_active = false;

  if (!process_layer_lock(keycode, record, LLOCK)) {
    return false;
  }

  switch (keycode) {
    case KC_LCLK:
      is_hold_active = false;
      return true;

    case KC_DCLK:
      if (record->event.pressed) {
        tap_code16(KC_LCLK);
        wait_ms(50);
        tap_code16(KC_LCLK);
        is_hold_active = false;
      }
      return false;

    case KC_HOLD:
      if (record->event.pressed) {
        if (is_hold_active) {
          unregister_code16(KC_LCLK);
          is_hold_active = false;
        } else {
          register_code16(KC_LCLK);
          is_hold_active = true;
        }
      }
      return false;

    case DE_A_AE:
      return fast_tap_hold(DE_A, DE_ADIA, true, record);

    case DE_O_OE:
      return fast_tap_hold(DE_O, DE_ODIA, true, record);

    case DE_U_UE:
      return fast_tap_hold(DE_U, DE_UDIA, true, record);

    case DE_S_SS:
      return fast_tap_hold(DE_S, DE_SS, false, record);

    case DE_E_EURO:
      return fast_tap_hold(DE_E, DE_EURO, false, record);

    case LT_NAV_SFT:
      return tap_shift(record);

    case LT_FUN_SFT:
      return tap_shift(record);

    case RGB_TOG:
      show_rgb_status();
      break;

    case RGB_MOD:
    case RGB_HUI:
    case RGB_SAI:
    case RGB_VAI:
    case RGB_SPI:
      if (rgb_matrix_is_enabled()) {
        show_rgb_status();
      }
      break;

    default:
      if (record->event.pressed) {
        fast_tap_hold_pressed = false;
      }
      break;
  }

  return true;
}

void matrix_scan_user(void) {
  if (fast_tap_hold_pressed && timer_expired(timer_read(), fast_tap_hold_timer)) {
    // make sure we really get the case we wanted
    clear_weak_mods();
    uint8_t mods = get_mods();
    unregister_mods(mods);

    tap_code16(KC_BSPC);
    tap_code16(fast_hold_keycode);
    fast_tap_hold_pressed = false;

    register_mods(mods);
  }
}

/*********************
 * TAP HOLD SETTINGS *
 *********************/

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

bool get_hold_on_other_key_press(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case SFT_T(DE_LABK):
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

void td_fn_reset(tap_dance_state_t *state, void *user_date) {
  if (state->count == 2) {
    soft_reset_keyboard();
  }
}

void td_fn_mode(tap_dance_state_t *state, void *user_date) {
  if (state->count == 2) {
    switch (get_highest_layer(default_layer_state)) {
      case L_QWRTY:
        set_single_persistent_default_layer(L_COLMK);
        break;

      case L_COLMK:
        set_single_persistent_default_layer(L_QWRTY);
        break;

      default:
        break;
    }
  }
}

enum tap_dances{
  TD_BOOT_ = 0,
  TD_RESET_,
  TD_MODE_
};

tap_dance_action_t tap_dance_actions[] = {
  [TD_BOOT_] = ACTION_TAP_DANCE_FN(td_fn_boot),
  [TD_RESET_] = ACTION_TAP_DANCE_FN(td_fn_reset),
  [TD_MODE_] = ACTION_TAP_DANCE_FN(td_fn_mode),
};

#define TD_BOOT TD(TD_BOOT_)
#define TD_RESET TD(TD_RESET_)
#define TD_MODE TD(TD_MODE_)

/*****************
 * KEY OVERRIDES *
 *****************/

const key_override_t shift_esc  = ko_make_basic(MOD_MASK_SHIFT, KC_ESC, DE_TILD);
const key_override_t shift_bspc = ko_make_basic(MOD_MASK_SHIFT, KC_BSPC, DE_QUES);
const key_override_t shift_lprn = ko_make_basic(MOD_MASK_SHIFT, DE_LPRN, DE_LABK);
const key_override_t shift_rprn = ko_make_basic(MOD_MASK_SHIFT, DE_RPRN, DE_RABK);
const key_override_t shift_lbrc = ko_make_basic(MOD_MASK_SHIFT, DE_LBRC, DE_LCBR);
const key_override_t shift_rbrc = ko_make_basic(MOD_MASK_SHIFT, DE_RBRC, DE_RCBR);
const key_override_t shift_bsls = ko_make_basic(MOD_MASK_SHIFT, DE_BSLS, DE_PIPE);

const key_override_t **key_overrides = (const key_override_t *[]){
  &shift_esc,
  &shift_bspc,
  &shift_lprn,
  &shift_rprn,
  &shift_lbrc,
  &shift_rbrc,
  &shift_bsls,
  NULL
};

/**********
 * COMBOS *
 **********/

const uint16_t PROGMEM lbrc_combo[] = {DE_S_SS, DE_D, COMBO_END};
const uint16_t PROGMEM lprn_combo[] = {DE_D, DE_F, COMBO_END};
const uint16_t PROGMEM rprn_combo[] = {DE_J, DE_K, COMBO_END};
const uint16_t PROGMEM rbrc_combo[] = {DE_K, DE_L, COMBO_END};

combo_t key_combos[] = {
  COMBO(lbrc_combo, DE_LBRC),
  COMBO(lprn_combo, DE_LPRN),
  COMBO(rprn_combo, DE_RPRN),
  COMBO(rbrc_combo, DE_RBRC)
};

uint8_t combo_ref_from_layer(uint8_t layer){
  switch (layer){
    case L_COLMK: return L_QWRTY;
    default: return layer;
  }
}

/**********
 * KEYMAP *
 **********/

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

  [L_QWRTY] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         KC_ESC,          DE_1,            DE_2,            DE_3,            DE_4,            DE_5,                                                 DE_6,            DE_7,            DE_8,            DE_9,            DE_0,            KC_BSPC,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         KC_TAB,          DE_Q,            DE_W,            DE_E_EURO,       DE_R,            DE_T,                                                 DE_Z,            DE_U_UE,         DE_I,            DE_O_OE,         DE_P,            KC_DEL,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         CW_TOGG,         DE_A_AE,         DE_S_SS,         DE_D,            DE_F,            DE_G,                                                 DE_H,            DE_J,            DE_K,            DE_L,            DE_PLUS,         DE_HASH,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         SFT_T(DE_LABK),  DE_Y,            DE_X,            DE_C,            DE_V,            DE_B,            XXXXXXX,            KC_MUTE,         DE_N,            DE_M,            DE_COMM,         DE_DOT,          DE_MINS,         KC_RSFT,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           KC_LGUI,         KC_LALT,         KC_LCTL,         LT_NAV_SFT,      KC_SPC,             LT_NUM_ENT,      LT_FUN_SFT,      KC_RCTL,         KC_LALT,         KC_RGUI
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_COLMK] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         KC_ESC,          DE_1,            DE_2,            DE_3,            DE_4,            DE_5,                                                 DE_6,            DE_7,            DE_8,            DE_9,            DE_0,            KC_BSPC,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         KC_TAB,          DE_Q,            DE_W,            DE_F,            DE_P,            DE_B,                                                 DE_J,            DE_L,            DE_U_UE,         DE_Y,            DE_PLUS,         KC_DEL,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         CW_TOGG,         DE_A_AE,         DE_R,            DE_S_SS,         DE_T,            DE_G,                                                 DE_M,            DE_N,            DE_E,            DE_I,            DE_O_OE,         DE_HASH,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         SFT_T(DE_LABK),  DE_Z,            DE_X,            DE_C,            DE_D,            DE_V,            XXXXXXX,            KC_MUTE,         DE_K,            DE_H,            DE_COMM,         DE_DOT,          DE_MINS,         KC_RSFT,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           KC_LGUI,         KC_LALT,         KC_LCTL,         LT_NAV_SFT,      KC_SPC,             LT_NUM_ENT,      LT_FUN_SFT,      KC_RCTL,         KC_LALT,         KC_RGUI
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_GER] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         DE_SUP2,         DE_SUP3,         _______,         _______,                                              _______,         DE_LCBR,         DE_LBRC,         DE_RBRC,         DE_RCBR,         DE_BSLS,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         DE_AT,           _______,         DE_EURO,         _______,         _______,                                              _______,         DE_UDIA,         _______,         DE_ODIA,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         DE_ADIA,         DE_SS,           _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         DE_TILD,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         DE_PIPE,         _______,         _______,         _______,         _______,         _______,         _______,            _______,         _______,         DE_MICR,         _______,         _______,         _______,         _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_NAV] = LAYOUT(
    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         RGB_TOG,         KC_MPLY,         KC_MSTP,         KC_MPRV,         KC_MNXT,                                              RGB_MOD,         RGB_HUI,         RGB_SAI,         RGB_VAI,         RGB_SPI,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         TD_BOOT,         TD_RESET,        TD_MODE,         LLOCK,           _______,                                              KC_ESC,          KC_HOME,         KC_UP,           KC_END,          KC_BSPC,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         OSM(MOD_LGUI),   OSM(MOD_LALT),   OSM(MOD_LCTL),   OSM(MOD_LSFT),   _______,                                              KC_TAB,          KC_LEFT,         KC_DOWN,         KC_RGHT,         KC_DEL,          _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         KC_HOLD,         KC_DCLK,         KC_RCLK,         KC_LCLK,         _______,         _______,            _______,         C(DE_Z),         C(DE_V),         C(DE_C),         C(DE_X),         C(DE_Y),         _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         TG(L_NAV),       _______,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_NUM] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         RGB_TOG,         KC_MPLY,         KC_MSTP,         KC_MPRV,         KC_MNXT,                                              RGB_MOD,         RGB_HUI,         RGB_SAI,         RGB_VAI,         RGB_SPI,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         DE_TILD,         DE_7,            DE_8,            DE_9,            DE_QUES,                                              _______,         LLOCK,           TD_MODE,         TD_RESET,        TD_BOOT,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         DE_CIRC,         DE_4,            DE_5,            DE_6,            DE_ACUT,                                              _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),   _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         DE_BSLS,         DE_1,            DE_2,            DE_3,            DE_HASH,         _______,            _______,         _______,         KC_LCLK,         KC_RCLK,         KC_DCLK,         KC_HOLD,         _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         DE_COMM,         DE_DOT,          DE_0,               TG(L_NUM),       _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  ),

  [L_FUN] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         RGB_TOG,         KC_MPLY,         KC_MSTP,         KC_MPRV,         KC_MNXT,                                              RGB_MOD,         RGB_HUI,         RGB_SAI,         RGB_VAI,         RGB_SPI,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         KC_F12,          KC_F7,           KC_F8,           KC_F9,           _______,                                              _______,         LLOCK,           TD_MODE,         TD_RESET,        TD_BOOT,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         KC_F11,          KC_F4,           KC_F5,           KC_F6,           _______,                                              _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),   _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         KC_F10,          KC_F1,           KC_F2,           KC_F3,           _______,         _______,            _______,         _______,         KC_LCLK,         KC_RCLK,         KC_DCLK,         KC_HOLD,         _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            _______,         TG(L_FUN),       _______,         _______,         _______
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
