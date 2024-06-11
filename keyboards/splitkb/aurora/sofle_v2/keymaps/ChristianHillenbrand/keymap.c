#include QMK_KEYBOARD_H

#include "keymap_german.h"
#include "transactions.h"

#include "features/layer_lock.h"

#include "layers.h"
#include "oled.h"

void show_rgb_data(void);

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

void rpc_show_rgb_data_slave_handler(uint8_t in_buflen, const void* in_data, uint8_t out_buflen, void* out_data) {
  show_rgb_data();
}

void keyboard_post_init_user(void) {
  transaction_register_rpc(RPC_CAPS_WORD, rpc_caps_word_slave_handler);
  transaction_register_rpc(RPC_SPACE_PRESSED, rpc_space_pressed_slave_handler);
  transaction_register_rpc(RPC_SHOW_RGB_DATA, rpc_show_rgb_data_slave_handler);
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

/******************
 * Tristate layer *
 ******************/

layer_state_t layer_state_set_user(layer_state_t state) {
  return update_tri_layer_state(state, L_NAV, L_FUN, L_ADJ);
}

/*******************
 * CUSTOM KEYCODES *
 *******************/

enum custom_keycodes {
  LLOCK = SAFE_RANGE,

  KC_QWRTY,
  KC_COLMK,

  LT_NAV_SFT_ ,
  LT_FUN_SFT_,
};

#define LT_NAV_SFT LT(L_NAV, LT_NAV_SFT_)
#define LT_NUM_ENT LT(L_NUM, KC_ENT)
#define LT_FUN_SFT LT(L_FUN, LT_FUN_SFT_)

bool process_record_user(uint16_t keycode, keyrecord_t* record) {
  if (!process_layer_lock(keycode, record, LLOCK)) {
    return false;
  }

  switch (keycode) {
    case KC_SPC:
      if (record->event.pressed) {
        trigger_jump();
        transaction_rpc_send(RPC_SPACE_PRESSED, 0, NULL);
      }
      return true;

    case KC_QWRTY:
      if (record->event.pressed) {
        set_single_persistent_default_layer(L_QWRTY);
      }
      return false;

    case KC_COLMK:
      if (record->event.pressed) {
        set_single_persistent_default_layer(L_COLMK);
      }
      return false;

    case LT_NAV_SFT:
    case LT_FUN_SFT:
      if (record->event.pressed && record->tap.count) {
        if (get_oneshot_mods() & MOD_MASK_SHIFT) {
          del_oneshot_mods(MOD_MASK_SHIFT);
        } else {
          add_oneshot_mods(MOD_MASK_SHIFT);
        }
        return false;
      }
      return true;

    case RGB_TOG:
    case RGB_MOD:
    case RGB_HUI:
    case RGB_SAI:
    case RGB_VAI:
    case RGB_SPI:
      if (record->event.pressed) {
        show_rgb_data();
        transaction_rpc_send(RPC_SHOW_RGB_DATA, 0, NULL);
      }
      return true;

    default:
      break;
  }

  return true;
}

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

const uint16_t PROGMEM adia_combo[] = {DE_A, DE_S, COMBO_END};
const uint16_t PROGMEM odia_combo[] = {DE_I, DE_O, COMBO_END};
const uint16_t PROGMEM udia_combo[] = {DE_U, DE_I, COMBO_END};
const uint16_t PROGMEM ss_combo[] = {DE_S, DE_D, COMBO_END};
const uint16_t PROGMEM at_combo[] = {DE_Q, DE_W, COMBO_END};
const uint16_t PROGMEM euro_combo[] = {DE_W, DE_E, COMBO_END};

combo_t key_combos[] = {
  COMBO(adia_combo, DE_ADIA),
  COMBO(odia_combo, DE_ODIA),
  COMBO(udia_combo, DE_UDIA),
  COMBO(ss_combo, DE_SS),
  COMBO(at_combo, DE_AT),
  COMBO(euro_combo, DE_EURO),
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

/**********
 * KEYMAP *
 **********/

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

  [L_QWRTY] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         KC_ESC,          DE_1,            DE_2,            DE_3,            DE_4,            DE_5,                                                 DE_6,            DE_7,            DE_8,            DE_9,            DE_0,            KC_BSPC,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         KC_TAB,          DE_Q,            DE_W,            DE_E,            DE_R,            DE_T,                                                 DE_Z,            DE_U,            DE_I,            DE_O,            DE_P,            KC_DEL,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         CW_TOGG,         DE_A,            DE_S,            DE_D,            DE_F,            DE_G,                                                 DE_H,            DE_J,            DE_K,            DE_L,            DE_PLUS,         DE_HASH,
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
         KC_TAB,          DE_Q,            DE_W,            DE_F,            DE_P,            DE_B,                                                 DE_J,            DE_L,            DE_U,            DE_Y,            DE_PLUS,         KC_DEL,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         CW_TOGG,         DE_A,            DE_R,            DE_S,            DE_T,            DE_G,                                                 DE_M,            DE_N,            DE_E,            DE_I,            DE_O,            DE_HASH,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         SFT_T(DE_LABK),  DE_Z,            DE_X,            DE_C,            DE_D,            DE_V,            XXXXXXX,            KC_MUTE,         DE_K,            DE_H,            DE_COMM,         DE_DOT,          DE_MINS,         KC_RSFT,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           KC_LGUI,         KC_LALT,         KC_LCTL,         LT_NAV_SFT,      KC_SPC,             LT_NUM_ENT,      LT_FUN_SFT,      KC_RCTL,         KC_LALT,         KC_RGUI
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_NAV] = LAYOUT(
    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         LLOCK,           _______,         _______,         _______,         _______,                                              KC_ESC,          KC_HOME,         KC_UP,           KC_END,          KC_BSPC,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         OSM(MOD_LGUI),   OSM(MOD_LALT),   OSM(MOD_LCTL),   OSM(MOD_LSFT),   _______,                                              KC_TAB,          KC_LEFT,         KC_DOWN,         KC_RGHT,         KC_DEL,          _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         _______,         KC_BTN3,         KC_BTN2,         KC_BTN1,         _______,         _______,            _______,         C(DE_Z),         C(DE_V),         C(DE_C),         C(DE_X),         C(DE_Y),         _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         TG(L_NAV),       _______,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),

  [L_NUM] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         DE_TILD,         DE_7,            DE_8,            DE_9,            DE_QUES,                                              _______,         _______,         _______,         _______,         LLOCK,           _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         DE_LBRC,         DE_4,            DE_5,            DE_6,            DE_RBRC,                                              _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),   _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         DE_BSLS,         DE_1,            DE_2,            DE_3,            DE_HASH,         _______,            _______,         _______,         KC_BTN1,         KC_BTN2,         KC_BTN3,         _______,         _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         DE_COMM,         DE_DOT,          DE_0,               TG(L_NUM),       _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  ),

  [L_FUN] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         KC_F12,          KC_F7,           KC_F8,           KC_F9,           _______,                                              _______,         _______,         _______,         _______,         LLOCK,           _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         KC_F11,          KC_F4,           KC_F5,           KC_F6,           _______,                                              _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),   _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         KC_F10,          KC_F1,           KC_F2,           KC_F3,           _______,         _______,            _______,         _______,         KC_BTN1,         KC_BTN2,         KC_BTN3,         _______,         _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            _______,         TG(L_FUN),       _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  ),

  [L_ADJ] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              _______,         _______,         _______,         _______,         _______,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         QK_BOOT,         QK_RBT,          KC_QWRTY,        KC_COLMK,        _______,                                              RGB_TOG,         RGB_HUI,         RGB_SAI,         RGB_VAI,         RGB_SPI,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         KC_MPRV,         KC_MPLY,         KC_MSTP,         KC_MNXT,         _______,                                              RGB_MOD,         RGB_HUD,         RGB_SAD,         RGB_VAD,         RGB_SPD,         _______,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         _______,         _______,         _______,         _______,         _______,         _______,            _______,         _______,         _______,         _______,         _______,         _______,         _______,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         _______,         _______,         _______,            _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  ),
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

#define RGB_DATA_TIMEOUT 5000
uint32_t rgb_data_timer = 0;

void show_rgb_data(void) {
  rgb_data_timer = timer_read32();
}

bool rgb_data_visible(void) {
  return (rgb_data_timer &&
    timer_elapsed32(rgb_data_timer) < RGB_DATA_TIMEOUT);
}

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
  if (rgb_data_visible()) {
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

oled_rotation_t oled_init_user(oled_rotation_t rotation) {
  return rotation;
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
