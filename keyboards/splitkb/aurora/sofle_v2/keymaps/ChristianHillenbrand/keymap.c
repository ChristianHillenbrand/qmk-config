#include QMK_KEYBOARD_H

#include "keymap_us_international.h"

enum layers {
  L_BASE = 0,
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

void caps_word_set_user(bool active) {
  if (active) {
    writePinLow(24);
  } else {
    writePinHigh(24);
  }
}

/*******************
 * CUSTOM KEYCODES *
 *******************/ 

enum custom_keycodes {
  MC_DCLK = SAFE_RANGE,
  MC_DRAG,

  MC_A_AE,
  MC_O_OE,
  MC_U_UE,
  MC_S_SS,
  MC_E_EURO,

  LT_NAV_SFT_ ,
  LT_FUN_SFT_,
};

#define KC_LCLK KC_BTN1
#define KC_RCLK KC_BNT2

#define LT_NAV_SFT LT(0, LT_NAV_SFT_)
#define LT_NUM_ENT LT(L_NUM, KC_ENT)
#define LT_FUN_SFT LT(0, LT_FUN_SFT_)

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

bool fast_tap_hold(uint16_t tap_keycode, uint16_t hold_keycode, bool keep_shift, keyrecord_t* record)
{
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

bool layer_shift(uint16_t layer, keyrecord_t* record)
{
  if (record->event.pressed) {
    if (record->tap.count) {
      if (is_mod_active(get_oneshot_mods(), MOD_MASK_SHIFT)) {
        del_oneshot_mods(MOD_MASK_SHIFT);
      } else {
        add_oneshot_mods(MOD_MASK_SHIFT);
      }
    } else {
      del_oneshot_mods(MOD_MASK_SHIFT);
      layer_on(layer);
    }
  } else if (!record->tap.count) {
      layer_off(layer);
  }

  return false;
}

bool process_record_user(uint16_t keycode, keyrecord_t* record) {
  static bool is_drag_active = false;

  switch (keycode) {
    case KC_LCLK:
      is_drag_active = false;
      return true;

    case MC_DCLK:
      if (record->event.pressed) {
        tap_code16(KC_LCLK);
        wait_ms(50);
        tap_code16(KC_LCLK);
        is_drag_active = false;
      }
      return false;

    case MC_DRAG:
      if (record->event.pressed) {
        if (is_drag_active) {
          unregister_code16(KC_LCLK);
          is_drag_active = false;
        } else {
          register_code16(KC_LCLK);
          is_drag_active = true;
        }
      }
      return false;

    case MC_A_AE:
      return fast_tap_hold(US_A, US_ADIA, true, record);

    case MC_O_OE:
      return fast_tap_hold(US_O, US_ODIA, true, record);

    case MC_U_UE:
      return fast_tap_hold(US_U, US_UDIA, true, record);

    case MC_S_SS:
      return fast_tap_hold(US_S, US_SS, false, record);

    case MC_E_EURO:
      return fast_tap_hold(US_E, US_EURO, false, record); 

    case LT_NAV_SFT:
      return layer_shift(L_NAV, record);

    case LT_FUN_SFT:
      return layer_shift(L_FUN, record);
    
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

enum tap_dances{
  TD_BOOT_ = 0,
  TD_RESET_
};

tap_dance_action_t tap_dance_actions[] = { 
  [TD_BOOT_] = ACTION_TAP_DANCE_FN(td_fn_boot),
  [TD_RESET_] = ACTION_TAP_DANCE_FN(td_fn_reset)
};

#define TD_BOOT TD(TD_BOOT_)
#define TD_RESET TD(TD_RESET_)

/*****************
 * KEY OVERRIDES *
 *****************/

const key_override_t shift_esc  = ko_make_basic(MOD_MASK_SHIFT, KC_ESC, US_DTIL);
const key_override_t shift_lprn = ko_make_basic(MOD_MASK_SHIFT, US_LPRN, US_LABK);
const key_override_t shift_rprn = ko_make_basic(MOD_MASK_SHIFT, US_RPRN, US_RABK);

const key_override_t **key_overrides = (const key_override_t *[]){
  &shift_esc,
  &shift_lprn,
  &shift_rprn,  
  NULL
};

/**********
 * COMBOS *
 **********/

const uint16_t PROGMEM lprn_combo[] = {MC_S_SS, US_D, COMBO_END};
const uint16_t PROGMEM rprn_combo[] = {US_D, US_F, COMBO_END};

const uint16_t PROGMEM mins_combo[] = {MC_U_UE, US_I, COMBO_END};
const uint16_t PROGMEM eql_combo[]  = {US_I, MC_O_OE, COMBO_END};
const uint16_t PROGMEM lbrc_combo[] = {US_J, US_K, COMBO_END};
const uint16_t PROGMEM rbrc_combo[] = {US_K, US_L, COMBO_END};

combo_t key_combos[] = {
  COMBO(lprn_combo, US_LPRN),
  COMBO(rprn_combo, US_RPRN),

  COMBO(mins_combo, US_MINS),
  COMBO(eql_combo, US_EQL),
  COMBO(lbrc_combo, US_LBRC),
  COMBO(rbrc_combo, US_RBRC)
};

/**********
 * KEYMAP *
 **********/

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

  [L_BASE] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         KC_ESC,          US_1,            US_2,            US_3,            US_4,            US_5,                                                 US_6,            US_7,            US_8,            US_9,            US_0,            KC_BSPC,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         KC_TAB,          US_Q,            US_W,            MC_E_EURO,       US_R,            US_T,                                                 US_Z,            MC_U_UE,         US_I,            MC_O_OE,         US_P,            KC_DEL,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         CW_TOGG,         MC_A_AE,         MC_S_SS,         US_D,            US_F,            US_G,                                                 US_H,            US_J,            US_K,            US_L,            US_SCLN,         US_ACUT,
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         SFT_T(KC_BSLS),  US_Y,            US_X,            US_C,            US_V,            US_B,            XXXXXXX,            KC_MUTE,         US_N,            US_M,            US_COMM,         US_DOT,          US_SLSH,         KC_RSFT,
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           KC_LGUI,         KC_LALT,         KC_LCTL,         LT_NAV_SFT,      KC_SPC,             LT_NUM_ENT,      LT_FUN_SFT,      KC_RCTL,         KC_LALT,         KC_RGUI
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯

  ),
  
  [L_NAV] = LAYOUT(
    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         _______,         _______,         _______,         _______,         _______,                                              RGB_TOG,         RGB_MOD,         RGB_HUI,         RGB_SAI,         RGB_VAI,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         TD_BOOT,         TD_RESET,        _______,         _______,         _______,                                              _______,         KC_HOME,         KC_UP,           KC_END,          _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         OSM(MOD_LGUI),   OSM(MOD_LALT),   OSM(MOD_LCTL),   OSM(MOD_LSFT),   _______,                                              _______,         KC_LEFT,         KC_DOWN,         KC_RGHT,         _______,         _______, 
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
         _______,         US_LBRC,         US_7,            US_8,            US_9,            US_RBRC,                                               _______,         _______,         _______,         TD_RESET,        TD_BOOT,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         US_MINS,         US_4,            US_5,            US_6,            US_EQL,                                              _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),   _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────╮  ╭────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤
         _______,         US_LPRN,         US_1,            US_2,            US_3,            US_RPRN,         _______,            _______,         _______,         _______,         _______,         _______,         _______,         _______, 
    // ╰────────────────┴────────────────┴────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤  ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┴────────────────┴────────────────╯
                                           _______,         _______,         US_COMM,         US_DOT,          US_0,               _______,         _______,         _______,         _______,         _______
    //                                   ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯  ╰────────────────┴────────────────┴────────────────┴────────────────┴────────────────╯
  ),

  [L_FUN] = LAYOUT(

    // ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮                                    ╭────────────────┬────────────────┬────────────────┬────────────────┬────────────────┬────────────────╮
         _______,         KC_MPLY,         KC_MSTP,         KC_MPRV,         KC_MNXT,         KC_MUTE,                                              _______,         _______,         _______,         _______,         _______,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤     
         _______,         KC_F12,          KC_F7,           KC_F8,           KC_F9,           _______,                                              _______,         _______,         _______,         TD_RESET,        TD_BOOT,         _______, 
    // ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤                                    ├────────────────┼────────────────┼────────────────┼────────────────┼────────────────┼────────────────┤          
         _______,         KC_F11,          KC_F4,           KC_F5,           KC_F6,           _______,                                              _______,         OSM(MOD_RSFT),   OSM(MOD_RCTL),   OSM(MOD_LALT),   OSM(MOD_RGUI),   _______, 
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

/********
 * OLED *
 ********/

extern void render_logo(void);
extern void render_logo_text(void);
extern void render_space(void);
extern void render_mod_status_gui_alt(uint8_t);
extern void render_mod_status_ctrl_shift(uint8_t);

void render_layer_state_user(void) {
  oled_write_ln_P(PSTR("LAYER"), false);
  switch (get_highest_layer(layer_state)) {
    case L_BASE:
      oled_write_P(PSTR("BASE\n"), false);
      break;

    case L_NAV:
      oled_write_P(PSTR("NAV\n"), false);
      break;

    case L_NUM:
      oled_write_P(PSTR("NUM\n"), false);
      break;

    case L_FUN:
      oled_write_P(PSTR("FUN\n"), false);
      break;

    default:
      oled_write_P(PSTR("?????\n"), false);
  }
}

struct rgb_data_t {
  bool enabled;
  uint8_t mode;
  HSV hsv;
};

struct rgb_data_t read_rgb_data(void) {
  struct rgb_data_t rgb_data = {
    rgb_matrix_is_enabled(),
    rgb_matrix_get_mode(),
    rgb_matrix_get_hsv()
  };
  return rgb_data;
}

bool compare_rgb_data(struct rgb_data_t a, struct rgb_data_t b) {
  return 
    a.enabled == b.enabled &&
    a.mode    == b.mode    && 
    a.hsv.h   == b.hsv.h   &&
    a.hsv.s   == b.hsv.s   &&
    a.hsv.v   == b.hsv.v;
}

void render_rgb_data(struct rgb_data_t rgb_data) {
  oled_write_ln_P(PSTR("RGB"), rgb_data.enabled);
  render_space();

  char mode_str[4];
  sprintf(mode_str, "%3d", rgb_data.mode);

  oled_write_P(PSTR("M:"), false);
  oled_write_P(PSTR(mode_str), false);
  render_space();

  char hue_str[4];
  char sat_str[4];
  char val_str[4];

  sprintf(hue_str, "%3d", rgb_data.hsv.h);
  sprintf(sat_str, "%3d", rgb_data.hsv.s);
  sprintf(val_str, "%3d", rgb_data.hsv.v);

  oled_write_P(PSTR("H:"), false);
  oled_write_P(PSTR(hue_str), false);
  oled_write_P(PSTR("S:"), false);
  oled_write_P(PSTR(sat_str), false);
  oled_write_P(PSTR("V:"), false);
  oled_write_P(PSTR(val_str), false);
  render_space();
}

bool oled_task_user(void) {
  if (is_keyboard_master()) {
    render_logo();
    render_logo_text();
    render_space();
    render_space();
    render_layer_state_user();
    render_space();
    render_space();
    render_mod_status_gui_alt(get_mods() | get_oneshot_mods());
    render_mod_status_ctrl_shift(get_mods() | get_oneshot_mods());
  }
  else {
    static struct rgb_data_t cur_rgb_data = {};
    static uint32_t rgb_data_timer = 0;

    struct rgb_data_t new_rgb_data = read_rgb_data();
    if (!compare_rgb_data(cur_rgb_data, new_rgb_data)) {
      rgb_data_timer = timer_read32() + 10000;
      cur_rgb_data = new_rgb_data;
      oled_clear();
    }

    if (timer_expired32(timer_read32(), rgb_data_timer)) {
      return true;
    }
    
    render_logo();
    render_logo_text();
    render_space();
    render_space();
    render_rgb_data(cur_rgb_data);
  }

  return false;
}