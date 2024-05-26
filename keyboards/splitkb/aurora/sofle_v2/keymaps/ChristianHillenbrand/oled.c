#include QMK_KEYBOARD_H

#include "layers.h"

extern void render_space(void);
extern void render_logo(void);
extern void render_logo_text(void);

void render_line(void) {
  oled_write_P(PSTR("-----"), false);
}

oled_rotation_t oled_init_user(oled_rotation_t rotation) {
  if (is_keyboard_master()) {
    return OLED_ROTATION_0;
  }
  return rotation;
}

void render_default_layer(void) {
  switch (get_highest_layer(default_layer_state)) {
    case L_QWRTY:
      oled_write_P(PSTR("QWRTY"), false);
      break;

    case L_COLMK:
      oled_write_P(PSTR("COLMK"), false);
      break;

    default:
      oled_write_P(PSTR("?????"), false);
      break;
  }
}

void render_layer(void) {
  oled_write_P(PSTR("LAYER"), false);
  render_space();
  switch (get_highest_layer(layer_state)) {
    case L_QWRTY:
    case L_COLMK:
      render_default_layer();
      break;

    case L_NAV:
      oled_write_P(PSTR(" NAV "), false);
      break;

    case L_NUM:
      oled_write_P(PSTR(" NUM "), false);
      break;

    case L_FUN:
      oled_write_P(PSTR(" FUN "), false);
      break;

    default:
      oled_write_P(PSTR("?????"), false);
      break;
  }
}

void render_mod_status_shift_ctrl(uint8_t modifiers) {
  static const char PROGMEM shift_off_1[] = {0x80, 0x81, 0};
  static const char PROGMEM ctrl_off_1[]  = {0x82, 0x83, 0};
  static const char PROGMEM shift_off_2[] = {0x84, 0x85, 0};
  static const char PROGMEM ctrl_off_2[]  = {0x86, 0x87, 0};

  static const char PROGMEM shift_on_1[] = {0x88, 0x89, 0};
  static const char PROGMEM ctrl_on_1[]  = {0x8a, 0x8b, 0};
  static const char PROGMEM shift_on_2[] = {0x8c, 0x8d, 0};
  static const char PROGMEM ctrl_on_2[]  = {0x8e, 0x8f, 0};

  static const char PROGMEM off_off_1[] = {0x01, 0};
  static const char PROGMEM off_off_2[] = {0x02, 0};
  static const char PROGMEM on_off_1[]  = {0x03, 0};
  static const char PROGMEM on_off_2[]  = {0x04, 0};
  static const char PROGMEM off_on_1[]  = {0x05, 0};
  static const char PROGMEM off_on_2[]  = {0x06, 0};
  static const char PROGMEM on_on_1[]   = {0x07, 0};
  static const char PROGMEM on_on_2[]   = {0x08, 0};

  if(modifiers & MOD_MASK_SHIFT) {
    oled_write_P(shift_on_1, false);
  } else {
    oled_write_P(shift_off_1, false);
  }

  if ((modifiers & MOD_MASK_SHIFT) && (modifiers & MOD_MASK_CTRL)) {
    oled_write_P(on_on_1, false);
  } else if(modifiers & MOD_MASK_SHIFT) {
    oled_write_P(on_off_1, false);
  } else if(modifiers & MOD_MASK_CTRL) {
    oled_write_P(off_on_1, false);
  } else {
    oled_write_P(off_off_1, false);
  }

  if(modifiers & MOD_MASK_CTRL) {
    oled_write_P(ctrl_on_1, false);
  } else {
    oled_write_P(ctrl_off_1, false);
  }

  if(modifiers & MOD_MASK_SHIFT) {
    oled_write_P(shift_on_2, false);
  } else {
    oled_write_P(shift_off_2, false);
  }

  if ((modifiers & MOD_MASK_SHIFT) && (modifiers & MOD_MASK_CTRL)) {
    oled_write_P(on_on_2, false);
  } else if(modifiers & MOD_MASK_SHIFT) {
    oled_write_P(on_off_2, false);
  } else if(modifiers & MOD_MASK_CTRL) {
    oled_write_P(off_on_2, false);
  } else {
    oled_write_P(off_off_2, false);
  }

  if(modifiers & MOD_MASK_CTRL) {
    oled_write_P(ctrl_on_2, false);
  } else {
    oled_write_P(ctrl_off_2, false);
  }
}

void render_mod_status_alt_gui(uint8_t modifiers) {
  static const char PROGMEM alt_off_1[] = {0x90, 0x91, 0};
  static const char PROGMEM gui_off_1[] = {0x92, 0x93, 0};
  static const char PROGMEM alt_off_2[] = {0x94, 0x95, 0};
  static const char PROGMEM gui_off_2[] = {0x96, 0x97, 0};

  static const char PROGMEM alt_on_1[] = {0x98, 0x99, 0};
  static const char PROGMEM gui_on_1[] = {0x9a, 0x9b, 0};
  static const char PROGMEM alt_on_2[] = {0x9c, 0x9d, 0};
  static const char PROGMEM gui_on_2[] = {0x9e, 0x9f, 0};

  static const char PROGMEM off_off_1[] = {0x01, 0};
  static const char PROGMEM off_off_2[] = {0x02, 0};
  static const char PROGMEM on_off_1[]  = {0x03, 0};
  static const char PROGMEM on_off_2[]  = {0x04, 0};
  static const char PROGMEM off_on_1[]  = {0x05, 0};
  static const char PROGMEM off_on_2[]  = {0x06, 0};
  static const char PROGMEM on_on_1[]   = {0x07, 0};
  static const char PROGMEM on_on_2[]   = {0x08, 0};

  if(modifiers & MOD_MASK_ALT) {
    oled_write_P(alt_on_1, false);
  } else {
    oled_write_P(alt_off_1, false);
  }

  if ((modifiers & MOD_MASK_ALT) && (modifiers & MOD_MASK_GUI)) {
    oled_write_P(on_on_1, false);
  } else if(modifiers & MOD_MASK_ALT) {
    oled_write_P(on_off_1, false);
  } else if(modifiers & MOD_MASK_GUI) {
    oled_write_P(off_on_1, false);
  } else {
    oled_write_P(off_off_1, false);
  }

  if(modifiers & MOD_MASK_GUI) {
    oled_write_P(gui_on_1, false);
  } else {
    oled_write_P(gui_off_1, false);
  }

  if(modifiers & MOD_MASK_ALT) {
    oled_write_P(alt_on_2, false);
  } else {
    oled_write_P(alt_off_2, false);
  }

  if ((modifiers & MOD_MASK_ALT) && (modifiers & MOD_MASK_GUI)) {
    oled_write_P(on_on_2, false);
  } else if(modifiers & MOD_MASK_ALT) {
    oled_write_P(on_off_2, false);
  } else if(modifiers & MOD_MASK_GUI) {
    oled_write_P(off_on_2, false);
  } else {
    oled_write_P(off_off_2, false);
  }

  if(modifiers & MOD_MASK_GUI) {
    oled_write_P(gui_on_2, false);
  } else {
    oled_write_P(gui_off_2, false);
  }
}

struct rgb_data_t {
  bool enable;
  uint8_t mode;
  HSV hsv;
  uint8_t speed;
};

struct rgb_data_t read_rgb_data(void) {
  struct rgb_data_t rgb_data = {
    rgb_matrix_is_enabled(),
    rgb_matrix_get_mode(),
    rgb_matrix_get_hsv(),
    rgb_matrix_get_speed()
  };
  return rgb_data;
}

void render_rgb_data(void) {
  struct rgb_data_t rgb_data = read_rgb_data();

  oled_write_ln_P(PSTR("RGB"), rgb_data.enable);

  render_space();

  char mode_str[4];
  sprintf(mode_str, "%3d", rgb_data.mode);

  oled_write_P(PSTR("M:"), false);
  oled_write_P(PSTR(mode_str), false);

  render_space();
  render_line();
  render_space();

  char hue_str[4];
  char sat_str[4];
  char val_str[4];

  sprintf(hue_str, "%3d", rgb_data.hsv.h);
  sprintf(sat_str, "%3d", rgb_data.hsv.s);
  sprintf(val_str, "%3d", rgb_data.hsv.v);

  oled_write_P(PSTR("H:"), false);
  oled_write_P(PSTR(hue_str), false);
  render_space();
  oled_write_P(PSTR("S:"), false);
  oled_write_P(PSTR(sat_str), false);
  render_space();
  oled_write_P(PSTR("V:"), false);
  oled_write_P(PSTR(val_str), false);

  render_space();
  render_line();
  render_space();

  char speed_str[4];
  sprintf(speed_str, "%3d", rgb_data.speed);

  oled_write_P(PSTR("S:"), false);
  oled_write_P(PSTR(speed_str), false);
}

void render_wpm(void) {
  static char wpm[10];

  oled_set_cursor(0, 0);
  sprintf(wpm, "WPM: %-3d", get_current_wpm());
  oled_write(PSTR(wpm), false);
}

#define FRAME_DURATION 100

#define NUM_IDLE_FRAMES 5
#define NUM_PREP_FRAMES 1
#define NUM_TAP_FRAMES 2

#define CAT_ROWS 4
#define CAT_COLS 8

#define CAT_POS_X 8
#define CAT_POS_Y 0

void render_idle_cat(void) {
  static uint8_t idle_frame = 0;
  static uint32_t frame_timer = 0;

  static const char PROGMEM idle_frames[NUM_IDLE_FRAMES][CAT_ROWS][CAT_COLS] = {
    {
      {0x20, 0xd4, 0xb3, 0xc5, 0xc8, 0xdc, 0},
      {0xb5, 0xcf, 0xd3, 0x20, 0xa9, 0xa0, 0xb1, 0},
      {0xa8, 0xd9, 0xc1, 0xa3, 0xbb, 0xc9, 0xbd, 0},
      {0x20, 0xcd, 0xc4, 0xdd, 0x20, 0x20, 0x20, 0}
    },

    {
      {0xa6, 0xd4, 0xb3, 0xc6, 0xc0, 0xdc, 0},
      {0xb4, 0xcf, 0xd3, 0x20, 0xa9, 0xa0, 0xb1, 0},
      {0xa8, 0xd9, 0xc1, 0xa3, 0xbb, 0xc9, 0xbd, 0},
      {0x20, 0xcd, 0xc4, 0xdd, 0x20, 0x20, 0x20, 0}
    },

    {
      {0x20, 0xd5, 0xae, 0xbf, 0xb7, 0x20, 0},
      {0xaf, 0xcb, 0x20, 0x20, 0xd6, 0xa5, 0xad, 0},
      {0xa7, 0xd8, 0xb8, 0xa2, 0xba, 0xc7, 0xbd, 0},
      {0x20, 0xcd, 0xc4, 0xdd, 0x20, 0x20, 0x20, 0}
    },

    {
      {0x20, 0xd4, 0xb3, 0xbe, 0xc3, 0xdc, 0},
      {0xb6, 0xcF, 0xd3, 0x20, 0xa9, 0xa0, 0xb1, 0},
      {0xa1, 0xd9, 0xc1, 0xa3, 0xbb, 0xc9, 0xbd, 0},
      {0x20, 0xcd, 0xc4, 0xDD, 0x20, 0x20, 0x20, 0}
    },

    {
      {0x20, 0xd4, 0xb3, 0xbe, 0xc3, 0xdc, 0},
      {0xb6, 0xcf, 0xd3, 0x20, 0xa9, 0xa0, 0xb1, 0},
      {0xa1, 0xd9, 0xc1, 0xa3, 0xbb, 0xc9, 0xbd, 0},
      {0x20, 0xcd, 0xC4, 0xdd, 0x20, 0x20, 0x20, 0}
    }
  };

  if (timer_elapsed32(frame_timer) > FRAME_DURATION) {
    for (uint8_t i = 0; i < CAT_ROWS; i++) {
      oled_set_cursor(CAT_POS_X, CAT_POS_Y + i);
      oled_write_P(idle_frames[idle_frame][i], false);
    }

    idle_frame = (idle_frame + 1) % NUM_IDLE_FRAMES;
    frame_timer = timer_read32();
  }
}

bool render_central(void) {
  render_wpm();
  render_idle_cat();
  return false;


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

bool render_peripheral(void) {
  if (get_highest_layer(layer_state) != L_NAV) {
    return true;
  }

  oled_clear();
  render_space();
  render_rgb_data();
  return false;
}

bool oled_task_user(void) {
  if (is_keyboard_master()) {
    return render_central();
  }
  else {
    return render_peripheral();
  }
}