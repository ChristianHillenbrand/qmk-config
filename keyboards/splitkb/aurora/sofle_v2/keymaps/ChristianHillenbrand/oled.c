#include QMK_KEYBOARD_H

#include "layers.h"

extern void render_space(void);
extern void render_logo(void);
extern void render_logo_text(void);

void render_line(void) {
  static const char PROGMEM line[] = {0xD1, 0xD1, 0xD1, 0xD1, 0xD1, 0};
  oled_write_P(line, false);
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
  static const char PROGMEM shift_off_1[] = {0x89, 0x8a, 0};
  static const char PROGMEM shift_off_2[] = {0xa9, 0xaa, 0};
  static const char PROGMEM shift_on_1[] = {0x91, 0x92, 0};
  static const char PROGMEM shift_on_2[] = {0xb1, 0xb2, 0};
    
  static const char PROGMEM ctrl_off_1[] = {0x8b, 0x8c, 0};
  static const char PROGMEM ctrl_off_2[] = {0xab, 0xac, 0};
  static const char PROGMEM ctrl_on_1[] = {0xcd, 0xce, 0};
  static const char PROGMEM ctrl_on_2[] = {0xcf, 0xd0, 0};

  static const char PROGMEM off_off_1[] = {0xc5, 0};
  static const char PROGMEM off_off_2[] = {0xc6, 0};
  static const char PROGMEM on_off_1[] = {0xc7, 0};
  static const char PROGMEM on_off_2[] = {0xc8, 0};
  static const char PROGMEM off_on_1[] = {0xc9, 0};
  static const char PROGMEM off_on_2[] = {0xca, 0};
  static const char PROGMEM on_on_1[] = {0xcb, 0};
  static const char PROGMEM on_on_2[] = {0xcc, 0};

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
  static const char PROGMEM alt_off_1[] = {0x85, 0x86, 0};
  static const char PROGMEM alt_off_2[] = {0xa5, 0xa6, 0};
  static const char PROGMEM alt_on_1[] = {0x8d, 0x8e, 0};
  static const char PROGMEM alt_on_2[] = {0xad, 0xae, 0};
    
  static const char PROGMEM gui_off_1[] = {0x87, 0x88, 0};
  static const char PROGMEM gui_off_2[] = {0xa7, 0xa8, 0};
  static const char PROGMEM gui_on_1[] = {0x8f, 0x90, 0};
  static const char PROGMEM gui_on_2[] = {0xaf, 0xb0, 0};

  static const char PROGMEM off_off_1[] = {0xc5, 0};
  static const char PROGMEM off_off_2[] = {0xc6, 0};
  static const char PROGMEM on_off_1[] = {0xc7, 0};
  static const char PROGMEM on_off_2[] = {0xc8, 0};
  static const char PROGMEM off_on_1[] = {0xc9, 0};
  static const char PROGMEM off_on_2[] = {0xca, 0};
  static const char PROGMEM on_on_1[] = {0xcb, 0};
  static const char PROGMEM on_on_2[] = {0xcc, 0};

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

bool render_central(void) {
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