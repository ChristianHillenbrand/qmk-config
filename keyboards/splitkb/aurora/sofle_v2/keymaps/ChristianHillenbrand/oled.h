#pragma once

/* inherited from kb code */
extern void render_space(void);
extern void render_logo(void);
extern void render_logo_text(void);

/* bongocat */
void render_bongocat(void);

/* luna */
void render_luna(void);
void jump_luna(void);

/* menus */
void render_line(void);
void render_layer(void);
void render_mod_status_shift_ctrl(uint8_t modifiers);
void render_mod_status_alt_gui(uint8_t modifiers);
void render_wpm(void);
void render_rgb_data(void);
