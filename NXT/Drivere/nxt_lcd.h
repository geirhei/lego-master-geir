#ifndef __NXT_LCD_H__
#  define __NXT_LCD_H__

#  define NXT_LCD_WIDTH 100
#  define NXT_LCD_DEPTH 8

void nxt_lcd_init(unsigned char *disp);
void nxt_lcd_power_up(void);
void nxt_lcd_power_down(void);
void nxt_lcd_update(void);
void nxt_lcd_force_update(void);
void nxt_lcd_command(unsigned char cmd);
void nxt_lcd_set_col(unsigned long coladdr);
void nxt_lcd_set_multiplex_rate(unsigned long mr);
void nxt_lcd_set_temp_comp(unsigned long tc);
void nxt_lcd_set_panel_loading(unsigned long hi);
void nxt_lcd_set_pump_control(unsigned long pc);
void nxt_lcd_set_scroll_line(unsigned long sl);
void nxt_lcd_set_page_address(unsigned long pa);
void nxt_lcd_set_pot(unsigned long pot);
void nxt_lcd_set_ram_address_control(unsigned long ac);
void nxt_lcd_set_frame_rate(unsigned long fr);
void nxt_lcd_set_all_pixels_on(unsigned long on);
void nxt_lcd_inverse_display(unsigned long on);
void nxt_lcd_enable(unsigned long on);
void nxt_lcd_set_map_control(unsigned long map_control);
void nxt_lcd_reset(void);
void nxt_lcd_set_bias_ratio(unsigned long ratio);

void nxt_lcd_set_cursor_update(unsigned long on);

void nxt_lcd_force_update();

#endif
