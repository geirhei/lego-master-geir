#ifndef __DISPLAY_H__
#  define __DISPLAY_H__

void display_init(void);

void display_update(void);

void display_force_update(void);

void display_clear(unsigned long updateToo);

void display_goto_xy(int x, int y);

void display_char(int c);

void display_string(const char *str);

void display_int(int val, unsigned long places);
void display_hex(unsigned long val, unsigned long places);

void display_unsigned(unsigned long val, unsigned long places);

void display_bitmap_copy(const unsigned char *data, unsigned long width, unsigned long depth, unsigned long x, unsigned long y);

void display_test(void);

unsigned char *display_get_buffer(void);

void display_set_auto_update(int);

extern int display_tick;
extern int display_auto_update;

#endif
