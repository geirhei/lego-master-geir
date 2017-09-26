#include "nxt_lcd.h"
#include "nxt_spi.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

static unsigned char *display = (unsigned char *)0;

void nxt_lcd_command(unsigned char cmd)
{
  unsigned char tmp = cmd;

  nxt_spi_write(0, &tmp, 1);
}

void nxt_lcd_set_col(unsigned long coladdr)
{
  nxt_lcd_command(0x00 | (coladdr & 0xF));
  nxt_lcd_command(0x10 | ((coladdr >> 4) & 0xF));
}

void nxt_lcd_set_multiplex_rate(unsigned long mr)
{
  nxt_lcd_command(0x20 | (mr & 3));
}

void nxt_lcd_set_temp_comp(unsigned long tc)
{
  nxt_lcd_command(0x24 | (tc & 3));
}

void nxt_lcd_set_panel_loading(unsigned long hi)
{
  nxt_lcd_command(0x28 | ((hi) ? 1 : 0));
}

void nxt_lcd_set_pump_control(unsigned long pc)
{
  nxt_lcd_command(0x2c | (pc & 3));
}

void nxt_lcd_set_scroll_line(unsigned long sl)
{
  nxt_lcd_command(0x40 | (sl & 0x3f));
}

void nxt_lcd_set_page_address(unsigned long pa)
{
  nxt_lcd_command(0xB0 | (pa & 0xf));
}

void nxt_lcd_set_pot(unsigned long pot)
{
  nxt_lcd_command(0x81);
  nxt_lcd_command(pot & 0xff);
}

void
nxt_lcd_set_ram_address_control(unsigned long ac)
{
  nxt_lcd_command(0x88 | (ac & 7));
}

void nxt_lcd_set_frame_rate(unsigned long fr)
{
  nxt_lcd_command(0xA0 | (fr & 1));
}

void nxt_lcd_set_all_pixels_on(unsigned long on)
{
  nxt_lcd_command(0xA4 | ((on) ? 1 : 0));
}

void nxt_lcd_inverse_display(unsigned long on)
{
  nxt_lcd_command(0xA6 | ((on) ? 1 : 0));
}

void nxt_lcd_enable(unsigned long on)
{
  nxt_lcd_command(0xAE | ((on) ? 1 : 0));
}

void nxt_lcd_set_map_control(unsigned long map_control)
{
  nxt_lcd_command(0xC0 | ((map_control & 3) << 1));
}

void nxt_lcd_reset(void)
{
  nxt_lcd_command(0xE2);
}

void nxt_lcd_set_bias_ratio(unsigned long ratio)
{
  nxt_lcd_command(0xE8 | (ratio & 3));
}

void nxt_lcd_set_cursor_update(unsigned long on)
{
  nxt_lcd_command(0xEE | ((on) ? 1 : 0));
}

void nxt_lcd_force_update(void)
{
  // Update the screen the slow way. Works with interrupts disabled
  int i;
  unsigned char *disp = display;

  for (i = 0; i < NXT_LCD_DEPTH; i++) {
    nxt_lcd_set_col(0);
    nxt_lcd_set_page_address(i);

    nxt_spi_write(1, disp, NXT_LCD_WIDTH);
    disp += NXT_LCD_WIDTH;
  }
}


void nxt_lcd_update(void)
{
#define DMA_REFRESH
#ifdef DMA_REFRESH
  nxt_spi_refresh();
#else
  nxt_lcd_force_update();
#endif
}

void nxt_lcd_power_up(void)
{
  int i;
  
  for(i=0;i<500000;++i) {
    __no_operation();
  }
  //vTaskDelay(xDelay);
  nxt_lcd_reset();
  for(i=0;i<500000;++i) {
    __no_operation();
  }
  nxt_lcd_set_multiplex_rate(3);	// 1/65
  nxt_lcd_set_bias_ratio(3);	// 1/9
  nxt_lcd_set_pot(0x60);	// ?? 9V??

  nxt_lcd_set_ram_address_control(1); // auto wrap
  nxt_lcd_set_map_control(0x02); // mirror in y

  nxt_spi_set_display(display);
  nxt_lcd_enable(1);

}

void nxt_lcd_power_down(void)
{
  nxt_lcd_reset();
}

void nxt_lcd_init(unsigned char *disp)
{
  display = disp;
  nxt_spi_init();

  nxt_lcd_power_up();

}
