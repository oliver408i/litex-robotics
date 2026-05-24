#include "lcd.h"

#include <system.h>
#include <generated/csr.h>

/* The pads_ctrl register holds two firmware-driven bits (reset_n,
 * backlight). CS and DC are owned by the engine FSM and not exposed
 * here. We keep a shadow word so set-one-bit operations don't have to
 * read-modify-write the live CSR. */
static uint32_t lcd_pads_state = LCD_PADS_RESET_N;

void lcd_pads_apply(void)
{
	lcd_pads_ctrl_write(lcd_pads_state);
}

void lcd_pads_set(uint32_t mask, int value)
{
	if(value) lcd_pads_state |=  mask;
	else      lcd_pads_state &= ~mask;
	lcd_pads_apply();
}

void lcd_reset_n(int v)   { lcd_pads_set(LCD_PADS_RESET_N,   v); }
void lcd_backlight(int v) { lcd_pads_set(LCD_PADS_BACKLIGHT, v); }

void lcd_wait_idle(void)
{
	while(lcd_status_read() & LCD_BUSY)
		;
}

void lcd_wait_can_accept(void)
{
	while(!(lcd_status_read() & LCD_CAN_ACCEPT))
		;
}

void lcd_write_cmd(uint8_t cmd)
{
	lcd_wait_can_accept();
	lcd_cmd_byte_write(cmd);
	lcd_op_write(LCD_OP_CMD);
}

void lcd_cmd_data(uint8_t cmd, const uint8_t *data, unsigned int len)
{
	lcd_wait_can_accept();
	lcd_cmd_byte_write(cmd);
	if(len) {
		lcd_dma_src_write((uint32_t)data);
		lcd_dma_row_bytes_write(len);
		lcd_dma_row_count_write(1);
		lcd_op_write(LCD_OP_CMD_DATA_DMA);
	} else {
		lcd_op_write(LCD_OP_CMD);
	}
}

void lcd_hw_reset(void)
{
	lcd_pads_state = LCD_PADS_RESET_N;
	lcd_pads_apply();
	busy_wait(20);
	lcd_reset_n(0);
	busy_wait(20);
	lcd_reset_n(1);
	busy_wait(120);
}

void lcd_init(void)
{
	/* ST7796S bring-up. Sequence is panel-vendor recipe; the values
	 * (gamma, power, VCOM) are calibrated for the module we use. */
	static const uint8_t unlock1[]   = {0xc3};
	static const uint8_t unlock2[]   = {0x96};
	static const uint8_t madctl[]    = {0x48};
	static const uint8_t pixfmt[]    = {0x55};
	static const uint8_t invctr[]    = {0x01};
	static const uint8_t entry[]     = {0xc6};
	static const uint8_t pwr1[]      = {0x80, 0x45};
	static const uint8_t pwr2[]      = {0x13};
	static const uint8_t pwr3[]      = {0xa7};
	static const uint8_t vcom[]      = {0x0a};
	static const uint8_t display[]   = {0x80};
	static const uint8_t pos_gamma[] = {
		0xf0, 0x09, 0x0b, 0x06, 0x04, 0x15, 0x2f,
		0x54, 0x42, 0x3c, 0x17, 0x14, 0x18, 0x1b
	};
	static const uint8_t neg_gamma[] = {
		0xe0, 0x09, 0x0b, 0x06, 0x04, 0x03, 0x2b,
		0x43, 0x42, 0x3b, 0x16, 0x14, 0x17, 0x1b
	};
	static const uint8_t lock1[] = {0x3c};
	static const uint8_t lock2[] = {0x69};

	lcd_hw_reset();
	lcd_write_cmd(0x01); busy_wait(120);

	lcd_cmd_data(0xf0, unlock1,   sizeof(unlock1));
	lcd_cmd_data(0xf0, unlock2,   sizeof(unlock2));
	lcd_cmd_data(0x36, madctl,    sizeof(madctl));
	lcd_cmd_data(0x3a, pixfmt,    sizeof(pixfmt));
	lcd_cmd_data(0xb4, invctr,    sizeof(invctr));
	lcd_cmd_data(0xb7, entry,     sizeof(entry));
	lcd_cmd_data(0xc0, pwr1,      sizeof(pwr1));
	lcd_cmd_data(0xc1, pwr2,      sizeof(pwr2));
	lcd_cmd_data(0xc2, pwr3,      sizeof(pwr3));
	lcd_cmd_data(0xc5, vcom,      sizeof(vcom));
	lcd_cmd_data(0xb6, display,   sizeof(display));
	lcd_cmd_data(0xe0, pos_gamma, sizeof(pos_gamma));
	lcd_cmd_data(0xe1, neg_gamma, sizeof(neg_gamma));
	lcd_cmd_data(0xf0, lock1,     sizeof(lock1));
	lcd_cmd_data(0xf0, lock2,     sizeof(lock2));

	lcd_write_cmd(0x11); busy_wait(120);   /* sleep out */
	lcd_write_cmd(0x29);                   /* display on */
	lcd_wait_idle();                       /* let 0x29 reach panel */
	busy_wait(20);
	lcd_backlight(1);
}

/* Pack a 16-bit panel coord pair {hi, lo} into one CSR write. */
static inline uint32_t pack_coords(uint16_t lo, uint16_t hi)
{
	return ((uint32_t)hi << 16) | (uint32_t)lo;
}

void lcd_dma_rect(int16_t x, int16_t y, int16_t w, int16_t h,
                  const void *buf, uint32_t len)
{
	uint16_t x1 = (uint16_t)(x + w - 1);
	uint16_t y1 = (uint16_t)(y + h - 1);

	lcd_wait_can_accept();
	lcd_rect_x_write(pack_coords((uint16_t)x, x1));
	lcd_rect_y_write(pack_coords((uint16_t)y, y1));
	lcd_dma_src_write((uint32_t)buf);
	lcd_dma_row_bytes_write(len);
	lcd_dma_row_count_write(1);
	lcd_op_write(LCD_OP_DMA_RECT);
}

void lcd_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	uint16_t x1 = (uint16_t)(x + w - 1);
	uint16_t y1 = (uint16_t)(y + h - 1);

	lcd_wait_can_accept();
	lcd_rect_x_write(pack_coords((uint16_t)x, x1));
	lcd_rect_y_write(pack_coords((uint16_t)y, y1));
	lcd_fill_color_write(color);
	lcd_fill_count_write((uint32_t)w * (uint32_t)h);
	lcd_op_write(LCD_OP_FILL_RECT);
}
