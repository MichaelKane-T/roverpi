#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"

#include "lcd.h"
#include "lcd_printf.h"

#define FONT_SZ 1
#define FONT_W (LCD_CHAR_W*FONT_SZ)
#define FONT_H (LCD_CHAR_H*FONT_SZ)
#define FONT_CLR WHITE
#define FONT_BKG BLACK

#define COLS (LCD_W/FONT_W)
#define ROWS (LCD_H/FONT_H)

#define BUF_SZ 128

static SemaphoreHandle_t sema_h;
static StaticSemaphore_t sema_buf;
static coord_t xpos, ypos;


// Print formatted data from variable argument list to LCD.
int lcd_vprintf(const char* fmt, va_list args)
{
	char buf[BUF_SZ];
	char *b, *s;
	coord_t len;

	int cnt = vsnprintf(buf, BUF_SZ, fmt, args);

	if (sema_h == NULL) sema_h = xSemaphoreCreateMutexStatic(&sema_buf);
	xSemaphoreTake(sema_h, portMAX_DELAY);
	lcd_setFontSize(FONT_SZ);
	lcd_setFontBackground(FONT_BKG);
	for (b = buf; (s = strpbrk(b, "\r\n")); b = s+1) { // Break into lines
		char cc = *s; // Control character
		len = s - b;
		if (len) { // Length greater than zero
			*s = '\0'; // Replace control char with NULL character
			coord_t xs = xpos * FONT_W;
			coord_t ys = ypos * FONT_H;
			lcd_drawString(xs, ys, b, FONT_CLR);
		}
		xpos = 0;
		if (cc == '\n') {
			if (++ypos >= ROWS) ypos = 0;
			// Clear next line
			lcd_fillRect(0, ypos * FONT_H, LCD_W, FONT_H, FONT_BKG);
		}
	}
	len = strlen(b);
	if (len) { // Last segment without a newline.
		coord_t xs = xpos * FONT_W;
		coord_t ys = ypos * FONT_H;
		lcd_drawString(xs, ys, b, FONT_CLR);
		xpos += len;
	}
	xSemaphoreGive(sema_h);
	return cnt;
}

// Print formatted data to LCD.
int lcd_printf(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	int cnt = lcd_vprintf(fmt, args);
	va_end(args);
	return cnt;
}
