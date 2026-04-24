#ifndef LCD_PRINTF_H_
#define LCD_PRINTF_H_

#include <stdarg.h>

// Print formatted data from variable argument list to LCD.
int lcd_vprintf(const char* fmt, va_list args);

// Print formatted data to LCD.
int lcd_printf(const char* fmt, ...);

#endif // LCD_PRINTF_H_
