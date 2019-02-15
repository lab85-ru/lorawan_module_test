#ifndef LCD_H
#define LCD_H

int lcd_init(void);
void lcd_refresh(void);
void lcd_print(uint8_t x, uint8_t y, char * s, uint8_t r);
void lcd_cls(void);




#endif