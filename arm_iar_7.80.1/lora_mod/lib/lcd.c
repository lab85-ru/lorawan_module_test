// LCD Nokia 5110


#include "stm32f0xx_hal.h"
#include "lcd.h"
#include <string.h>
#include "font5x7.h"

extern SPI_HandleTypeDef hspi1;

static const uint8_t LCD_CMD_INIT[] = { 
                    0x21, // расширенный набор команд
                    0x13, // смещения напряжения
                    0x04, // установить температурную коррекцию командой
                    0xb8, // включить генератор повышенного напряжения на уровень 6,42 В командой
                    0x20, // вернуться в стандартный набор команд, послав 0x20
                    0x0c, // включить графический режим командой
    };
    
static const uint8_t LCD_CMD_X0Y0[] = {0x80, 0x40};


static uint8_t lcd_buf[ 84 * (48 / 8) ]; // 84 * 6 байт


//==============================================================================
// Вывод строки на экран в позицию x,y
// Для фонта 5x7
// колличество символов 16x6
//
// x - положение по горизонтали, значение 0-15
// y - номер строки, значения 0-5 
// s - указатель на строку TXT
// r - Флаг вывода буфера на ЖКИ(при r != 0) 
//==============================================================================
#define LCD57_X_MAX (16)
#define LCD57_Y_MAX (6)
#define LCD57_X_STEP (5)
#define LCD_X_POS_MASK (0x7f)
#define LCD_Y_POS_MASK (0x07)
void lcd_print(uint8_t x, uint8_t y, char * s, uint8_t r)
{
    int i;

    uint8_t pos_x = (x <= LCD57_X_MAX-1 ? x : 0) * LCD57_X_STEP;
    uint8_t pos_y = (y <= LCD57_Y_MAX-1 ? y :0);

    uint8_t *index = &lcd_buf[pos_x + pos_y * 84];
    uint8_t *lcd_end = lcd_buf + sizeof(lcd_buf);
    
    while(*s != '\0'){
        for (i=0; i<5; i++){
          if (index != lcd_end){
              if (*s >= 32) // не отображать символы с кодами меньше 32
                  *index = Font5x7[ (*s - 32) * 5 + i];
              else
                  *index = 0;
              index++;
          }
        }
        s++;
    }
  
    if (r) lcd_refresh();
}

//==============================================================================
// Обновление экрана (целиком)
//==============================================================================
void lcd_refresh(void)
{
    HAL_GPIO_WritePin(GPIOA, LCD_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit( &hspi1, (uint8_t*)LCD_CMD_X0Y0, sizeof(LCD_CMD_X0Y0), 1000);

    HAL_GPIO_WritePin(GPIOA, LCD_DC_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit( &hspi1, lcd_buf, sizeof(lcd_buf), 1000);
}

//==============================================================================
// Init LCD
//==============================================================================
int lcd_init(void)
{
    HAL_StatusTypeDef res;

    HAL_GPIO_WritePin(GPIOA, LCD_DC_Pin, GPIO_PIN_RESET);
    res = HAL_SPI_Transmit( &hspi1, (uint8_t*)LCD_CMD_INIT, sizeof(LCD_CMD_INIT), 1000);
    if (res != HAL_OK) return 1;
    
    memset(lcd_buf, 0x00, sizeof(lcd_buf));
    
    lcd_refresh();

    return 0;
}

//==============================================================================
// Очистка LCD
//==============================================================================
void lcd_cls(void)
{
    memset(lcd_buf, 0x00, sizeof(lcd_buf));
    lcd_refresh();
}

