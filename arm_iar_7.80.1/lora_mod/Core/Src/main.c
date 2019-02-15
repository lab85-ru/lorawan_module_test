
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "lcd.h"
#include "queue_buf.h"

#define DELAY_100MS (100)

#define QUEUE_BUF_RX_SIZE (128)
queue_buf_t lora_rx_q;
uint8_t lora_rx_buf[ QUEUE_BUF_RX_SIZE ];
#define STR_SIZE (128)
char str[ STR_SIZE ];

const char PACK_HEADER_TX[] = {"AT+SEND=60111,7,P="};
const char PACK_HEADER_RX[] = {"+RCV="};
char str_num[32];


typedef enum {GET_STR_NOT_FOUND=0, GET_STR_OK, GET_STR_NOT_END} get_str_result_t;
typedef enum { RESULT_MDM_P_OK=0, RESULT_MDM_P_ERROR, RESULT_MDM_P_SIMCARD } result_mdm_p_t;
typedef enum {TIR_WAIT=0, TIR_OUT=1} tir_e;                          // result return for delta intervals

#ifndef DEBUG_PROC
#define DEBUG_PROC 0
#endif

#ifndef DEBUG_TIME
#define DEBUG_TIME 0
#endif

const char RX_ERR[] = {"+ERR="};
const char CMD_AT[] = {"AT\r\n"};
const char RX_OK[]  = {"+OK\r"};

const char CMD_RESET[] ={"AT+RESET\r\n"};
//const char RX_RESET[] = {"+RESET\r"};
const char RX_RESET[] = {"RESET\r"};

const char CMD_MODE_TX[] = {"AT+MODE=0\r\n"};
//const char RX_MODE_TX[] = {"+MODE=0\r"};
const char RX_MODE_TX[] = {"+OK\r"};

const char CMD_PARAMETER[] = {"AT+PARAMETER=12,7,4,5\r\n"};
//const char RX_PARAMETER[] = {"+PARAMETER=12,7,4,5\r"};
const char RX_PARAMETER[] = {"+OK\r"};

//const char CMD_BAND[] ={"AT+BAND=868500000\r\n"};
const char CMD_BAND[] ={"AT+BAND=915000000\r\n"};
//const char RX_BAND[] ={"+BAND=868500000\r"};
const char RX_BAND[] ={"+OK\r"};

const char CMD_ADDRESS[] = {"AT+ADDRESS=60111\r\n"};
//const char RX_ADDRESS[] = {"+ADDRESS=60111\r"};
const char RX_ADDRESS[] = {"+OK\r"};

const char CMD_NETWORKID[] = {"AT+NETWORKID=5\r\n"};
//const char RX_NETWORKID[] = {"+NETWORKID=5\r"};
const char RX_NETWORKID[] = {"+OK\r"};

const char CMD_CPIN[] = {"AT+CPIN=FABC0A02EEDCAA90F0BC0002E5DCAA91\r\n"};
//const char RX_CPIN[] =    {"+CPIN=FABC0A02EEDCAA90F0BC0002E5DCAA91\r"};
const char RX_CPIN[] =    {"+OK\r"};

const char CMD_CRFOP[] = {"AT+CRFOP=15\r\n"};
//const char RX_CRFOP[] = {"+CRFOP=15\r"};
const char RX_CRFOP[] = {"+OK\r"};

typedef struct modem_init_st_t {
    const char * tx_str;                // stroka peredovaemaya komanda v modem
    const uint32_t time_out;            // vrema interval v tecenii kotorogo dolna poluchena stroka rx_str (ms)
    const char * rx_str;                // stroka prinimaemay is modema
    const struct modem_init_st_t * init_next;  // ukazatel na sleduuheu structuru IF str=rx_str
} modem_init_t;

const modem_init_t lora_init_start;
const modem_init_t m_i_reset;
const modem_init_t m_i_mode;
const modem_init_t m_i_parameter;
const modem_init_t m_i_band;
const modem_init_t m_i_address;
const modem_init_t m_i_networkid;
const modem_init_t m_i_cpin;
const modem_init_t m_i_crfop;
const modem_init_t m_i_null;



const modem_init_t lora_init_start = {
    .tx_str        = CMD_AT,
    .time_out      = 200,
    .rx_str        = RX_OK,
    .init_next     = &m_i_mode,
};

const modem_init_t m_i_reset = {
    .tx_str        = CMD_RESET,
    .time_out      = 200,
    .rx_str        = RX_RESET,
    .init_next     = &m_i_mode,
};

const modem_init_t m_i_mode = {
    .tx_str        = CMD_MODE_TX,
    .time_out      = 200,
    .rx_str        = RX_MODE_TX,
    .init_next     = &m_i_parameter,
};

const modem_init_t m_i_parameter = {
    .tx_str        = CMD_PARAMETER,
    .time_out      = 200,
    .rx_str        = RX_PARAMETER,
    .init_next     = &m_i_band,
};

const modem_init_t m_i_band = {
    .tx_str        = CMD_BAND,
    .time_out      = 200,
    .rx_str        = RX_BAND,
    .init_next     = &m_i_address,
};

const modem_init_t m_i_address = {
    .tx_str        = CMD_ADDRESS,
    .time_out      = 200,
    .rx_str        = RX_ADDRESS,
    .init_next     = &m_i_networkid,
};

const modem_init_t m_i_networkid = {
    .tx_str        = CMD_NETWORKID,
    .time_out      = 200,
    .rx_str        = RX_NETWORKID,
    .init_next     = &m_i_cpin,
};

const modem_init_t m_i_cpin = {
    .tx_str        = CMD_CPIN,
    .time_out      = 200,
    .rx_str        = RX_CPIN,
    .init_next     = &m_i_crfop,
};

const modem_init_t m_i_crfop = {
    .tx_str        = CMD_CRFOP,
    .time_out      = 200,
    .rx_str        = RX_CRFOP,
    .init_next     = &m_i_null,
};

const modem_init_t m_i_null = {
    .tx_str        = NULL,
    .time_out      = 0,
    .rx_str        = NULL,
    .init_next     = NULL,
};

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void conv_uint16_to_str(char *s, uint16_t a)
{
    uint8_t i10000,i1000,i100,i10,i1;

    i10000=0;
    i1000=0;
    i100=0;
    i10=0;
    i1=0;


    while (a >= 10000){a=a-10000;i10000++;}
    while (a >= 1000){a=a-1000;i1000++;}
    while (a >= 100){a=a-100;i100++;}
    while (a >= 10){a=a-10;i10++;}
    while (a >= 1){a=a-1;i1++;}

    *(s + 0) = i10000 | 0x30;
    *(s + 1) = i1000 | 0x30;
    *(s + 2) = i100 | 0x30;
    *(s + 3) = i10 | 0x30;
    *(s + 4) = i1 | 0x30;
    *(s + 5) = 0x00;
    
}
//******************************************************************************
// Poluchenie Statusa - skolko vremeni prohlo s poslednego sobitiya.
// Status vichislaetcha kak raznost dT = (T tekushee)-(T sobitiya(prohloe))
// dalee if dT >= INTERVAL then return TIR_OUT else return TIR_WAIT
// Time = sec,ms & etc
//
// t_start - vrema nachala otchuta
// t_delay - interval 
// t_cur   - tekushee vrema
//
//******************************************************************************
tir_e calcul_time_out( const uint32_t t_cur, const uint32_t t_start, const uint32_t t_delay )
{
	uint32_t t_delta;
	
	if (DEBUG_TIME) printf("Time start = %ul\n", t_start);
	if (DEBUG_TIME) printf("time cur   = %ul\n", t_cur);
	if (t_cur >= t_start){
		t_delta = t_cur - t_start;
	}else{
	        t_delta = (uint32_t)0xffffffffUL - t_start + t_cur;
	}
	if (DEBUG_TIME) printf("\nTIME DELTA=%ul   time interval =%ul\n", t_delta, t_delay);
	if (t_delta >= t_delay)
		return TIR_OUT;
	else
		return TIR_WAIT;
}

//******************************************************************************
// Функция задержка(Обертка)
//******************************************************************************
void delay_ms(uint32_t t)
{
    HAL_Delay(t);
}

//******************************************************************************
// Функция получения счетчика системный тиков(Обертка)
//******************************************************************************
uint32_t time_get_ms_counter(void)
{
    return HAL_GetTick();
}

//******************************************************************************
// Передача одного байта в порт (консоль)
// - теперь кладем в буфер и запускаем передачу !
//******************************************************************************
int hal_modem_write(const uint8_t *buf, const uint16_t size)
{
    HAL_UART_Transmit( &huart1, (uint8_t*)buf, size, 1000); 
    return size;
}

/*****************************************************************************
 * Read from modem to structure qbuf
 *
*****************************************************************************/
int hal_modem_read(queue_buf_t *qbuf)
{
    QUEUE_VAR_TYPE f;                               // svobodnoe mesto v que buf

    __disable_irq();
    f = get_data_size_queue( qbuf );                 // vernuli kolichestvo dnih v buffere
    __enable_irq();
    
    return (int)f;
}

//******************************************************************************
/*
* function return String+\n iz kolchevogo buffer modema
* to qbuf_in => sout(sout_size)
* function return =
* GET_STR_OK:       stroka naydena
* GET_STR_NOT_FOUND:net stroki
*
* Ot modema v bufere nahodatcha stroki vida:
* 1. \r\nOK\r\n => return: OK\n
* 2. AT\n       => return: AT\n
*/
//******************************************************************************
static get_str_result_t get_string(queue_buf_t *qbuf_in, uint8_t *sout, const uint32_t sout_size)
{
    uint8_t c;
    int res;
    int sout_count = 0;       // kolicestvo byte v vihodnoy stroke
    int j = 0;                // counter kolichestva prohodov ohidaniya stroki ot modema
    const int J_MAX = 5;      // kolichestvo prohodov na sborky stroki iz chasteu (IF pri pervom prohode stroka sout okazalas ber koda '\r' v konche)
    QUEUE_VAR_TYPE dlen;      // Dlinna danih v kolchevom buffer modema
    int flag_overflow = 0;    // flag perepolneniya Stroki STR, pri etom zabiraem stroku polnostiu i udalaem ee, vozvrahaem GET_STR_NOT_FOUND

new_j:

    if (DEBUG_PROC) printf("get_string: prohod j=%d\n",j);

    res = hal_modem_read( qbuf_in );                            // pomeshaem v kolchevoy buffer byte prinatie ot modema 
    if (DEBUG_PROC) printf("hal_modem_read size = %d bytes.\n",res);

    if (strlen((char*)sout)>0 && *(sout+strlen((char*)sout)-1) == '\r'){      // proverka stroka ne pustya + '\r' znachit ostavlaem ee kak est i novui ne dobovlaem
        if (DEBUG_PROC) printf("get_string: string gotova.\n");
        return GET_STR_OK;
    }

    if (DEBUG_PROC) dump_queue(qbuf_in->queue, qbuf_in->in, qbuf_in->out, qbuf_in->len);

    dlen = get_data_size_queue(qbuf_in);
    if (dlen == 0) goto exit;

    while (pop_data_queue_b(qbuf_in, &c) == 0){
        if (sout_count == (sout_size - 2)){                     // proverka str zapolnili polnostiu ili ostalos mesto tolko dla \r + \0 !!! PEREPOLNENIE STROKI
            flag_overflow = 1;
            if (DEBUG_PROC) printf("\nERROR: STR OVERFLOW sout_count=%d STR_SIZE=%d\n",sout_count, sout_size);
        }

        if (c == '\n')                                          // ignore this (\n) byte, ne dobovlaem v sout !
            continue;

        if (c == '\r'){                                         // spec simvol: Add (\r) k stroke sout, Stroka sout gotova ! i return this function
            if (strlen((char*)sout)){
                sout[sout_count] = c;
                if (flag_overflow > 0){                         // clear stroki (sout) t.k. ona ne pomestilas v buffer sout
                    memset(sout, 0, sout_size);
                    return GET_STR_NOT_FOUND;
                }else{
                    sout_count++;
                    return GET_STR_OK;
                }
           }else{
               continue;
           }
        }

        sout[sout_count] = c;                                   // Vse ostalnie byte -> add to sout string
        if (flag_overflow == 0)
            sout_count++;
    }//End while (pop_data_queue_b(qbuf_in, &c) == 0){

exit:

    if (DEBUG_PROC) printf("get_string: sout=%s\n sout len =%d\n", sout, strlen((char*)sout));

    // Proverka nalichiya v konche stroki '\r' ? 
    // IF net '\r' zanchit stroka poluchena ne polnostyu !
    // delaem ehe popitku poluchit danie ot modema i zaverhit stroku
    // IF koluchestvo prohodov > MAX a str bez '\r' to vozvrashaem stroku BEZ '\r'
    if ( (strlen((char*)sout) > 0 && sout[strlen((char*)sout) - 1] != '\r' && j < J_MAX) || (strlen((char*)sout) == 0 && j < J_MAX)){
        if (DEBUG_PROC) printf("get_string: new prohod j=%d\n",j);
        j++;
        delay_ms(DELAY_100MS);
        goto new_j;
    }

    if (strlen((char*)sout))
        return GET_STR_OK;
    else
        return GET_STR_NOT_FOUND;
}

/*******************************************************************************
* Automat Modem Init
* Proizvodit init modema po structure modem_init_st_t
* input *modem_init_st_t
* return 0 - OK
*        1 - ERROR
*******************************************************************************/
result_mdm_p_t modem_init(struct queue_buf_type *modem_q_rx, struct modem_init_st_t * in, uint8_t *s_in, const uint32_t s_in_size)
{
    int f_cmd_tx_send = 0;                      // flag, flag = 1 komanda peredana modemu
    int res;
    uint32_t cur_sys_time;                          // sistemnoe vrema (ms)

    if (DEBUG_PROC) printf("---------------------------------------------------\n");
    if (DEBUG_PROC) printf("Modem Init.\n");

    while(1){
        if (in->tx_str == NULL && in->rx_str == NULL && in->init_next == NULL){ // uslovie okonchaniya inita modema
            return RESULT_MDM_P_OK;              // init ok
        }
        if (in->tx_str == NULL && in->rx_str == NULL && in->init_next != NULL){ // uslovie perehoda na init_next
                in = (struct modem_init_st_t *)in->init_next;
                f_cmd_tx_send = 0;
                memset(s_in, 0, s_in_size);       // stroku obrabotali chistim
                continue;
        }
        if ((f_cmd_tx_send == 0) && (in->tx_str != NULL)){
            if (DEBUG_PROC) printf("tx_str = %s\n",in->tx_str);
            res = hal_modem_write( (const uint8_t*)in->tx_str, strlen(in->tx_str));
            if (res < 0) return RESULT_MDM_P_ERROR; // error I/O
            delay_ms(DELAY_100MS);
            f_cmd_tx_send = 1;
        }
        if (in->time_out) delay_ms(in->time_out);

        cur_sys_time = time_get_ms_counter();           // tekuhee systemnoe vrema (ms)
        while(calcul_time_out( time_get_ms_counter(), cur_sys_time, in->time_out) == TIR_WAIT){
            if (get_string( modem_q_rx, s_in, s_in_size) == GET_STR_OK){
                break;
            }
        }
        if (strlen((char*)s_in) != 0){
            if (DEBUG_PROC) printf("---------------------------------------------------\n");
            if (DEBUG_PROC) printf("get_string      = %s\n", s_in);
            if (DEBUG_PROC) printf("get_string  len = %d\n", strlen((char*)s_in));
            if (DEBUG_PROC) printf("---------------------------------------------------\n");
            if (in->rx_str == NULL){ // bezuslovniy perehod na init_next
                if (DEBUG_PROC) printf("set NEXT = init_next (rx_str==NULL)\n");
                in = (struct modem_init_st_t *)in->init_next;
                f_cmd_tx_send = 0;
                memset(s_in, 0, s_in_size); // stroku obrabotali chistim
                continue;

            }
            if (DEBUG_PROC) printf("rx_str = %s\n", in->rx_str);
            if (strstr((const char*)s_in, in->rx_str) != 0){// stroka sovpadaet
                if (DEBUG_PROC) printf("set NEXT = init_next.\n");
                in = (struct modem_init_st_t *)in->init_next;
                f_cmd_tx_send = 0;
                memset(s_in, 0, s_in_size); // stroku obrabotali chistim
                continue;
            }
            // stroka ne sovpadaet - poluchaem novuu stroku
            memset(s_in, 0, s_in_size); // stroku obrabotali chistim
        }else{ // time out & str = 0
            return RESULT_MDM_P_ERROR; // init error
        }

    }//while(1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    get_str_result_t get_str_result;
    result_mdm_p_t res;
    uint16_t wait_cnt = 0;
    uint16_t pack_cnt = 0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  
  lora_rx_q.in = 0;
  lora_rx_q.out = 0;
  lora_rx_q.queue = lora_rx_buf;
  lora_rx_q.len = QUEUE_BUF_RX_SIZE;
    
  /* USART1 interrupt Init */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  __HAL_UART_ENABLE_IT( &huart1, UART_IT_RXNE);// Receive Data register not empty interrupt

  // reset LCD + LORA Modules
  HAL_GPIO_WritePin(GPIOA, RST_MOD_LCD_L_Pin, GPIO_PIN_RESET);
  HAL_Delay(30);
  HAL_GPIO_WritePin(GPIOA, RST_MOD_LCD_L_Pin, GPIO_PIN_SET);
 
  // Включить досветку
  HAL_GPIO_WritePin(GPIOA, LCD_LED_Pin, GPIO_PIN_SET);
  
  lcd_init();
  
  lcd_print(2,0,"LORA TRX v1.0",0);

  if ( HAL_GPIO_ReadPin( GPIOA, MODTYPE_TX_L_Pin) == GPIO_PIN_RESET) lcd_print(6,1,"_TX_",0);
  else lcd_print(6,1,"_RX_",0);
  
  lcd_print(0,3,"Init...",1);
  
  HAL_Delay(500);
  lora_rx_q.in = 0;
  lora_rx_q.out = 0;
  
  res = modem_init( &lora_rx_q, (struct modem_init_st_t*)&lora_init_start, (uint8_t*)str, STR_SIZE);
  if(res != RESULT_MDM_P_OK){
      lcd_print(0,4, "ERROR 1.",1);
      while(1);
  }

  lcd_print(0,3,"            ",1); // lora init clear

  if ( HAL_GPIO_ReadPin( GPIOA, MODTYPE_TX_L_Pin) == GPIO_PIN_RESET){
      goto tx_;
  }else{
      goto rx_;    
  }
 
// RX --------------------------------------------------------------------------  
rx_:  
  wait_cnt = 0;
  lcd_print(0,3,"Message:",0);
  while(1){
      get_str_result = get_string( &lora_rx_q, (uint8_t*)str, STR_SIZE);
      wait_cnt++;

      memset(str_num, 0, sizeof(str_num));
      conv_uint16_to_str(str_num, wait_cnt);
      int i=0;
      while(str_num[i] == '0'){
          str_num[i] = ' ';
          i++;
      }
      lcd_print(0,2,"wait= ",0);
      lcd_print(5,2,str_num,1);
      
      if (get_str_result == GET_STR_NOT_END){
          lcd_print(0,5, "ERROR: str no end.",1);
      }
      
      if (get_str_result == GET_STR_OK){
          lcd_print(0,5, "         ",0); // clear error rx
          wait_cnt = 0;
          char *p = strstr(str, PACK_HEADER_RX);
          if (p != NULL ){
              wait_cnt = 0;
              memset(str_num, 0, strlen(str_num));
              memcpy(str_num, p + 13, strlen(p) - 13);
              lcd_print(0,4, str_num,0);
              lcd_print(0,5, "               ",1); // clear str ERR
              memset(str, 0, sizeof(str));
              continue;
          } // if
          
          p = strstr(str, RX_ERR);
          if (p != NULL){
              wait_cnt = 0;
              lcd_print(0,5, str,1); // print to ERR
              memset(str, 0, sizeof(str));
              memset(str_num, 0, strlen(str_num));
              continue;
          } // if

          lcd_print(0,5, "Rx string ???",1);
          memset(str, 0, sizeof(str));
          memset(str_num, 0, strlen(str_num));
          
      } // if
  }
  
// TX --------------------------------------------------------------------------
tx_:  
  while(1){
      HAL_Delay(1000);
      pack_cnt++;
      memset(str_num, 0, sizeof(str_num));
      conv_uint16_to_str(str_num, pack_cnt);
      lcd_print(0,3,"P=",0);
      lcd_print(3,3,str_num,0);
      lcd_print(9,3,">",1);
      hal_modem_write((const uint8_t*)PACK_HEADER_TX, strlen(PACK_HEADER_TX));
      hal_modem_write((const uint8_t*)str_num, strlen(str_num));
      hal_modem_write("\r\n", strlen("\r\n"));

      wait_cnt = 0;
      while(((get_str_result = get_string( &lora_rx_q, (uint8_t*)str, STR_SIZE)) != GET_STR_OK) || wait_cnt != 10){
          HAL_Delay(500);
          wait_cnt++;
      }
      
      if (get_str_result != GET_STR_OK){
          lcd_print(4,2, "ERROR TX.",1);
      }
      if (strcmp(str, RX_OK) == 0 ){
          lcd_print(9,3," ",1); // CLEAR >
      }
      
  } // while(1)



  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RST_MOD_LCD_L_Pin|LCD_DC_Pin|LCD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RST_MOD_LCD_L_Pin LCD_DC_Pin LCD_LED_Pin */
  GPIO_InitStruct.Pin = RST_MOD_LCD_L_Pin|LCD_DC_Pin|LCD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MODTYPE_TX_L_Pin */
  GPIO_InitStruct.Pin = MODTYPE_TX_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MODTYPE_TX_L_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
