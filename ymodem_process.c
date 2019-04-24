/**
  * @file       ymodem_process.c
  * @brief      串口驱动
  * @author     redoc
  * @version    v1.0
  * @date       2019-04-04
  *
  * @note
  * [2019-04-04] <redoc> (v1.0)
  * 初始版本
  *
  * @remark
  */
#define LOG_TAG   "ymd.pro"  
#define LOG_LVL   ELOG_LVL_DEBUG 

/* Includes ------------------------------------------------------------------*/
#include "ymodem.h"
#include "multi_timer.h"
#include "uart.h"
#include "cqueue.h"
#include "elog.h"

/* Private typedef -----------------------------------------------------------*/



/* Private define ------------------------------------------------------------*/
#define YMODEM_TIMEOUT                  (10000)
#define YMODEM_SOFTWARE_TIME            (200)   //100ms

/* Private constants ---------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
ymodem_up_down_t ymodem_mode_cur = YMODEM_DEFAULT;;
static struct Timer ymodem_timer;
void ymodem_process(void);
char ymodem_rx_buf[PACKET_OVERHEAD + PACKET_1K_SIZE] = {0};
uint16_t ymodem_rx_buf_len = 0;
uint8_t ymodem_init_ok = false;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void ymodem_init(void)
{
    ymodem_mode_cur = YMODEM_DEFAULT;
    ymodem_rx_buf_len = 0;
    memset(ymodem_rx_buf,0,PACKET_OVERHEAD + PACKET_1K_SIZE);
    

    /* 100ms */
    timer_init(&ymodem_timer,ymodem_process,YMODEM_SOFTWARE_TIME/MULTI_TIME_AVE,YMODEM_SOFTWARE_TIME/MULTI_TIME_AVE);
    timer_start(&ymodem_timer);
    
    ymodem_init_ok = true;
}

void ymodem_up_down_set(ymodem_up_down_t mode)
{
    if(false == ymodem_init_ok)
    {
        ymodem_init();
    }

    queue_pop_all(&queue_uart1_rx);
    ymodem_tx_rx_reset();
    ymodem_mode_cur = mode;
    
    if(mode == YMODEM_DEFAULT)
    {
        uart_mode_set(UART_MODE_GENERAL);
    }
    else
    {
        uart_mode_set(UART_MODE_YMODEM); 
    }
}



static void ymodem_process(void)
{
    static uint16_t tm_out = 0;
    
    if(YMODEM_DEFAULT != ymodem_mode_cur)
    {
        ymodem_rx_buf_len = queue_get_count(&queue_uart1_rx);
    
        queue_peek_length(&queue_uart1_rx,(uint8_t *)ymodem_rx_buf,ymodem_rx_buf_len);
        queue_pop_length(&queue_uart1_rx,ymodem_rx_buf_len);
        
        if(tm_out > YMODEM_TIMEOUT)
        {
            ymodem_up_down_set(YMODEM_DEFAULT);
            tm_out = 0;
            log_e("ymodem over time.");
        }
    }
 
    switch( ymodem_mode_cur )
    {
    case YMODEM_DOWNLOAD:
    {
        uint8_t res_download;
        
        res_download = ymodem_rx_put( (char*)ymodem_rx_buf, ymodem_rx_buf_len );
    
        if(YMODEM_NULL == res_download)
        {
            tm_out += YMODEM_SOFTWARE_TIME;
        }
        else if(YMODEM_OK == res_download)
        {
            tm_out = 0;
        }
        else if(YMODEM_TX_RX_OVER == res_download)
        {
            ymodem_up_down_set(YMODEM_DEFAULT);
            tm_out = 0;
//            log_i("ymodem download done.");
        }
        else
        {
            ymodem_up_down_set(YMODEM_DEFAULT);
            tm_out = 0;
            log_e("ymodem download err.");
        }   
    }
        break;
    case YMODEM_UPLOAD:
    {
        uint8_t res_upload;
        
        res_upload = ymodem_tx_put( (char*)ymodem_rx_buf, ymodem_rx_buf_len );
        
        if(YMODEM_NULL == res_upload)
        {
            tm_out += YMODEM_SOFTWARE_TIME;
        }
        else if(YMODEM_OK == res_upload)
        {
            tm_out = 0;
        }
        else if(YMODEM_TX_RX_OVER == res_upload)
        {
            ymodem_up_down_set(YMODEM_DEFAULT);
            tm_out = 0;
//            log_i("ymodem upload done.");
        }
        else
        {
            ymodem_up_down_set(YMODEM_DEFAULT);
            tm_out = 0;
            log_e("ymodem upload err.");
        }   
        
    } 
        break;
    default:
        break;
    }
}


