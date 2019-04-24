#include "ymodem.h"
#include "uart.h"
#include "stdio.h"
#include "easyflash.h"
//实现的接口函数
//用户接口

static  size_t fil_rx_sz = 0;
static  char   fil_tx_nm[10] = {"log.txt"};      //传输的文件名称
static  size_t fil_tx_sz = 0;       //传输的文件大小


//接收
uint8_t ymodem_rx_header( char* fil_nm, size_t fil_sz )
{
    uint8_t ans = YMODEM_ERR;
    
    fil_rx_sz = fil_sz;
    
    ans = YMODEM_OK;
    return ans;
}

uint8_t ymodem_rx_finish( uint8_t status )
{
    return YMODEM_OK;
}

uint8_t ymodem_rx_pac_get( char *buf, size_t seek, size_t size )
{
    uint8_t ans = YMODEM_ERR;
    
    ans = YMODEM_OK;
    return ans;
}

//发送
/*********************************************************************
 * @fn      ymodem_tx_header : 系统调用，用来获取文件名和大小
 * @param   fil_nm : 文件名 fil_sz : 文件大小
 */
uint8_t ymodem_tx_header( char** fil_nm, size_t *fil_sz )
{
    uint8_t ans = YMODEM_ERR;
    
    *fil_nm = fil_tx_nm;
    *fil_sz = ef_log_get_used_size();
    fil_tx_sz = ef_log_get_used_size();    
    ans = YMODEM_OK;
    
    return ans;
}
/*********************************************************************
 * @fn      ymodem_tx_finish : 当传输结束时，会被调用
 * @param   status : 关闭的原因 YMODEM_OK 或 YMODEM_ERR
 */
uint8_t ymodem_tx_finish( uint8_t status )                         //返回结束原因，成功还是出错
{
    
    return YMODEM_OK;
}
/*********************************************************************
 * @fn      ymodem_tx_pac_get : 调用此来读取文件中的相应数据
 * @param   buf : 待写入的缓冲区地址 offset : 数据的偏移 size : 数据的大小
 */
uint8_t ymodem_tx_pac_get( char *buf, size_t offset, size_t size )
{
    uint8_t ans = YMODEM_ERR;
    
    if((offset + size) > fil_tx_sz)
    {
        memset(buf,0,size);
        size = fil_tx_sz - offset;
    }
    
    if(EF_NO_ERR == ef_log_read(offset , (uint32_t *)buf, size))
    {
       ans = YMODEM_OK; 
    }

    return ans;
}

//底层接口
void __putchar( char ch )
{
    ymodem_send((const uint8_t*)&ch,1);
}
void __putbuf( char *buf, size_t len )
{
    ymodem_send((const uint8_t*)buf,len);
}
