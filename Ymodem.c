/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/

#include "ymodem.h"

/*********************************************************************
 * CONSTANTS
 */
#define YMODEM_DATA_SIZE_128    128
#define YMODEM_DATA_SIZE_1024   1024

#define YMODEM_RX_IDLE          0
#define YMODEM_RX_ACK           1
#define YMODEM_RX_EOT           2
#define YMODEM_RX_OVER          3
#define YMODEM_RX_ERR           4
#define YMODEM_RX_EXIT          5

#define YMODEM_TX_IDLE          0
#define YMODEM_TX_IDLE_ACK      1
#define YMODEM_TX_DATA          2
#define YMODEM_TX_DATA_ACK      3
#define YMODEM_TX_EOT           4
#define YMODEM_TX_OVER          5
#define YMODEM_TX_ERR           6
#define YMODEM_TX_EXIT          7
/*********************************************************************
 * GLOBAL VARIABLES
 */
static  uint8_t ym_rx_status = YMODEM_RX_IDLE;
static  uint8_t ym_tx_status = YMODEM_TX_IDLE;
static  size_t pac_size;
static  size_t seek;
static  size_t ym_tx_fil_sz;
static  uint8_t ym_cyc;   //发送时的轮转变量
char    ym_pbuf[PACKET_OVERHEAD + PACKET_1K_SIZE];

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * TYPE_DEFS
 */

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/
uint8_t ymodem_rx_header( char* fil_nm, size_t fil_sz );
uint8_t ymodem_rx_finish( uint8_t status );
uint8_t ymodem_rx_pac_get( char *buf, size_t seek, size_t size );
uint8_t ymodem_tx_header( char **fil_nm, size_t *fil_sz );
uint8_t ymodem_tx_finish( uint8_t status );
uint8_t ymodem_tx_pac_get( char *buf, size_t seek, size_t size );
void __putchar( char ch );
void __putbuf( char* buf, size_t len );


void ymodem_tx_rx_reset(void)
{
    ym_rx_status = YMODEM_RX_IDLE;
    ym_tx_status = YMODEM_TX_IDLE;
    seek = 0;
}

//核心函数
uint16_t ymodem_crc16(const uint8_t *buf, uint16_t count)
{
    uint16_t crc = 0;
    uint32_t i;

    while(count--) {
        crc = crc ^ *buf++ << 8;

        for (i=0; i<8; i++) {
            if (crc & 0x8000) {
                crc = crc << 1 ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

static const char *ymodem_u32_to_str(uint32_t val)
{
    /* Maximum number of decimal digits in u32 is 10 */
    static char num_str[11];
    uint32_t  pos = 10;
    num_str[10] = 0;

    if (val == 0) {
        /* If already zero then just return zero */
        return "0";
    }

    while ((val != 0) && (pos > 0)) {
        num_str[--pos] = (val % 10) + '0';
        val /= 10;
    }

    return &num_str[pos];
}

static unsigned long ymodem_str_to_u32(char* str)
{
    const char *s = str;
    unsigned long acc;
    uint32_t c;

    /* strip leading spaces if any */
    do {
        c = *s++;
    } while (c == ' ');

    for (acc = 0; (c >= '0') && (c <= '9'); c = *s++) {
        c -= '0';
        acc *= 10;
        acc += c;
    }
    return acc;
}

//返回包的类型
uint8_t ymodem_rx_pac_check( char* buf, size_t sz )
{
    char ch;
    ch = buf[0];
    if( sz < 128 ) //是个指令包
    {
        if( ch==EOT || ch==ACK || ch==NAK || ch==CAN || ch==CNC )
        {
            uint32_t i=1;
            while( i<sz && buf[i++]==ch );    //判断包中所有内容是否一样
            if( sz == i )     //是全部一样的话，则认为此命令包有效
                return ch;
            else
                return 0xff;
        }
        else
            return 0xff;      //错误的指令码
    }
    else
    {
        if( ch==SOH || ch==STX )
        {
            uint16_t crc1 = ymodem_crc16( (uint8_t*)(buf+PACKET_HEADER), sz-PACKET_OVERHEAD );
            uint16_t crc2 = ((uint16_t)(buf[sz-2]))*256+buf[sz-1];
            if( crc1 == crc2 && 0xff == (uint8_t)buf[1]+(uint8_t)buf[2] )
            {
                return ch;
            }
            else
            {
                return 0xff;      //数据包校验为错
            }
        }
        else
        {
            return 0xff;      //错误的指令码
        }
    }
}

/***********************接收部分*********************/

//包是否为空
uint8_t ymodem_rx_pac_if_empty( char *buf, size_t sz )
{
    size_t offset=0;

    while( buf[offset]==0x00 && ++offset<sz );
    if( offset==sz )
    {
        return true;
    }
    else
    {
        return false;
    }
}

//解析出头包中的文件名和大小
uint8_t ymodem_rx_prepare( char *buf, size_t sz ) 
{
    uint8_t ans = YMODEM_OK;
    char *fil_nm;
    uint8_t   fil_nm_len;
    size_t fil_sz;
    
    fil_nm = buf;
    fil_nm_len = strlen( fil_nm );
    fil_sz = (size_t)ymodem_str_to_u32( buf+fil_nm_len+1 );
    ans = ymodem_rx_header( fil_nm, fil_sz );
    
    return ans;
}

/*********************************************************************
 * @fn      ymodem_tx_put : Ymodem接收时，逻辑轮转调用函数
 * @param   buf : 数据缓冲区 buf : 数据大小
 */
uint8_t ymodem_rx_put( char *buf, size_t rx_sz )
{
    if( 0 == rx_sz )      //超时，从而得到的长度为0，则尝试发送“C”，并返回
    {
        if(YMODEM_RX_IDLE == ym_rx_status)
        {
            __putchar( 'C' );
        }
        return YMODEM_NULL;
    }

    switch( ym_rx_status )
    {
    case YMODEM_RX_IDLE:
        
        switch( ymodem_rx_pac_check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
        {
        case SOH:
        case STX:
            pac_size = (uint8_t)(buf[0])==SOH? PACKET_SIZE:PACKET_1K_SIZE;
            if( true == ymodem_rx_pac_if_empty( buf+PACKET_HEADER, pac_size ) )   //判断是否是空包
            {
                __putchar( ACK );
                ym_rx_status = YMODEM_RX_EXIT;
                goto exit_rx;                  //这是在本循环必须完成的操作，所以需要用到 goto 语句
            }
            else    //如果不是空包，则认为是第一个包（包含文件名和文件大小）
            {
                if( pac_size==128 && YMODEM_OK == ymodem_rx_prepare( buf+PACKET_HEADER, pac_size ) )
                {
                    __putchar( ACK );
                    seek = 0;      //初始化变量，用于接收新文件
                    __putchar( 'C' );
                    ym_rx_status = YMODEM_RX_ACK;
                }
                else
                {
                    ym_rx_status = YMODEM_RX_ERR;
                    goto err_rx; //在IDLE中接收到一个1024的数据包，则肯定是状态有问题
                }
            }
            break;
        case EOT:
            ym_rx_status = YMODEM_RX_EXIT;
            goto exit_rx;                      //这是在本循环必须完成的操作，所以需要用到 goto 语句
            
        default:
            //     __putchar( NAK );      //不正常的状态，调试用
            ym_rx_status = YMODEM_RX_ERR;
            goto err_rx;              //这儿暂时认为，包有误，就退出
            
        }
        break;
        
    case YMODEM_RX_ACK:                                         //1级——文件接收状态中
        switch( ymodem_rx_pac_check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
        {
        case SOH:
        case STX:
            __putchar( ACK );
            pac_size = (uint8_t)(buf[0])==SOH? PACKET_SIZE:PACKET_1K_SIZE;
            ymodem_rx_pac_get( buf+PACKET_HEADER, seek, pac_size );  //将接收的包保存
            seek += pac_size;

            break;
            //指令包
        case EOT:
            __putchar( NAK );
            ym_rx_status = YMODEM_RX_EOT;
            break;
        case CAN:
            ym_rx_status = YMODEM_RX_ERR;
            goto err_rx;
        default:
            ym_rx_status = YMODEM_RX_ERR;
            goto err_rx;           //这儿暂时认为，包有误，就重发
        }
        break;
        
    case YMODEM_RX_EOT:         //在这里保存文件
    {
        switch( ymodem_rx_pac_check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
        {
            //指令包
        case EOT:
            __putchar( ACK );
            __putchar( 'C' );
            ymodem_rx_finish( YMODEM_OK );   
            ym_rx_status = YMODEM_RX_OVER;
            break;
        default:
            ym_rx_status = YMODEM_RX_ERR;
            goto err_rx; 
        }
    }
    break;
    
    case YMODEM_RX_OVER:         //在这里保存文件
    {
        if(rx_sz > 133)
        {
            rx_sz = 133;
        }
        
        switch( ymodem_rx_pac_check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
        {
        case SOH:
            __putchar( ACK );
                 
            ym_rx_status = YMODEM_RX_IDLE;
            return YMODEM_TX_RX_OVER;
        default:
            ym_rx_status = YMODEM_RX_EXIT;
            goto exit_rx;   
        }
    }
    
err_rx:
    case YMODEM_RX_ERR:         //在这里放弃保存文件,终止传输
        __putchar( CAN );
        ymodem_rx_finish( YMODEM_ERR );
    
exit_rx:
    case YMODEM_RX_EXIT:        //到这里，就收拾好，然后退出
        ym_rx_status = YMODEM_RX_IDLE;

        return YMODEM_TX_RX_OVER;
    default:
        break;
    }
    
    return YMODEM_OK;
}

/**************************************发送部分********************************/

//pbuf 是指向缓冲区的最开始的地方， pac_sz 是数据区的大小
uint8_t ymodem_tx_make_pac_data( char *pbuf, size_t pac_sz )
{
    uint8_t ans = YMODEM_ERR;
    uint16_t crc;

    pbuf[0] = pac_sz==128 ? SOH:STX;
    pbuf[1] = ym_cyc;
    pbuf[2] = ~ym_cyc;
    crc = ymodem_crc16( (unsigned char const*)(pbuf + PACKET_HEADER), pac_sz );
    pbuf[PACKET_HEADER+pac_sz]   = (uint8_t)(crc/256);
    pbuf[PACKET_HEADER+pac_sz+1] = (uint8_t)(crc&0x00ff);
    ym_cyc++;
    
    ans = YMODEM_OK;
    
    return ans;
}

//制作包头
uint8_t ymodem_tx_make_pac_header( char *pbuf, char *fil_nm, size_t fil_sz )
{
    uint8_t ans = YMODEM_ERR;
    uint8_t nm_len;
    memset( pbuf+PACKET_HEADER, 0, 128);
    if( fil_nm )
    {
        nm_len = strlen( fil_nm );
        strcpy( pbuf+PACKET_HEADER, fil_nm );
        strcpy( pbuf+PACKET_HEADER+nm_len+1, ymodem_u32_to_str( fil_sz ) );
    }
    ym_cyc = 0x00;
    ymodem_tx_make_pac_data( pbuf, 128 );
    
    return ans;
}

/*********************************************************************
 * @fn      ymodem_tx_put : Ymodem发送时，逻辑轮转调用函数
 * @param   buf : 数据缓冲区 buf : 数据大小
 * 说明：
 * 1.发送 [包  头] 状态：如果没有文件名，则发送空包，否则发送封装的头包
 * 2.发送 [数据包] 状态：发送数据包，出现问题或结束，则进入结束状态
 * 3.发送 [结  束] 状态：处理发送完成的相关事情
 */
uint8_t ymodem_tx_put( char *buf, size_t rx_sz )
{
    char *fil_nm = NULL;
    size_t fil_sz = NULL;
    
    if( 0 == rx_sz )      
    {
        return YMODEM_NULL;
    }
    
    switch( ym_tx_status )
    {
    case YMODEM_TX_IDLE:
        switch( ymodem_rx_pac_check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
        {
        case CNC:
        {

            if( YMODEM_OK == ymodem_tx_header( &fil_nm, &fil_sz ) )   //得到 文件名和大小
            {   //封装一个包头，然后发送出去
                ym_tx_fil_sz = fil_sz;
                ymodem_tx_make_pac_header( ym_pbuf, fil_nm, fil_sz );
                __putbuf( ym_pbuf, PACKET_OVERHEAD+PACKET_SIZE );
                ym_tx_status = YMODEM_TX_IDLE_ACK;

            }
            else //封装一个空包，然后发出去
            {
                ymodem_tx_make_pac_header( ym_pbuf, NULL, NULL );
                __putbuf( ym_pbuf, PACKET_OVERHEAD+PACKET_SIZE );
            }
        }
        break;
        case CAN:
            ym_tx_status = YMODEM_TX_ERR;
            goto err_tx;
        default:
                         //这儿暂时认为，包有误，就退出
            break;
        }
        break;
    case YMODEM_TX_IDLE_ACK:
    {
        switch( ymodem_rx_pac_check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
        {
        case ACK://准备发数据包
            ym_tx_status = YMODEM_TX_DATA;
           
            //ACK C连发
            if(CNC == ymodem_rx_pac_check( buf+1, rx_sz-1 ))
            {
                buf[0] = buf[1];
                rx_sz-=1;
                goto dt_tx;
            }
        
            break;
        case NAK://准备重发包头
            ym_tx_status = YMODEM_TX_IDLE;
            break;
         case CAN:
            ym_tx_status = YMODEM_TX_ERR;
            goto err_tx;
            
        default://啥也不做
            ym_tx_status = YMODEM_TX_ERR;
            goto err_tx;
          
        }
    }
    break;
dt_tx:
    case YMODEM_TX_DATA:                             //1级——文件发送状态中
        switch( ymodem_rx_pac_check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
        {
        case CNC:
            if( YMODEM_OK == ymodem_tx_pac_get( ym_pbuf+PACKET_HEADER, seek, PACKET_1K_SIZE ) )  //读取下一组数据
            {
                if( YMODEM_OK == ymodem_tx_make_pac_data( ym_pbuf, PACKET_1K_SIZE ) )
                {
                    __putbuf( ym_pbuf, PACKET_OVERHEAD+PACKET_1K_SIZE );
                    ym_tx_status = YMODEM_TX_DATA_ACK;
                    
                }
                else        //读取数据出错，结束传输
                {
                    ym_tx_status = YMODEM_TX_ERR;
                    goto err_tx;
                }
            }
            break;
        case CAN:
            ym_tx_status = YMODEM_TX_ERR;
            goto err_tx;
        default:        //暂时啥也不做
            ym_tx_status = YMODEM_TX_ERR;
            goto err_tx;
           
        }
        break;
    case YMODEM_TX_DATA_ACK:
    {
        switch( ymodem_rx_pac_check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
        {
        case ACK:
            seek += PACKET_1K_SIZE;
            if( seek < ym_tx_fil_sz )     //数据未发送完（不能加‘=’！）
            {
                ym_tx_status = YMODEM_TX_DATA_ACK;
                
                if( YMODEM_OK == ymodem_tx_pac_get( ym_pbuf+PACKET_HEADER, seek, PACKET_1K_SIZE ) )  //读取下一组数据
                {
                    if( YMODEM_OK == ymodem_tx_make_pac_data( ym_pbuf, PACKET_1K_SIZE ) )
                    {
                        __putbuf( ym_pbuf, PACKET_OVERHEAD+PACKET_1K_SIZE );
                        ym_tx_status = YMODEM_TX_DATA_ACK;
                        
                    }
                    else        //读取数据出错，结束传输
                    {
                        ym_tx_status = YMODEM_TX_ERR;
                        goto err_tx;
                    }
                }
            }
            else  //数据发送完
            {
                ym_tx_status = YMODEM_TX_EOT;
                __putchar( EOT );
            }
            break;
        case CNC:       //如果接收方不先应答[ACK]而是直接发'C'在这里处理
            seek += PACKET_1K_SIZE;
            if( seek < ym_tx_fil_sz )     //数据未发送完（不能加‘=’！）
            {
                ym_tx_status = YMODEM_TX_DATA_ACK;
                //下面的状态，因为我需要马上回复，所以用goto
                goto dt_tx;         //发送下一个数据包
            }
            else  //数据发送完
            {
                ym_tx_status = YMODEM_TX_EOT;
                __putchar( EOT );
            }
            break;
        case CAN:
            ym_tx_status = YMODEM_TX_ERR;
            goto err_tx;
        default:
            ym_tx_status = YMODEM_TX_ERR;
            goto err_tx;
            
        }
    }
    break;
    case YMODEM_TX_EOT:
    {
        switch( ymodem_rx_pac_check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
        {
            //指令包
        case NAK:
            __putchar( EOT );
            break;
        case ACK:
            
            //ACK C连发
            if(CNC == ymodem_rx_pac_check( buf+1, rx_sz-1 ))
            {
                buf[0] = buf[1];
                rx_sz-=1;
                goto over_tx;
            }

            ym_tx_status = YMODEM_TX_OVER;
            break;
        case CAN:
            ym_tx_status = YMODEM_TX_ERR;
            goto err_tx;
        default:
            ym_tx_status = YMODEM_TX_ERR;
            goto err_tx;
           
        }
    }
    break;
over_tx:    
    case YMODEM_TX_OVER:
    {
        switch( ymodem_rx_pac_check( buf, rx_sz ) )   //检查当前包是否合法,并返回包的类型
        {
            //指令包
        case CNC:
            ymodem_tx_make_pac_header( ym_pbuf, NULL, NULL );
            __putbuf( ym_pbuf, PACKET_OVERHEAD+PACKET_SIZE );

            ymodem_tx_finish( YMODEM_OK );

            
            seek = 0;
            ym_tx_status = YMODEM_TX_IDLE;
            
            return YMODEM_TX_RX_OVER;
        case CAN:
            ym_tx_status = YMODEM_TX_EXIT;
            goto err_tx;           
        default:
            ym_tx_status = YMODEM_TX_EXIT;
            goto err_tx;
           
        }
    }
    
    
err_tx:
    case YMODEM_TX_ERR:         //在这里放弃保存文件,终止传输
        __putchar( CAN );
        
        ymodem_rx_finish( YMODEM_ERR );
    case YMODEM_TX_EXIT:        //到这里，就收拾好，然后退出
        ym_tx_status = YMODEM_TX_IDLE;
        seek = 0;
        

        return YMODEM_TX_RX_OVER;
    default:
        break;
    }
    
    return YMODEM_OK;
}
