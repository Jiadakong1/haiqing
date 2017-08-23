/******************************************************************************

  Copyright (C), 2017 Deephi Tech. Co., Ltd.

 ******************************************************************************
  File Name     : console.c
  Version       : Initial Draft
  Author        : Ye Yang - Deephi software engineer
  Created       : 2017/7/70
  Description   :
  History       :
  1.Date        : 2017/7/20
    Author      :
    Modification: Created file

Note:
******************************************************************************/

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif

/* include */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <malloc.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include "frame.hpp"
#include <semaphore.h>

#define  DEBUG
#ifdef  DEBUG
#define  dbg printf
#else
#define  dbg
#endif

#define FALSE  -1
#define TRUE   0

#define TIMEOUT       40
#define RECEIVE_BUF   256
#define MSG_LEN       10

#define CMD            "fb 71 00 01 1b 00 00 00 11 84 40 01 00 00 00 ff 01"
#define CMD_QEURY      "fb 71 00 01 0c 00 00 00 01 01 80 00 01 02 03 04 05 06 07 08"
#define CMD_HEART_BEAT "fb 71 00 01 04 00 00 00 02 01 80 00"
#define CMD_PARA_SET   "fb 71 00 01 04 00 00 00 03 01 80 00"
#define CMD_SYNC_TIME  "fb 71 00 01 04 00 00 00 04 01 80 00"
#define CMD_CHECK      "fb 71 00 01 04 00 00 00 7f 01 80 00"

/*define the device port*/

#define DEV_PORT "/dev/ttyPS1"

/*define the flag to cancel the thread*/

volatile sig_atomic_t g_exit_flag = 0;

int totalLen=0;
int totalLen_back=0;

uart_pack_t *pUart_Pack;
uart_pack_t *pUart_Pack_back;

sem_t sem;

pthread_cond_t cond;
pthread_mutex_t mutex;

int baud_rate = 115200;


int UART0_Open(char* port)
{
    int fd;
   // fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);
    fd = open(port, O_RDWR|O_NOCTTY|O_SYNC);
    if (FALSE == fd)
        {
        perror("Can't Open Serial Port");
        return(FALSE);
        }

     //恢复串口为阻塞状态
    if(fcntl(fd, F_SETFL, 0) < 0)
        {
        printf("fcntl failed!\n");
        return(FALSE);
        }
        else
        {
        //printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
        }

    //测试是否为终端设备
    if(0 == isatty(STDIN_FILENO))
        {
        printf("standard input is not a terminal device\n");
        return(FALSE);
        }
    return fd;
}

void UART0_Close(int fd)
{
    close(fd);
}


int UART0_Set_BaudRate(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int   i;
    int   status;
    baud_rate = speed;
    //int   speed_arr[] = {B4000000, B3500000, B2500000, B115200, B19200, B9600, B4800, B2400, B1200, B300};
    //int   name_arr[] = {4000000, 3500000, 2500000, 115200,  19200,  9600,  4800,  2400,  1200,  300};
    int   speed_arr[] = {B9600, B19200, B38400, B57600, B115200, B230400, B460800, B576000, B921600, B1000000};
    int   name_arr[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 576000, 921600, 1000000};
    struct termios options;

    if (tcgetattr(fd,&options) !=  0)
       {
          perror("SetupSerial 1");
          return(FALSE);
       }

    //设置输入输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
        {
        if  (speed == name_arr[i])
            {
                cfsetispeed(&options, speed_arr[i]);
                cfsetospeed(&options, speed_arr[i]);
            }
        }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;

    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    switch(flow_ctrl)
        {
        case 0 ://不用流控
              options.c_cflag &= ~CRTSCTS;
              break;

        case 1 ://硬件流控
              options.c_cflag |= CRTSCTS;
              break;

        case 2 ://软件流控
              options.c_cflag |= IXON | IXOFF | IXANY;
              break;

        default:
            fprintf(stderr, "Unkown flow!\n");
            return -1;
        }

    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;

    switch (databits)
        {
        case 5:
                options.c_cflag |= CS5;
                break;
        case 6:
                options.c_cflag |= CS6;
                break;
        case 7:
                options.c_cflag |= CS7;
                break;
        case 8:
                options.c_cflag |= CS8;
                break;
        default:
                fprintf(stderr,"Unsupported data size\n");
                return (FALSE);
        }

    switch (parity)
        {
        case 'n':
        case 'N': //无奇偶校验位。
                 options.c_cflag &= ~PARENB;
                 options.c_iflag &= ~INPCK;
                 break;
        case 'o':
        case 'O'://设置为奇校验
                 options.c_cflag |= (PARODD | PARENB);
                 options.c_iflag |= INPCK;
                 break;
        case 'e':
        case 'E'://设置为偶校验
                 options.c_cflag |= PARENB;
                 options.c_cflag &= ~PARODD;
                 options.c_iflag |= INPCK;
                 break;
        case 's':
        case 'S': //设置为空格
                 options.c_cflag &= ~PARENB;
                 options.c_cflag &= ~CSTOPB;
                 break;
        default:
                 fprintf(stderr,"Unsupported parity\n");
                 return (FALSE);
        }

    switch (stopbits)
        {
        case 1:
                options.c_cflag &= ~CSTOPB; break;
        case 2:
                options.c_cflag |= CSTOPB; break;
        default:
                fprintf(stderr,"Unsupported stop bits\n");
                return (FALSE);
        }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

//  二进制数据时遇到0x0d,0x11和0x13却会被丢掉。不用说也知道，这几个肯定是特殊字符，被用作特殊控制了。关掉ICRNL和IXON选项即可解决。
// c_iflag &= ~(ICRNL | IXON);
// 0x0d 回车符CR
// 0x11 ^Q VSTART字符
// 0x13 ^S VSTOP字符
// ICRNL 将输入的CR转换为NL
// IXON 使起动/停止输出控制流起作用

    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 0; /* 读取一个字符等待1*(1/10)s */

    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读

    tcflush(fd,TCIFLUSH);

    if (tcsetattr(fd,TCSANOW,&options) != 0)
        {
        perror("com set error!\n");
        return (FALSE);
        }

    return (TRUE);
}

int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int   i;
    int   status;
    int   speed_arr[] = {B4000000, B3500000, B2500000, B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {4000000, 3500000, 2500000, 115200,  19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    baud_rate = speed;

    if (tcgetattr(fd,&options) !=  0)
       {
          perror("SetupSerial 1");
          return(FALSE);
       }

    //设置输入输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
        {
        if  (speed == name_arr[i])
            {
                cfsetispeed(&options, speed_arr[i]);
                cfsetospeed(&options, speed_arr[i]);
            }
        }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;

    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    switch(flow_ctrl)
        {
        case 0 ://不用流控
              options.c_cflag &= ~CRTSCTS;
              break;

        case 1 ://硬件流控
              options.c_cflag |= CRTSCTS;
              break;

        case 2 ://软件流控
              options.c_cflag |= IXON | IXOFF | IXANY;
              break;

        default:
            fprintf(stderr, "Unkown flow!\n");
            return -1;
        }

    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;

    switch (databits)
        {
        case 5:
                options.c_cflag |= CS5;
                break;
        case 6:
                options.c_cflag |= CS6;
                break;
        case 7:
                options.c_cflag |= CS7;
                break;
        case 8:
                options.c_cflag |= CS8;
                break;
        default:
                fprintf(stderr,"Unsupported data size\n");
                return (FALSE);
        }

    switch (parity)
        {
        case 'n':
        case 'N': //无奇偶校验位。
                 options.c_cflag &= ~PARENB;
                 options.c_iflag &= ~INPCK;
                 break;
        case 'o':
        case 'O'://设置为奇校验
                 options.c_cflag |= (PARODD | PARENB);
                 options.c_iflag |= INPCK;
                 break;
        case 'e':
        case 'E'://设置为偶校验
                 options.c_cflag |= PARENB;
                 options.c_cflag &= ~PARODD;
                 options.c_iflag |= INPCK;
                 break;
        case 's':
        case 'S': //设置为空格
                 options.c_cflag &= ~PARENB;
                 options.c_cflag &= ~CSTOPB;
                 break;
        default:
                 fprintf(stderr,"Unsupported parity\n");
                 return (FALSE);
        }

    switch (stopbits)
        {
        case 1:
                options.c_cflag &= ~CSTOPB; break;
        case 2:
                options.c_cflag |= CSTOPB; break;
        default:
                fprintf(stderr,"Unsupported stop bits\n");
                return (FALSE);
        }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

//  二进制数据时遇到0x0d,0x11和0x13却会被丢掉。不用说也知道，这几个肯定是特殊字符，被用作特殊控制了。关掉ICRNL和IXON选项即可解决。
// c_iflag &= ~(ICRNL | IXON);
// 0x0d 回车符CR
// 0x11 ^Q VSTART字符
// 0x13 ^S VSTOP字符
// ICRNL 将输入的CR转换为NL
// IXON 使起动/停止输出控制流起作用

    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 0; /* 读取一个字符等待1*(1/10)s */

    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读

    tcflush(fd,TCIFLUSH);

    if (tcsetattr(fd,TCSANOW,&options) != 0)
        {
        perror("com set error!\n");
        return (FALSE);
        }

    return (TRUE);
}

int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int err;

    if (UART0_Set(fd, speed, 0, databits, stopbits, 'N') == FALSE)
       {
        return FALSE;
       }
    else
       {
        return  TRUE;
        }
}

int UART0_Recv(int fd, char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);

    FD_SET(fd,&fs_read);

    time.tv_sec = 10;

    time.tv_usec = 0;

    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);

    if(fs_sel)
        {
        len = read(fd,rcv_buf,data_len);

        printf("len = %d fs_sel = %d\n",len,fs_sel);

        return len;
       }
    else
       {
       printf("Sorry,I am wrong!");
       return FALSE;
       }
}

ssize_t safe_write(int fd, const void *vptr, size_t n)
{
    size_t  nleft;
    ssize_t nwritten;
    const char *ptr;

    ptr = vptr;
    nleft = n;

    while(nleft > 0)
    {
    if((nwritten = write(fd, ptr, nleft)) <= 0)
        {
            if(nwritten < 0 && errno == EINTR)
                nwritten = 0;
            else
                return -1;
        }

        nleft -= nwritten;

        ptr   += nwritten;
    }
    return(n);
}

ssize_t tread(int fd, void *buf, size_t nbytes, unsigned int timout)
{

    int    nfds;

    fd_set readfds;
    ssize_t cnt = 0;
    struct timeval tv;

    tv.tv_sec = timout;

    tv.tv_usec = 0;

    FD_ZERO(&readfds);

    FD_SET(fd, &readfds);

    nfds = select(fd+1, &readfds, NULL, NULL, &tv);

    // switch(nfds)
    //     {
    //         case -1:
    //             fprintf(stderr,"select error!\n");
    //             return -1;
    //         case 0:
    //             fprintf(stderr,"time over!\n");
    //             return -1;
    //         default:
    //             cnt = safe_read(fd,buf,nbytes);
    //             if(cnt == -1)
    //             {
    //                 fprintf(stderr,"read error!\n");
    //                 return -1;
    //             }
    //             return cnt;
    //     }

    if (nfds <= 0)
        {
        if (nfds == 0)
            errno = ETIME;
        return(-1);
        }

    return(read(fd, buf, nbytes));
}

int UART0_Send(int fd, const void *send_buf,int data_len)
{
    int len = 0;

    len = safe_write(fd, send_buf, data_len);

    if(len == -1)
        {
        fprintf(stderr,"write error!\n");
        return -1;
        }

    return len;
}

int hextoi(unsigned char * asic)
{
    if(*asic >= 48 && *asic <= 57)
        {
        *asic -= 48;
        }
    else if(*asic >= 65 && *asic <= 70)
        {
        *asic = *asic - 65 + 10;
        }
    else if(*asic >= 97 && *asic <= 102)
        {
        *asic = *asic - 97 + 10;
        }
    else
        {
        printf("Not a valid asic number\n");
        return -1;
        }
    return 0;
}

int hex2numeric(void * src, unsigned int len, unsigned char * serialCmd)
{
    if(src == NULL || serialCmd == NULL)
        return -1;

    unsigned char *sCmd = serialCmd;
    unsigned char *pStart = (unsigned char *)src;

    int i = 0;
    int j = 0;

    while(j < len/2)
        {
        if(!hextoi(&pStart[i]) && !hextoi(&pStart[i+1]))
            {
            // eg: ascii '8''0' to hex 0x80
            sCmd[j] = ((pStart[i] << 4) | pStart[i+1]);
            }
        else
            {
            return -1;
            }
        j += 1;
        i += 2;
        }
    return 0;
}

void send_hex_via_uart2(char *cmd)
{
    int i = 0;
     char *src= cmd;

    int length = strlen(cmd);

    unsigned char serialCmd[length/2];

    int fd = 0, len = 0, err = 0;

    memset(serialCmd, 0x00, sizeof(serialCmd));

    if (hex2numeric(src, length, serialCmd) < 0)
        {
        printf("error\r\n");
        return;
        }

    for(i=0; i<length/2; i++)
        printf("%02x",serialCmd[i]);

    printf("\r\n");

    fd = UART0_Open(DEV_PORT);

    do
        {
        err = UART0_Init(fd, 115200, 0, 8, 1, 'N');
        } while(FALSE == err || FALSE == fd);

    len = UART0_Send(fd, serialCmd, length);

    if(len <= 0)
        printf("send data failed!\n");

    UART0_Close(fd);
}

//打印字符串中指定长度的内容，用二进制显示出来。
static void print_rcv_buf (unsigned char *rcv_buf,int size)
{
    int i;
    unsigned char *buf = rcv_buf;

    for (i=0; i<size;i++) {
        dbg ("%02x ", buf[i]);
    }

    dbg ("\r\n");
}

void response_query(query_pack_t *pPack,  int * len_to_send)
{
    int len;

    /* Prepare PACKAGE HAED */

    pPack->header.magic1 = 0xfb;
    pPack->header.magic2 = 0x71;
    pPack->header.srcId  = 0x00;
    pPack->header.dstId  = 0x1;
    pPack->header.checkSum = 0;

    /* Prepare COMMAND */

    pPack->cmd.cmd   = 0x0101;
    pPack->cmd.flags = 0x80;
    pPack->cmd.num   = 0x0;

    /* Prepare query package */

    pPack->query.software_ver  = 0x1001;
    pPack->query.fpga_ver      = 0x2002;
    pPack->query.hardware_ver  = 0x3003;
    pPack->query.extra_len     = 0x0;

    len = sizeof(msg_hdr_t) + sizeof(cmd_t) + sizeof(query_t);
    //dbg("Total length: %d, effecive length: %d\n",len, len - sizeof(msg_hdr_t));

    pPack->header.length = len - sizeof(msg_hdr_t);

    *(int *)len_to_send = len;

    return;
}

void no_response_cmd(no_response_pack_t *pPack,  int * len_to_send, unsigned short cmd)
{
    int len;

    /* Prepare PACKAGE HAED */

    pPack->header.magic1   = 0xfb;
    pPack->header.magic2   = 0x71;
    pPack->header.srcId    = 0x00;
    pPack->header.dstId    = 0x01;
    pPack->header.checkSum = 0x0000;

    /* Prepare COMMAND */

    pPack->cmd.cmd   = cmd;
    pPack->cmd.flags = 0x80;
    pPack->cmd.num   = 0x0;

    len = sizeof(msg_hdr_t) + sizeof(cmd_t);
    //dbg("Total length: %d, effecive length: %d\n",len, len - sizeof(msg_hdr_t));

    pPack->header.length = len - sizeof(msg_hdr_t);

    *(int *)len_to_send = len;

    return;
}

/******************************************************************************
* function : handle signal
******************************************************************************/
void HandleSig(int signo)
{
    if (SIGINT == signo || SIGTSTP == signo)
    {
        g_exit_flag = 1; // set g_exit_flag

        dbg("\033[0;31mprogram exit abnormally!\033[0;39m\n");
    }

    exit(0);
}


ssize_t packet_reception(int fd, char *buf, unsigned int timout)
{
    size_t nleft;

    ssize_t nread;

    char *cur_p_char =  (char *)buf;
    unsigned short *p_len = (unsigned short *)(cur_p_char+4);

    if((nread = tread(fd, cur_p_char, 1, timout)) <= 0)
        return -1;
    if(buf[0] != 0xFB)
        return 0;
    cur_p_char++;

    if((nread = tread(fd, cur_p_char, 1, timout)) <= 0)
        return 0;
    if(buf[1] != 0x71)
        return 0;
    cur_p_char++;

    //receive header left , 6 bytes
    nleft = 6;
    while (nleft > 0)
    {
        if ((nread = tread(fd, cur_p_char, nleft, timout)) < 0)
            return 0; /* error, return -1 */
        cur_p_char += nread;
        nleft -= nread;

        if(nleft <= 0)
            break;
        //if time out, then quit
    }


    //receive message
    nleft = (size_t)(*p_len);
    while (nleft > 0)
    {
        if ((nread = tread(fd, cur_p_char, nleft, timout)) < 0)
            return 0; /* error, return -1 */
        cur_p_char += nread;
        nleft -= nread;

        if(nleft <= 0)
            break;
    }

    return 8 + (*p_len);      /* return >= 0 */
}


void *thread_handshake(void *data)
{
    int fd = *(int *)data;
    int i;
    int len;
    int receive_len, send_len;
    unsigned char rcv_buf[RECEIVE_BUF];
    query_pack_t *pQueryPack;
    no_response_pack_t* pNoResPack;
    int timeout = 10;
    char ack = 0;
    int seconds;

    seconds= time((time_t*)NULL);

    while (1)
    {
        if(g_exit_flag)
        {
            g_exit_flag = 0;
            UART0_Close(fd);
            break;
        }

        len = 0;
        send_len = 0;

        receive_len = packet_reception(fd, rcv_buf, timeout);

        if(receive_len > 0)
        {
            rcv_buf[receive_len] = '\0';

            dbg(">>>>>>>> ");
            print_rcv_buf(rcv_buf, receive_len);

            /* QUERY */
            if (rcv_buf[8] == 0x01 && rcv_buf[9] == 0x01)
            {
                dbg("<<<<<<<< %s\n", CMD_QEURY );
                dbg("Query command!\n" );

                pQueryPack = malloc(sizeof(query_pack_t));
                memset(pQueryPack, 0, sizeof(query_pack_t));

                response_query(pQueryPack, &len);

                send_len = UART0_Send(fd, pQueryPack, len);

                if(send_len > 0)
                {
                    ack |= 0x01;
                    dbg("send %d Bytes, ack: %d\n",send_len, ack);
                }
                else
                    dbg("send data failed!\n");

                free(pQueryPack);
            }

            /* CHECK */

            if (rcv_buf[8] == 0x7f && rcv_buf[9] == 0x01)
            {
                dbg("<<<<<<<< %s\n", CMD_CHECK );
                dbg("Check command!\n" );

                pNoResPack = malloc(sizeof(no_response_pack_t));
                memset(pNoResPack, 0, sizeof(no_response_pack_t));

                no_response_cmd(pNoResPack, &len, 0x017f);

                send_len = UART0_Send(fd, pNoResPack, len);

                if(send_len > 0)
                {
                    ack |= 0x02;
                    dbg("send %d Bytes, ack: %d\n",send_len, ack);
                }
                else
                    dbg("send data failed!\n");

                free(pNoResPack);

            }

            /* PARAMETER SET */

            if (rcv_buf[8] == 0x03 && rcv_buf[9] == 0x01)
            {

                dbg("<<<<<<<< %s\n", CMD_PARA_SET);
                dbg("Parameter set command!\n" );

                pNoResPack = malloc(sizeof(no_response_pack_t));
                memset(pNoResPack, 0, sizeof(no_response_pack_t));

                no_response_cmd(pNoResPack, &len, 0x0103);

                send_len = UART0_Send(fd, pNoResPack, len);

                if(send_len > 0)
                {
                    ack |= 0x04;
                    dbg("send %d Bytes, ack: %d\n",send_len, ack);
                }
                else
                    dbg("send data failed!\n");

                free(pNoResPack);
            }

            /* SYNC */

            if (rcv_buf[8] == 0x04 && rcv_buf[9] == 0x01)
            {
                dbg("<<<<<<<< %s\n", CMD_SYNC_TIME);
                dbg("Sync time command!\n" );

                pNoResPack = malloc(sizeof(no_response_pack_t));
                memset(pNoResPack, 0, sizeof(no_response_pack_t));

                no_response_cmd(pNoResPack, &len, 0x0104);

                send_len = UART0_Send(fd, pNoResPack, len);

                if(send_len > 0)
                {
                    ack |= 0x08;
                    dbg("send %d Bytes, ack: %d\n",send_len, ack);
                }
                else
                    dbg("send data failed!\n");

                free(pNoResPack);
            }

            /* HEART BEAT */

            if (rcv_buf[8] == 0x02 && rcv_buf[9] == 0x01)
            {
                dbg("<<<<<<<< %s\n", CMD_HEART_BEAT);
                dbg("Heart beat command!\n" );

                pNoResPack = malloc(sizeof(no_response_pack_t));
                memset(pNoResPack, 0, sizeof(no_response_pack_t));

                no_response_cmd(pNoResPack, &len, 0x0102);

                send_len = UART0_Send(fd, pNoResPack, len);

                if(send_len > 0)
                {
                    dbg("send %d data successful\n", send_len);
                }
                else
                    dbg("send data failed!\n");

                free(pNoResPack);
            }

            /* BAUDRATE SET */

            if (rcv_buf[8] == 0x01 && rcv_buf[9] == 0x03)
            {
                dbg("<<<<<<<< baud_rate = %d\n", *((unsigned int *)(&rcv_buf[12])));
                dbg("Baudrate set command!\n" );
                usleep(100000);
                //UART0_Set_BaudRate(fd, *((unsigned int *)(&rcv_buf[12])), 0, 8, 1, 'N');

            }
            //dbg("##Find package! package_len:%d, COMMAND: 0x%02x%02x\n",rcv_buf[4],rcv_buf[9],rcv_buf[8]);
            dbg("\n");
        }
        else if(receive_len == -1)    //receive == 0 : have revieved data, but it is wrong
        {
            printf("cannot receive data, %ds timeout\n",timeout);
        }

        if (ack == 0xf)
        {
            printf("Elapsed: %lds, Handshake done!\n", time((time_t*)NULL) - seconds);
            break;
        }
    } //exit while(1);
}

void *prepare_roi(void *data)
{
    long long L1,L2,L3;
    struct timeval tv;
    int fd = *(int *)data;
    int send_len = 0;

    while(1)
    {
    if(g_exit_flag)
        {
        g_exit_flag = 0;
        free(pUart_Pack);
        UART0_Close(fd);
        return;
        }

    gettimeofday(&tv,NULL);
    L1 = tv.tv_sec*1000*1000 + tv.tv_usec;

    prepare_data(pUart_Pack, &totalLen);

    gettimeofday(&tv,NULL);
    L2 = tv.tv_sec*1000*1000+tv.tv_usec;

    printf("Total: %lldms\r\n\n",(L2-L1)/1000);

  //  sem_post(&sem); //为信号量加1

    pthread_mutex_lock(&mutex);

    totalLen_back = totalLen;

    memcpy(pUart_Pack_back, pUart_Pack, sizeof(uart_pack_t));

    pthread_cond_signal(&cond);

    pthread_mutex_unlock(&mutex);
    }

}

int main(int argc, char **argv)
{
    int fd,err;
    int send_len;
    pthread_t handshake;
    pthread_t prepare;
    long long L2,L3;
    struct timeval tv;
    struct timespec ts;
    int timeInMs = TIMEOUT;

    pthread_mutex_init(&mutex, NULL);

    pthread_cond_init(&cond, NULL);

    pUart_Pack = malloc(sizeof(uart_pack_t));
    pUart_Pack_back = malloc(sizeof(uart_pack_t));

    memset(pUart_Pack, 0, sizeof(uart_pack_t));
    memset(pUart_Pack_back, 0, sizeof(uart_pack_t));

    sem_init(&sem, 0, 0); //信号量初始化

    // Register signals

    signal(SIGINT, HandleSig);

    signal(SIGTERM, HandleSig);

    fd = UART0_Open(DEV_PORT);

    do
        {
        err = UART0_Init(fd, 115200, 0, 8, 1, 'N');
        } while(FALSE == err || FALSE == fd);

    if (*argv[1] == '1')
        {
        if(pthread_create(&handshake,NULL,(void *)thread_handshake,(void *)&fd) == -1){
            dbg("Create the thread of read error!\n");
        return -1;
        }

        pthread_join(handshake, NULL);
        }


    /* Start DPU */

    deephi_dpu_init();

    if (pthread_create(&prepare,NULL,(void *)prepare_roi, (void *)&fd) == -1){
        dbg("Create the thread of read error!\n");
        return -1;
        }

    while(1)
        {
        if(g_exit_flag)
            {
            g_exit_flag = 0;
            free(pUart_Pack);
            free(pUart_Pack_back);
            UART0_Close(fd);
            return;
            }

      //  sem_wait(&sem);

        pthread_mutex_lock(&mutex);

        gettimeofday(&tv, NULL);

        ts.tv_sec = time(NULL) + timeInMs / 1000;
        ts.tv_nsec = tv.tv_usec * 1000 + 1000 * 1000 * (timeInMs % 1000);
        ts.tv_sec += ts.tv_nsec / (1000 * 1000 * 1000);
        ts.tv_nsec %= (1000 * 1000 * 1000);

        pthread_cond_timedwait(&cond, &mutex, &ts);

        pthread_mutex_unlock(&mutex);

  //      gettimeofday(&tv,NULL);
  //      L2 = tv.tv_sec*1000*1000 + tv.tv_usec;

        send_len = UART0_Send(fd, pUart_Pack_back, totalLen_back);

  //      gettimeofday(&tv,NULL);
  //      L3 = tv.tv_sec*1000*1000+tv.tv_usec;

       //printf("framd id## %d \r\n", pUart_Pack_back->rcv_buf.rcv_buf_num);
       //printf("Send %dB in %lldus\n\r\n", totalLen_back, L3-L2);

        if(send_len <= 0)
          printf("send data failed!\n");
        }

    pthread_join(prepare, NULL);

    /* Stop DPU */

    deephi_dpu_fini();

    sem_destroy(&sem); //销毁信号量

    free(pUart_Pack);
    free(pUart_Pack_back);

    UART0_Close(fd);
}

int main_serialization(int argc, char **argv)
{
    int fd,err;
    int send_len,ready_to_send=0;
    pthread_t handshake;
    uart_pack_t *pRoi_Pack;
    long long L1,L2,L3;
    struct timeval tv;

    // Register signals

    signal(SIGINT, HandleSig);

    signal(SIGTERM, HandleSig);

    fd = UART0_Open(DEV_PORT);

    do
        {
        err = UART0_Init(fd, 115200, 0, 8, 1, 'N');
        } while(FALSE == err || FALSE == fd);

    if (*argv[1] == '1')
        {
        if(pthread_create(&handshake,NULL,(void *)thread_handshake,(void *)&fd) == -1){
            dbg("Create the thread of read error!\n");
            return -1;
        }

    pthread_join(handshake, NULL);
    }

    /* Start DPU */

    deephi_dpu_init();

    pRoi_Pack = malloc(sizeof(uart_pack_t));

    while(1)
    {
    if(g_exit_flag)
        {
        g_exit_flag = 0;
        free(pRoi_Pack);
        UART0_Close(fd);
        return;
        }

    memset(pRoi_Pack, 0, sizeof(uart_pack_t));

    gettimeofday(&tv,NULL);
    L1 = tv.tv_sec*1000*1000 + tv.tv_usec;

    prepare_data(pRoi_Pack, &ready_to_send);

    gettimeofday(&tv,NULL);
    L2 = tv.tv_sec*1000*1000+tv.tv_usec;

    send_len = UART0_Send(fd, pRoi_Pack, ready_to_send);

    gettimeofday(&tv,NULL);
    L3 = tv.tv_sec*1000*1000+tv.tv_usec;

    printf("Total: %lldms, send %dB in %lldus\n\r\n",(L2-L1)/1000, ready_to_send, L3-L2);

    if(send_len <= 0)
      printf("send data failed!\n");
    }

    free(pRoi_Pack);

    /* Stop DPU */

    deephi_dpu_fini();

    UART0_Close(fd);
}


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
