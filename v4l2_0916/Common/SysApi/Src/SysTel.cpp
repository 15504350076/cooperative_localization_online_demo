
/*================================================================================
                                                                               
Module Name:  SysTel.cpp

General Description: telnet server module

==================================================================================*/



#include <sys/prctl.h>
#include <SysApi/SysApi.h>
#include <SysApi/SysVos.h>
#include <SysApi/SysTel.h>
#include <sys/epoll.h>

extern cmd_table_t g_t_cmd_table[CMD_TABLE_SIZE];
extern int g_i_cmd_idx;

static tel_server_t g_t_telsrv = {0};

static u16 tel_cli_rcv();         /* telnet recv for select */
static u16 tel_cli_snd();         /* telnet send for select */
static u16 tel_cli_close();        /* telnet close client */
static u16 tel_accept();             /* telnet accept client */

static void* tel_proc(void* param);  /* telnet procedure */
static u16 tel_select();             /* telnet select */
static s32 tel_open(u32 dw_ip, u16 w_port);
static u16 tel_send(const char* pch_buf, u32 dw_len);

static void send_ac(s8 cmd, s8 opt);
static s8 remove_iac(u8 c);
static void run_cmd(char *szCmd);
static void parse_cmd(s8* pch_cmd, u32 dw_cmd_len);
static void prompt_show(void);

#define TEL_SEM_TAKE(t_sem) {if (sys_api_task_self_id() != g_t_telsrv.dw_thread_id) sys_api_sem_take(t_sem);}
#define TEL_SEM_GIVE(t_sem) {if (sys_api_task_self_id() != g_t_telsrv.dw_thread_id) sys_api_sem_give(t_sem);}

/* @Description:
 * init the telnet server module
 * @Parameters(in):
 * b_tel:init telnet server or not
 * w_port: telnet port
 * @Return value: 
 *  Success return true, fail return errno
 */
u16 tel_init(bool b_tel, u16 w_port)
{
    static bool b_init = false;
    u16 ret = 0;
    if (!b_init)
    {
        b_init = true;
        memset(&g_t_telsrv, 0, sizeof(g_t_telsrv));

        g_t_telsrv.aby_cmd[0] = '\0';
        g_t_telsrv.n_client_sock = INVALID_SOCKET;
        g_t_telsrv.n_listen_sock = INVALID_SOCKET;
        g_t_telsrv.t_tel_sem = NULL;
        g_t_telsrv.dw_snd_len = 0;
        g_t_telsrv.dw_cmd_len = 0;
    }

    do
    {
        /* if port is 0, use default port */
        if (w_port != 0)
        {
            g_t_telsrv.n_listen_sock = tel_open(0, w_port);
            if (g_t_telsrv.n_listen_sock != INVALID_SOCKET)
            {
                g_t_telsrv.w_port = w_port;
            }
        }
        
        if (g_t_telsrv.n_listen_sock == INVALID_SOCKET)
        {
            for (w_port = MIN_TELSVR_PORT; w_port < MAX_TELSVR_PORT; w_port++)
            {
                g_t_telsrv.n_listen_sock = tel_open(0, w_port); /* server's port */
                if (g_t_telsrv.n_listen_sock != INVALID_SOCKET) 
                {
                    g_t_telsrv.w_port = w_port;
                    break;
                }           
            }
        }
        
        if (g_t_telsrv.n_listen_sock == INVALID_SOCKET)
        {
            ret = SYS_API_TELNET_BIND_FAIL;
            break;
        }
        
        listen(g_t_telsrv.n_listen_sock, 1);
        
        if (!sys_api_b_sem_create(&g_t_telsrv.t_tel_sem))
        {
            ret = SYS_API_VOS_SEM_FAIL;
            break;
        }
        
        /*Create the server communicate with client pthread*/
        g_t_telsrv.t_telsrv_task = sys_api_task_create((void *)tel_proc, "TelenetServer", SYS_API_TELEECHO_TASKPRI,
            SYS_API_TELEECHO_STACKSIZE, NULL, &g_t_telsrv.dw_thread_id);
        
        if (0 == g_t_telsrv.t_telsrv_task)
        {
            ret = SYS_API_VOS_TASK_FAIL;
            break;
        }
    } while(0);
    
    return ret;
}


/* @Description:
 * end the telnet server module
 * @Parameters(in):
 * @Return value: 
 *  Success return true, fail return errno
 */
u16 tel_end()
{
    /* exit thread */
    g_t_telsrv.b_exit = true;
    if (0 != g_t_telsrv.t_telsrv_task)
    {
        sys_api_del_threadinfo(g_t_telsrv.dw_thread_id);
        sys_api_task_wait_end(g_t_telsrv.t_telsrv_task);
        g_t_telsrv.t_telsrv_task = 0;
    }
    if (NULL != g_t_telsrv.t_tel_sem)
    {
        sys_api_sem_delete(g_t_telsrv.t_tel_sem);
    }
    if (INVALID_SOCKET != g_t_telsrv.n_listen_sock)
    {
        close(g_t_telsrv.n_listen_sock);
        g_t_telsrv.n_listen_sock = INVALID_SOCKET;
    }
    if (INVALID_SOCKET != g_t_telsrv.n_client_sock)
    {
        close(g_t_telsrv.n_client_sock);
        g_t_telsrv.n_client_sock = INVALID_SOCKET;
    }
    g_t_telsrv.aby_cmd[0] = '\0';
    g_t_telsrv.dw_snd_len = 0;
    g_t_telsrv.dw_cmd_len = 0;
    return 0;
}

/* @Description:
 * open the socket to listen
 * @Parameters(in):
 * dw_ip: socket addr use 0
 * w_port:listen port
 * @Return value: 
 *  Success return true, fail return errno
 */
s32 tel_open(u32 dw_ip, u16 w_port)
{

    s32 n_sock = INVALID_SOCKET;
    struct sockaddr_in t_srv_addr;
    struct linger Linger;
    s32 yes = 1;
    s32 on = 1;
    
    memset( &t_srv_addr, 0, sizeof(t_srv_addr) );
    
    t_srv_addr.sin_family = AF_INET; 
    t_srv_addr.sin_port = htons(w_port);
    //add security support, only listen on from localhost
    //t_srv_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); //dw_ip;
    t_srv_addr.sin_addr.s_addr = inet_addr("0.0.0.0"); //dw_ip;
    
    n_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (n_sock == INVALID_SOCKET)
    {
        printf("\nsys_api_: Tcp server can't create socket!\n");
        return INVALID_SOCKET;
    }
    
    /* set socket option */
    Linger.l_onoff = 1; 
    Linger.l_linger = 0;

    setsockopt(n_sock, SOL_SOCKET, SO_LINGER, (s8*)&Linger, sizeof(Linger));
    setsockopt(n_sock, IPPROTO_TCP, TCP_NODELAY, (s8*)&yes, sizeof(yes));
    setsockopt(n_sock, SOL_SOCKET, SO_KEEPALIVE, (s8*)&yes, sizeof(yes));

    if (ioctl(n_sock, FIONBIO, &on) < 0)
    {
        printf("ioctl FIONBIO error\n");
        close(n_sock);
        return INVALID_SOCKET;
    }
        
    if (SOCKET_ERROR == setsockopt(n_sock, SOL_SOCKET, SO_REUSEADDR, (s8*)&yes, sizeof(yes)))
    {
		printf("*****set reuse error\n");
    }

    if (bind(n_sock, (struct sockaddr *)&t_srv_addr, sizeof(t_srv_addr)) == SOCKET_ERROR)
    {
        printf("bind error,w_port=%d\n", w_port);
        close(n_sock);
        return INVALID_SOCKET;
    }

    return n_sock;
}

#define TEL_EPOLL_MAXEVENTS   256
struct epoll_event g_epoll_event[TEL_EPOLL_MAXEVENTS];
static  char b_epoll_set = 0; //0:EPOLLIN;1:EPOLLIN | EPOLLOUT

static u16 tel_epoll(int efd)
{
     int n, i;
     s32 n_byte;	 
     int dw_wait_ms = 100;
     sys_api_sem_give(g_t_telsrv.t_tel_sem);	

     if (INVALID_SOCKET != g_t_telsrv.n_client_sock)
     {
        if (g_t_telsrv.dw_snd_len > 0)
        {
           if(b_epoll_set == 0)
           {
                struct epoll_event mod_event;
                mod_event.data.fd = g_t_telsrv.n_client_sock;
                mod_event.events = EPOLLIN | EPOLLOUT;
                if(epoll_ctl (efd,  EPOLL_CTL_MOD,  g_t_telsrv.n_client_sock,  &mod_event) == -1)
                {
                    perror ("epoll_ctl");
                }
                b_epoll_set	 = 1;
           }
        }
        else
        {
            if(b_epoll_set  == 1)
            {
                struct epoll_event mod_event;
                mod_event.data.fd = g_t_telsrv.n_client_sock;
                mod_event.events = EPOLLIN;
                if(epoll_ctl (efd,  EPOLL_CTL_MOD,  g_t_telsrv.n_client_sock,  &mod_event) == -1)
                {
                    perror ("epoll_ctl");
                }	
                b_epoll_set  =  0;
            }
        }
     }
	 
    n = epoll_wait (efd,    g_epoll_event,    TEL_EPOLL_MAXEVENTS,   dw_wait_ms);
    sys_api_sem_take(g_t_telsrv.t_tel_sem);	 

    if (INVALID_SOCKET != g_t_telsrv.n_client_sock)
    {
        /*check read*/
       for (i = 0; i < n; i++)
       {
     	   if (g_epoll_event[i].data.fd == g_t_telsrv.n_client_sock)
           {
                if(g_epoll_event[i].events & EPOLLIN)
                {                        
            		ioctl(g_t_telsrv.n_client_sock, FIONREAD, &n_byte);
            		if (n_byte<= 0)
            		{            		
            		        printf(" first  close\n");
            			if(epoll_ctl(efd, EPOLL_CTL_DEL,  g_t_telsrv.n_client_sock, &g_epoll_event[i]) == -1)	
                                {
	       				 perror ("epoll_ctl");
	 			}
                		tel_cli_close();				 	
                		return 0;
            		}
            		else
            		{
            		    //int  remeber_fd = g_t_telsrv.n_client_sock;
               			tel_cli_rcv ();
				        /*
	            		if (INVALID_SOCKET == g_t_telsrv.n_client_sock)
	            		{            		
	                        printf("efd:%d  second  close  %d\n", efd, remeber_fd);
				            if(epoll_ctl(efd, EPOLL_CTL_DEL,  remeber_fd, &g_epoll_event[i]) == -1)	
	                        {
		       				    perror ("epoll_ctl");
		 			        }				 	
	            		}
	            		*/
            		}
                }
     	   }
       	}

        if (g_t_telsrv.n_client_sock!= INVALID_SOCKET)
        {         
	       for (i = 0; i < n; i++)
	       {
	     	   if (g_epoll_event[i].data.fd == g_t_telsrv.n_client_sock)
	           {	               
	                if(g_epoll_event[i].events & EPOLLOUT)
	                {
                		tel_cli_snd();
	                }
	     	   }			   
					
            }
        }
    }

    /*check listen*/
    if (INVALID_SOCKET != g_t_telsrv.n_listen_sock)
    {   
       for (i = 0; i < n; i++)
       {
     	   if (g_epoll_event[i].data.fd == g_t_telsrv.n_listen_sock)
           {
                if(g_epoll_event[i].events & EPOLLIN)
                {
                    //clear send buffer
                    memset(g_t_telsrv.aby_snd_buf, 0, g_t_telsrv.dw_snd_len);
                    g_t_telsrv.dw_snd_len = 0;
		            tel_accept();
	                struct epoll_event new_event;
		            new_event.data.fd = g_t_telsrv.n_client_sock;
		            new_event.events = EPOLLIN;
                    if(epoll_ctl (efd,  EPOLL_CTL_ADD,  g_t_telsrv.n_client_sock,  &new_event) == -1)
                    {
                        perror ("epoll_ctl");
                    }
		            printf("accept efd:%d sock:%d\n", efd,  g_t_telsrv.n_client_sock);	
                }
     	    }
        } 	   
    }
    return 0;	 
}

/* @Description:
 * Telnet proc 
 * @Parameters(in):
 *  param:the thread input param
 */
static void* tel_proc(void* param)
{
    sys_thread_info_t t_thread_info;
    prctl(15, "telnet server");
    if (NULL == g_t_telsrv.t_tel_sem)
    {
        return NULL;
    }
    
    sprintf(t_thread_info.ach_thread_name,"telnet thread");
    t_thread_info.dw_thread_id = getpid();
    sys_api_add_threadinfo( &t_thread_info );
    sys_api_sem_take(g_t_telsrv.t_tel_sem);
    while (!g_t_telsrv.b_exit)
    {  
        tel_select();
    }

	printf("telnet server exit\n");
    return NULL;
}

/* @Description:
 * Telnet select , to recive the data from network
 */
static u16 tel_select()
{
    fd_set  t_read_set;
    fd_set  t_write_set;
    s32 n_byte;
    
    struct timeval *pTv;
    struct timeval tv;
    u32 dw_wait_ms = 100;
    s32 n_ready_cnt;
    if (INVALID_SOCKET == g_t_telsrv.n_listen_sock)
    {
        return SYS_API_TELNET_SERVER_FAIL;
    }
    FD_ZERO(&t_read_set);
    FD_ZERO(&t_write_set);
    FD_SET(g_t_telsrv.n_listen_sock, &t_read_set);

    if (INVALID_SOCKET != g_t_telsrv.n_client_sock)
    {
        FD_SET(g_t_telsrv.n_client_sock, &t_read_set);
        if (g_t_telsrv.dw_snd_len > 0)
        {
            FD_SET(g_t_telsrv.n_client_sock, &t_write_set);
        }
    }
    tv.tv_sec = dw_wait_ms/1000;
    tv.tv_usec = (dw_wait_ms%1000)*1000;
    pTv = &tv;
    
    sys_api_sem_give(g_t_telsrv.t_tel_sem);
    n_ready_cnt = select(FD_SETSIZE, &t_read_set, &t_write_set, NULL, pTv);
    
    sys_api_sem_take(g_t_telsrv.t_tel_sem);
    if (n_ready_cnt < 0)
    {
        sys_api_printf( "asflib select error\n");
        return 0;
    }
    if (INVALID_SOCKET != g_t_telsrv.n_client_sock)
    {
        /*check read*/
        if (FD_ISSET(g_t_telsrv.n_client_sock, &t_read_set))
        {
            ioctl(g_t_telsrv.n_client_sock, FIONREAD, &n_byte);
            if (n_byte<= 0)
            {
                tel_cli_close();
                return 0;
            }
            else
            {
               tel_cli_rcv ();
            }
        }
        if (g_t_telsrv.n_client_sock!= INVALID_SOCKET)
        {         
            if(FD_ISSET(g_t_telsrv.n_client_sock, &t_write_set))
            {
                tel_cli_snd();
            }
        }
    }
    /*check listen*/
    if (INVALID_SOCKET != g_t_telsrv.n_listen_sock)
    {   
        if (FD_ISSET(g_t_telsrv.n_listen_sock, &t_read_set))
        {
            tel_accept();
        }       
    }
    return 0;
}

/* @Description:
 * Recv the date from the network and echo on the screen
 * @Parameters(in):
 * @Return value: 
 *  Success return true, fail return errno
 */
static u16 tel_cli_rcv()
{
    s32 ret;
    u8 by_input_char;
    s32 n_try = 0;
    while(n_try < 3)
    {
        n_try++;
        ret = recv(g_t_telsrv.n_client_sock, (u8*)&by_input_char, 1, 0);
  
        if (ret <= 0)
        {
            if (errno != EWOULDBLOCK)
            {
                tel_cli_close();
                return 0;
            }
            else
            {
                if(n_try <= 3)
                {
                    sys_api_task_delay(50);
                }
                continue;    
            }
        }
        else
            break;
     }
    by_input_char = remove_iac(by_input_char);
    
    /*parse the cmd*/
    switch (by_input_char)
    {
    case 13:        /* '\r' -> "\r\n"*/
        tel_send("\r\n", 2);
        if (g_t_telsrv.dw_cmd_len > 0)
        {
            parse_cmd((s8*)g_t_telsrv.aby_cmd, g_t_telsrv.dw_cmd_len);
            g_t_telsrv.dw_cmd_len = 0;
        }
        if(g_t_telsrv.n_client_sock != INVALID_SOCKET)
        {
            prompt_show();
        }
        break;
        
    case 10:        /* '\n' : nop*/
        break;

    case 8:         /* Backspace in WinXP*/
    case 127:       /* Backspace in Linux*/
        {
            u8 aby_buf[] = {8, ' ', 8};            
            if (g_t_telsrv.dw_cmd_len > 0)
            {
                tel_send((const char*)aby_buf, sizeof(aby_buf));
                g_t_telsrv.dw_cmd_len--;
            }
        } break;

    default:
        tel_send((const char*)&by_input_char, 1);
        if (g_t_telsrv.dw_cmd_len < MAX_COMMAND_LENGTH)
        {               
            g_t_telsrv.aby_cmd[g_t_telsrv.dw_cmd_len++] = by_input_char;   
        }
        else
        {
            tel_send("\r\n", 2);
            parse_cmd((s8*)g_t_telsrv.aby_cmd, g_t_telsrv.dw_cmd_len);   
            g_t_telsrv.dw_cmd_len = 0;
            if(g_t_telsrv.n_client_sock != INVALID_SOCKET)
            {
                prompt_show();
            }            
        }
    }
    
    return 0;
}



/* @Description:
 * send the data to the telnet server
 * @Parameters(in):
 * @Return value: 
 *  Success return true, fail return errno
 */
u16 tel_send(const char* pch_buf, u32 dw_len)
{
    s32 n_send = 0;
    if (INVALID_SOCKET == g_t_telsrv.n_client_sock)
    {
        return SYS_API_TELNET_NOT_CONNECT;
    }

    if (NULL == pch_buf)
    {
        return SYS_API_PARAM_ERROR;
    }

    if (g_t_telsrv.dw_snd_len == 0)
    {
        s32 ret;
        u32 dw_cnt = 0;
        s32 dw_need_snd = dw_len;
        while(dw_cnt < 3 && n_send < dw_len)
        {
            dw_cnt++;
            ret = send(g_t_telsrv.n_client_sock, pch_buf+n_send, dw_need_snd, MSG_NOSIGNAL);
            if (ret < 0)
            {
                if (errno != EWOULDBLOCK)
                {
                    tel_cli_close();
                    return SYS_API_TELNET_SOCK_ERROR;
                }
                else
                {
                    if (dw_cnt > 3 )
                    {
                        break;
                    }
                    else
                    {
                        sys_api_task_delay(50);
                        continue;
                    }
                }
            }
            else
            {
                n_send += ret;
                dw_need_snd -= ret;
            }
        }
    }
    
    if (n_send < (s32)dw_len)
    {
        /* modify send buff, need take semphone */
        u16 result = 0;
        s32 n_remain;

        TEL_SEM_TAKE(g_t_telsrv.t_tel_sem);

        n_remain = dw_len - n_send;
       
        if (g_t_telsrv.dw_snd_len + n_remain > sizeof(g_t_telsrv.aby_snd_buf))
        {
            /* overflow*/
            result = SYS_API_TELNET_SEND_OVERFLOW;
        }
        else
        {
            memcpy(g_t_telsrv.aby_snd_buf + g_t_telsrv.dw_snd_len, pch_buf+n_send, n_remain);
            g_t_telsrv.dw_snd_len += n_remain;
        }

        TEL_SEM_GIVE(g_t_telsrv.t_tel_sem);
        
        return result;
    }

    return 0;
}

/* @Description:
 * Internal send the data to the tel server
 * @Parameters(in):
 * @Return value: 
 *  Success return true, fail return errno
 */
static u16 tel_cli_snd()
{
    s32 ret=0;
    u32 dw_try = 0;
    s32 n_send = 0;
    s32 n_total = 0;
    if (g_t_telsrv.dw_snd_len == 0 ||g_t_telsrv.n_client_sock==INVALID_SOCKET)
    {
        return 0;
    }
    n_total =  g_t_telsrv.dw_snd_len;
    
    while (dw_try < 3&& n_send < n_total)
    {
        dw_try++;
        ret = send(g_t_telsrv.n_client_sock, g_t_telsrv.aby_snd_buf+n_send,n_total, 0);
        if (ret < 0)
        {
            if (errno != EWOULDBLOCK)
            {
                close(g_t_telsrv.n_client_sock);
                g_t_telsrv.n_client_sock = INVALID_SOCKET;
                break;
            }
            else
            {
                if (dw_try <= 3)
                {
                    sys_api_task_delay(50);
                }
                continue;
            }
        }
        else
        {
            n_send += ret;
            n_total -= ret;
        }
    }

    if (n_send < (s32)g_t_telsrv.dw_snd_len && (g_t_telsrv.n_client_sock != INVALID_SOCKET))
    {
        g_t_telsrv.dw_snd_len -= n_send;
        memmove(g_t_telsrv.aby_snd_buf, g_t_telsrv.aby_snd_buf+ret, g_t_telsrv.dw_snd_len);
        return 0;
    }

    if (g_t_telsrv.n_client_sock == INVALID_SOCKET)
    {
        memset(g_t_telsrv.aby_snd_buf, 0, g_t_telsrv.dw_snd_len);
    }

    g_t_telsrv.dw_snd_len = 0;
    
    return 0;
}

/* @Description:
 * close the telnet client socket
 * @Parameters(in):
 * @Return value: 
 *  Success return true, fail return errno
 */
static u16 tel_cli_close()
{
    close(g_t_telsrv.n_client_sock);
    g_t_telsrv.n_client_sock = INVALID_SOCKET;
    return 0;
}

/* @Description:
 * send the data to the telnet server
 * @Parameters(in):
 * @Return value: 
 *  Success return true, fail return errno
 */
static u16 tel_accept()
{
    struct sockaddr_in t_addr;
    socklen_t n_len = (socklen_t)sizeof(t_addr);
    s32 n_sock;
    memset(&t_addr, 0, sizeof(t_addr));
    u32 dw_on = 1;
ACCEPT_AGAIN:
    n_sock = accept(g_t_telsrv.n_listen_sock, (struct sockaddr *)&t_addr, &n_len);

    if (n_sock == -1 )
    {
        printf("tel_accept: accept Error[%d]\n", errno);
        if (errno == EINTR)
            goto ACCEPT_AGAIN;
        else
            return -1;
    }
    /* use the new client, need close old client */
    if (g_t_telsrv.n_client_sock != INVALID_SOCKET) 
    {   
        close(g_t_telsrv.n_client_sock);

        g_t_telsrv.n_client_sock = INVALID_SOCKET;
    }

    if (ioctl(n_sock, FIONBIO, &dw_on) < 0)
    {
        printf("cannot set telnet client socket no block, socket:%d, errno:%d\n", n_sock, errno);
    }
 
    g_t_telsrv.n_client_sock = n_sock;
       
    /*printf the welcome & prompt chart*/
    send_ac(TELCMD_DO, TELOPT_ECHO);
    send_ac(TELCMD_DO, TELOPT_NAWS);
    send_ac(TELCMD_DO, TELOPT_LFLOW);
    send_ac(TELCMD_WILL, TELOPT_ECHO);
    send_ac(TELCMD_WILL, TELOPT_SGA);
        
    tel_print("*===============================================================\n");
    tel_print("welcome to debug telnet server\n");
    tel_print("*===============================================================\n");
        
    prompt_show();
    return 0;
}


/* telnet */
enum tel_state { tel_normal = 0, tel_nego = 1, tel_sub = 2 };

static s32 seen_iac = 0;
static enum tel_state state;
static s32 count_after_sb = 0;

static s8 remove_iac(u8 c)
{
    s8 ret = 0;
    if ((c == 255) && !seen_iac)    /* IAC*/
    {
        seen_iac = 1;
        return ret;
    }
    
    if (seen_iac)
    {
        switch(c)
        {
        case 251:
        case 252:
        case 253:
        case 254:
            if (state != tel_normal) 
            {
                printf(" illegal negotiation.\n"); 
            }
            state = tel_nego;
            break;
        case 250:
            if (state != tel_normal)
            {
                printf(" illegal sub negotiation.\n"); 
            }
            state = tel_sub;
            count_after_sb = 0;
            break;
        case 240:
            if (state != tel_sub) 
            {
                printf(" illegal sub end.\n"); 
            }
            state = tel_normal;
            break;
        default:
            if (!((c > 240) && (c < 250) && (state == tel_normal)))
            {
                printf("illegal command.\n"); 
            }
            state = tel_normal;
        }
        seen_iac = 0;
        return 0;
    }
    
    switch (state)
    {
    case tel_nego:
        state = tel_normal;
        break;
    case tel_sub:
        count_after_sb++; /* set maximize sub negotiation length*/
        if (count_after_sb >= 100) 
            state = tel_normal;
        break;
    default:
        ret = c;
    }
    return ret;
}


typedef struct 
{
    s8 *paraStr;
    s32 bInQuote;
    s32 bIsChar;
}TRawPara;

reg_func_t find_cmd(const char* name)
{
    s32 i;
    for(i = 0; i < g_i_cmd_idx; i++)
    {
        if (strcmp(g_t_cmd_table[i].ach_name, name) == 0)
            return g_t_cmd_table[i].t_cmd;
    }
    return NULL;
}

static void run_cmd(char *command)
{  
    s32 cmdLen = strlen(command) + 1;
    s32 n;
    s32 argc;
    char *argv[100];
    reg_func_t func;

    /* Replace all the spaces with '\0'. */
    for (n = 0; command[n] != '\0' && n < cmdLen; n++) {
        if (isspace(command[n])) {
            command[n] = '\0';
        }
    }

    /* Get all the command parameters. */
    for (argc = 0, n = 0; n < cmdLen; ) {
        if (command[n] == '\0') {
            n++;
        }
        else {
            argv[argc++] = command + n;
            if (argc >= 100)
	
                return;
            n += strlen(command+n);
        }
    }

    if (strcmp("bye", command) == 0 || strcmp("exit", command) == 0) {
        tel_print("\n  bye......\n");
        close(g_t_telsrv.n_client_sock);
        g_t_telsrv.n_client_sock = INVALID_SOCKET;        
        return;
    }

    if (strcmp("help", command) == 0) {
        sys_api_help();
        return;
    }

    /* Execute the function with parameters. */
    func = find_cmd(argv[0]);
    if (func != NULL) 
    {
	    //func may wait the result from another thread whiches call sys_api_printf to take the lock
        sys_api_sem_give(g_t_telsrv.t_tel_sem);		
        n = (*func)(argc, (char **)argv);
        sys_api_sem_take(g_t_telsrv.t_tel_sem);	 		
    }
    else 
    {
        tel_print("Function doesn't exist!\n");
    }

    return ;
}

/* @Description:
 * parse the user cmd
 * @Parameters(in):
 * pch_cmd: cmd string
 * dw_cmd_len:cmd len
 * @Return value: 
 *  Success return true, fail return errno
 */
static void parse_cmd(s8* pch_cmd, u32 dw_cmd_len)
{  
    u8 count;
    s32 nCpyLen = 0;
    char command[MAX_COMMAND_LENGTH] = {0};

    for(count = 0; count < dw_cmd_len; count++)
    {
        s8 chTmp;

        chTmp = pch_cmd[count];

        if (isdigit(chTmp) || islower(chTmp) || isupper(chTmp))
        {
            break;
        }
    }

    nCpyLen = dw_cmd_len-count;
    if(nCpyLen <= 0)
    {
        return;
    }

    memcpy(command, pch_cmd+count, nCpyLen);   
    if (dw_cmd_len < MAX_COMMAND_LENGTH)
    {
        command[nCpyLen] = '\0';
    }
    else
    {
        command[MAX_COMMAND_LENGTH-1] = '\0';
    }

    run_cmd(command);
}


/* @Description:
 * show the prompt character
 */
static void prompt_show(void)
{
    char achModName[MAX_MODNAME_LEN] = {0};
    char *pchBaseModName = NULL;
    char prompt[MAX_PROMPT_LENGTH+4] = {0};
    
    /* if user set prompt, use it */
    if(strlen(g_t_telsrv.ach_prompt) > 0)
    {
        sprintf(prompt, "%s->", g_t_telsrv.ach_prompt);
        tel_print(prompt);
        return;
    }

    if (NULL != getcwd(achModName, sizeof(achModName)))
    {
        pchBaseModName = (char*)basename(achModName);
    }

    if(pchBaseModName != NULL)
    {
        //sprintf(prompt, "%s->", pchBaseModName);
        sprintf(prompt, "->");
    }
    else
    {
        sprintf(prompt, "#");
    }

    tel_print(prompt);
}

/* @Description:
 * telnet print
 */
u16 tel_print(const char* pch_msg)
{
    u32 dwStart = 0;
    u32 dwCount = 0;
    s16 OutputCount;
    char OutputBuff[MAX_COMMAND_LENGTH] = {0};
    if (pch_msg == NULL || g_t_telsrv.n_client_sock == INVALID_SOCKET)
    {
        return SYS_API_TELNET_NOT_CONNECT;
    }

    dwStart = 0;
    dwCount = strlen(pch_msg); 
    OutputCount = 0;
    
    while(dwStart < dwCount)
    {
        s8 c = pch_msg[dwStart];
        /* convert "\r","\n" to "\r\n" */
        if (( c == '\n' ) || (c == '\r'))
        {
            OutputBuff[OutputCount++] = '\r';
            OutputBuff[OutputCount++] = '\n';
        } 
        else 
        { 
            OutputBuff[OutputCount++] = c;
        }

        if (OutputCount >= (MAX_COMMAND_LENGTH - 1))
        {
            u16 ret;
            ret = tel_send(OutputBuff, OutputCount);
            if (ret != 0) return 0;
            OutputCount = 0;
        }
        dwStart++;
    }
    
    return tel_send(OutputBuff, OutputCount);
}

static void send_ac(s8 cmd, s8 opt)
{
    u8 buf[5];
    buf[0] = TELCMD_IAC;
    buf[1] = cmd;
    buf[2] = opt;
    tel_send((const char*)buf, 3);
}
