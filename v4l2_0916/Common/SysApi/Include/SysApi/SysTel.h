
/*================================================================================
                                                                               
Module Name:  SysTel.h

General Description: telnet server module header

==================================================================================*/


#pragma once

/* Telnet server's port range */
#define MIN_TELSVR_PORT 2500
#define MAX_TELSVR_PORT 8000 
#define MAX_PATH 255

#define SYS_API_TELEECHO_TASKPRI 70
#define SYS_API_TELEECHO_STACKSIZE (1000<<10)

#define MAX_MODNAME_LEN MAX_PATH
#define MAX_MOD_NUM 20

#define MAX_COMMAND_LENGTH 255
#define MAX_SEND_LENGTH 2048

#define MAX_PROMPT_LENGTH 32


#define TELCMD_WILL    (u8)251
#define TELCMD_WONT    (u8)252
#define TELCMD_DO      (u8)253
#define TELCMD_DONT    (u8)254
#define TELCMD_IAC     (u8)255

#define TELOPT_ECHO     (u8)1
#define TELOPT_SGA      (u8)3
#define TELOPT_LFLOW    (u8)33
#define TELOPT_NAWS     (u8)34

#define NEWLINE_CHAR    '\n'
#define BACKSPACE_CHAR  8
#define BLANK_CHAR      ' '
#define RETURN_CHAR     13
#define TAB_CHAR        9
#define DEL_CHAR        127
#define CTRL_S          19
#define CTRL_R          18
#define UP_ARROW        27
#define DOWN_ARROW      28
#define LEFT_ARROW      29
#define RIGHT_ARROW     30

#define CMD_NAME_SIZE       32
#define CMD_USAGE_SIZE      200
#define CMD_TABLE_SIZE      256
/*prompt len*/
//#define MAX_PROMPT_LEN      21



#define SOCKET_ERROR          (-1)
#define INVALID_SOCKET        (-1)

/* UniformFunc: definition for all functions those can invoked by user through telnet */
//typedef int (*reg_func_t)(s32,s32,s32,s32,s32,s32,s32,s32,s32,s32);
typedef int (*reg_func_t)(int argc, char *argv[]);


typedef struct {
    char ach_name[CMD_NAME_SIZE];    /* Command Name; less than 30 bytes */
    reg_func_t t_cmd;    /* Implementation function */
    char ach_usage[CMD_USAGE_SIZE];    /* Usage message */
} cmd_table_t;

typedef struct 
{
    u16 w_port;
    u8  aby_cmd[MAX_COMMAND_LENGTH];
    u32 dw_cmd_len;
    char  ach_prompt[MAX_PROMPT_LENGTH];
    u8  aby_snd_buf[MAX_SEND_LENGTH];
    u32 dw_snd_len;
    bool b_exit; 
    task_handle_t t_telsrv_task;
    task_handle_t dw_thread_id;
    s32 n_listen_sock;
    s32 n_client_sock;
    sem_handle_t t_tel_sem;
} tel_server_t;

u16 tel_print(const char* pch_msg);
u16 tel_init(bool b_tel, u16 w_port);
u16 tel_end();

