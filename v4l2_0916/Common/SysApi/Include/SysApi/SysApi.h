/*================================================================================
                                                                               
Module Name:  SysApi.h

General Description: SysApi header file 

==================================================================================*/

#pragma once

/******************************************************************************
    Linux header files, need to be cleaned up if not needed
******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>
#include <malloc.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/times.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <signal.h>
#include <ctype.h>
#include <time.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <cstdint>

/* Type definition */
typedef uint32_t    u32;
typedef int32_t    s32;
typedef uint8_t   u8;
typedef uint16_t  u16;
typedef int16_t s16;
typedef int8_t s8;
typedef int64_t s64;
typedef uint64_t u64;

#define SYS_API_VER          "sys api version 0.2"  /* Version */
#define SYS_API_THREAD_NUMBER               100     

typedef enum{
	SYS_API_ERROR_CODE_BASE = 5000,
	SYS_API_PARAM_ERROR,
	SYS_API_MALLOC_ERROR,
	SYS_API_ERROR_UUIDSTR,

	/*Vos module*/
	SYS_API_VOS_ERR_BASE = 5100,
	SYS_API_VOS_SEM_FAIL,
	SYS_API_VOS_TASK_FAIL,

	/*Tel module*/
	SYS_API_TEL_ERR_BASE = 5200,
	SYS_API_TEL_NOT_INIT,
	SYS_API_TELNET_BIND_FAIL,
	SYS_API_TELNET_NOT_CONNECT,
	SYS_API_TELNET_SERVER_FAIL,
	SYS_API_TELNET_SEND_OVERFLOW,
	SYS_API_TELNET_SOCK_ERROR,

	/*Shm module*/
	SYS_API_SHM_ERR_BASE = 5400,

	SYS_API_ERR_MAX
}sys_api_errno;

typedef  pthread_cond_t* sem_handle_t;
typedef  pthread_t task_handle_t;

typedef struct
{
	pthread_cond_t t_cond;
	pthread_mutex_t t_mutex;
	s32 n_cur_cnt;
	u32 dw_max_cnt;
}sys_cond_t;

/*****************sys_api module manage****************/

/* @Description:
 * initialize SYS_API module
 * There is only one sys_api lib for one process
 * @Parameters(in):
 * b_tel: telnet is enable or not. 
 * w_port: telnet server port.if you set 0, it means that will choose a unused port(from 2500)
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_init(bool b_tel, u16 w_port);


/* @Description:
 * check if SYS_API module is already init
 * @Parameters(in):
 * @Return value: 
 *  already exit return true, not exit return false
 */
bool is_sys_api_initd(void);

/* @Description:
 * exit SYS_API module
 * @Parameters(in):
 * @Return value: 
 *  Success return true, fail return false
 */
void sys_api_quit(void);

/* @Description:
 * pritnf the sys_api version
 */
void sys_api_ver();

/* @Description:
 * pritnf the sys_api lib help infomation
 */
void sys_api_help();

/* @Description:
 * register the telnet function for debug
 * @Parameters(in):
 * pch_name:cmd name;
 * p_func: function point;
 * pch_usage: function help infomation
 */
void sys_api_reg_cmd(const char* pch_name, void* p_func, const char* pch_usage);


/* @Description:
 * printf the msg to the telent port
 * @Parameters(in):
 * pch_format:print infomation;
 */
void sys_api_printf(const char *pch_format, ...);
void sys_api_printf2(const char* format, va_list args);

/******************pthread module *********************/
typedef enum {
    SYS_SCHED_MIN = 0,
    SYS_SCHED_RR,/* For linux */
    SYS_SCHED_FIFO, /* For linux */
    SYS_SCHED_OTHER,/* For linux */
    /* Add other sched type*/
    SYS_SCHED_MAX 
} sys_sched_t; 

/* @Description:
 * Create a pthread binary sem
 * @Parameters(out):
 * *pt_sem will take the sem handle to the user
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_b_sem_create(sem_handle_t *pt_sem);

/* @Description:
 * Create a pthread polybasic sem
 * @Parameters(in):
 * dw_init_cnt: init count, dw_max_cnt :max count
 * @Parameters(out):
 * *pt_sem will take the sem handle to the user
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_c_sem_create(sem_handle_t *pt_sem, u32 dw_init_cnt, u32 dw_max_cnt);

/* @Description:
 * Delete a pthread sem
 * @Parameters(in):
 * t_sem : the sem handle
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_sem_delete(sem_handle_t t_sem );

/* @Description:
 * Take a pthread sem,sem "P" operation,the task will be hung up
 * @Parameters(in):
 * t_sem : the sem handle
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_sem_take(sem_handle_t t_sem );

/* @Description:
 * Take a pthread sem,sem "P" operation,the task will be hung up,but when time pass the dwtime(ms), task will be unblock
 * @Parameters(in):
 * t_sem : the sem handle
 * dw_time:hung up time
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_sem_take_bytime(sem_handle_t t_sem, u32 dw_time);

/* @Description:
 * Take a pthread sem,sem "V" operation,the task will be release
 * @Parameters(in):
 * t_sem : the sem handle
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_sem_give(sem_handle_t t_sem );


/* @Description:
 * Show all the sem create by sys_api
 */
void sys_api_sem_show();

/* @Description:
 * Create a pthread,and process the pthread
 * @Parameters(in):
 * p_task : the thread entry func point.
    ex:
     API void MyTask(void)
     {...}  
     sys_api_task_create((void *)(u32)MyTask, ...);
 * pch_name: task name, a string
 * by_priority: task priority
 * dw_stack_size: task stack size
 * dw_data: the user data,which want to pass to the task
 * @Parameters(in):
 * pdw_taskID:task id
 * @Return value: 
 *  Success return task handle, fail return NULL
 */
task_handle_t sys_api_task_create(
	void* p_task, 
	const char* pch_name, 
	u8 by_priority, 
	u32 dw_stack_size, 
	void *dw_data,
	task_handle_t *pdw_taskID);


/* @Description:
 * pthread exit(no wait),user must release the resoure for the task by himself 
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_task_exit(void);

/* @Description:
 * pthread exit(wait for the specified task exit),user must release the resoure for the task by himself 
 * @Parameters(in):
 * t_task:the specified task handle
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_task_wait_end(task_handle_t t_task);

/* @Description:
 * pthread cancel,user must release the resoure for the task by himself  
 * @Parameters(in):
 * t_task:the specified task handle
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_task_terminate(task_handle_t t_task);

/* @Description:
 * set the pthread priorty
 * @Parameters(in):
 * handle:the specified task handle,newPriority: priority value 
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_task_set_priority(task_handle_t t_task, u8 by_priority);

/* @Description:
 * set the pthread priorty
 * @Parameters(in):
 * t_task:the specified task handle
 * @Parameters(out):
 * pby_priority: get the priority value
 * @Return value: 
 *  Success return true, fail return false
 */
s32 sys_api_task_get_priority(task_handle_t t_task, u8 *pby_priority);

/* @Description:
 * set the pthread sched type
 * @Parameters(in):
 * handle:the specified task handle,newSched:  sys_sched_t value 
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_task_set_sched(task_handle_t t_task, sys_sched_t sched);

/* @Description:
 * set the pthread sched type
 * @Parameters(in):
 * newSched:  sys_sched_t value 
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_task_set_self_sched(sys_sched_t sched);

/* @Description:
 * get the current task handle
 * @Return value: 
 *  current taskhandle transfer to u32
 */
//$id 2014-12-26
//u32 sys_api_task_self_id(void);
task_handle_t sys_api_task_self_id(void);


/* @Description:
 * set the task hung up for server ms
 * @Parameters(in):
 dwMseconds: set specfic ms
 */
void sys_api_task_delay(u32 dw_ms);

/************************system status & info***********************/

/* CPU info*/
typedef struct
{
    /*CPU load persent*/
    u8 by_idle_per;
}sys_cpu_info_t;

/*memory info */
typedef struct
{
    /* physical memory (KByte) */
    u32 dw_phy_size;
    /* uesd memory (KBytes) */
    u32 dw_alloc_size;
    /* free memory(KBytes) */
    u32 dw_free_size;
}sys_mem_info_t;

/* time info */
typedef struct
{   
    u16 w_year;
    u16 w_month;
    u16 w_day;
    u16 w_hour;
    u16 w_minute;
    u16 w_second;
}sys_time_info;

/*thread info*/
typedef struct tag_sys_thread_info
{
    char ach_thread_name[100];                /* pthread name*/
    u32 dw_thread_id;                      /* pthread id*/
}sys_thread_info_t;

/* @Description:
 * get the current system memory info  
 * @Parameters(in):
 * pt_mem_info:the memory infomation 
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_get_meminfo(sys_mem_info_t* pt_mem_info );

/* @Description:
 * get the current system CPU info ,like %percent top
 * @Parameters(in):
 * pt_cpu_info:the CPU infomation 
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_get_cpuinfo( sys_cpu_info_t* pt_cpu_info );

/* @Description:
 * get the current system time info 
 * @Parameters(in):
 * pt_time_info:the system time infomation 
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_get_timeinfo( sys_time_info* pt_time_info );

/* @Description:
 * Set the current system time info 
 * @Parameters(in):
 * pt_time_info:the system time infomation 
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_set_timeinfo( sys_time_info* pt_time_info );

/* @Description:
 * Add the thread to the sys_api lib for managment
 * @Parameters(in):
 * pt_thread_info:the thread info
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_add_threadinfo( sys_thread_info_t* pt_thread_info );

/* @Description:
 * Del the thread from the sys_api lib 
 * @Parameters(in):
 * dwThreadId:the thread id
 * @Return value: 
 *  Success return true, fail return false
 */
bool sys_api_del_threadinfo( u32 dw_thread_id );

/* @Description:
 * print the thread info which manage by sys_api lib
 */
void sys_api_print_threadinfo();

/* @Description:
 * get the cpu tick precision (tick/ms)
 * @Return value: 
 *  the cpu tick precision
 */
u32 sys_api_clk_rate_get();

/* @Description:
 * get the cpu tick value
 * @Return value: 
 *  the cpu tick value
 */
u32 sys_api_tick_get();

/*******************************************/
/*Share memory and process sem used for mutil process */
/*							                               */
/******************************************/
typedef struct shm_sem_parm
{
	const char ch_path[32];			/*the Shm path,just for namespace*/
	u32 n_shm_size;				/*how long for the shm size, just can be set at the open, can't be edit*/
	s32 n_sem_num;				/*for a sem group, how many sem for each group*/
	s32 n_sem_val;				/*init value for each sem*/
}shm_sem_parm_t;

/* @Description:
 * open a shm
 * @Parameters(in):
 * pt_shm_sem:the shm parameter 
 * @Parameters(out):
 * pn_shm_id:shm id, it is the handle to operate the shm
 * pn_sem_id:sem id, it it the sem handle to lock the shm
 * @Return value: 
 *  fail return NULL, Sucess return addr of the shm ,it had been attach with the process space memory
 */
void *sys_api_shm_open(shm_sem_parm_t *pt_shm_sem,s32 *pn_shm_id,s32 *pn_sem_id);

/* @Description:
 * close a shm
 * @Parameters(in):
 * p_addr:the shm addr 
 * n_shm_id:shm id, it is the handle to operate the shm
 * n_sem_id:sem id, it it the sem handle to lock the shm
 * @Return value: 
 * fail return errno, Sucess return 0
 */
s32 sys_api_shm_close(s32 n_shm_id,s32 n_sem_id,const void *p_addr);

/* @Description:
 * wait a sem of shm
 * @Parameters(in):
 * n_sem_id:sem id, it it the sem handle to lock the shm
 * n_idx : which sem in the sem group
 * dw_wait_ms: try to wait time 
 * @Return value: 
 * fail return errno, Sucess return true
 */
bool sys_api_shm_wait(s32 n_sem_id,s32 n_idx,u32 dw_wait_ms);

/* @Description:
 * get a sem of shm
 * @Parameters(in):
 * n_sem_id:sem id, it it the sem handle to lock the shm
 * n_idx : which sem in the sem group
 * @Return value: 
 * fail return -1, Sucess return true
 */
bool sys_api_shm_get(s32 n_sem_id,s32 n_idx);

/* @Description:
 * put a sem of shm
 * @Parameters(in):
 * n_sem_id:sem id, it it the sem handle to lock the shm
 * n_idx : which sem in the sem group
 * @Return value: 
 * fail return -1, Sucess return true
 */
bool sys_api_shm_put(s32 n_sem_id,s32 n_idx);

