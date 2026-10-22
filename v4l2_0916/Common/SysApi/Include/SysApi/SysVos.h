
/*================================================================================
                                                                               
Module Name:  SysVos.h

General Description: wrapper OS API  header file 

==================================================================================*/

#pragma once

typedef void *(*LINUXFUNC)(void *);

typedef struct tag_sys_thread_list
{
	sys_thread_info_t t_thread;
	struct tag_sys_thread_list* pt_next;      
}sys_thread_list_t;

typedef struct
{
    bool b_init;
    s32 b_exit;
    sys_thread_list_t* pt_used_list;     	/*used list*/
    sys_thread_list_t* pt_free_list_head;   /*free list header*/  
    sys_thread_list_t* pt_free_list_tail;    	/*free list tail*/   
    sys_thread_list_t* pt_total_list;  		/*total list*/
    u32 dw_total_cnt;      				/*thread count*/
    sem_handle_t t_sem;         	   		/*thread mgr sem*/
} sys_thread_mgr_t;

