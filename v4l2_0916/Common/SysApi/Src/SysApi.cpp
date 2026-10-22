
/*================================================================================
                                                                               
Module Name:  SysApi.cpp

General Description: SysApi module mgr 

==================================================================================*/

#include <SysApi/SysApi.h>
#include <SysApi/SysTel.h>
#include <SysApi/SysVos.h>
#include <sys/syscall.h>
#include <time.h>

#define SYS_THREAD_INFO_NUM 100
#define MAX_LOG_MSG_LEN 6000    /*log max length*/

cmd_table_t g_t_cmd_table[CMD_TABLE_SIZE];
s32  g_i_cmd_idx = 0;
bool g_b_sys_api_init = false;
sys_thread_mgr_t g_t_thread_mgr = {0};

/* @Description:
 * Init the sys_api lib thread mgr info
 * @Parameters(in):
 * dwTotalNum:Total sys_api lib thread mgr num
 * @Return value: 
 *  Success return true, fail return false
 */
static bool thread_info_init(u32 dwTotalNum)
{    
    if(!g_t_thread_mgr.b_init)
    {
        u32 i;

        /* set init flag */
        g_t_thread_mgr.b_init = true;

        /* malloc thread record buff */
        g_t_thread_mgr.pt_total_list = (sys_thread_list_t*)malloc(sizeof(sys_thread_list_t) * dwTotalNum);
        if (NULL == g_t_thread_mgr.pt_total_list)
        {
            sys_api_task_delay(100);
            return false;
        }

	  memset(g_t_thread_mgr.pt_total_list,0,sizeof(sys_thread_list_t) * dwTotalNum);
        g_t_thread_mgr.dw_total_cnt = dwTotalNum;
        
        /* Create Semaphone */
        if (true != sys_api_b_sem_create(&g_t_thread_mgr.t_sem))
        {
            sys_api_task_delay(100);
            return false;
        }

        /* init thread table */
        g_t_thread_mgr.pt_free_list_head = g_t_thread_mgr.pt_total_list;
        g_t_thread_mgr.pt_free_list_tail = g_t_thread_mgr.pt_total_list;
        g_t_thread_mgr.pt_used_list = NULL;
        for(i = 1; i < dwTotalNum; i++)
        {
            g_t_thread_mgr.pt_free_list_tail->pt_next = g_t_thread_mgr.pt_total_list + i;
            g_t_thread_mgr.pt_free_list_tail = g_t_thread_mgr.pt_free_list_tail->pt_next;
        }
        g_t_thread_mgr.pt_free_list_tail->pt_next = NULL;
    }
    else
    {
        sys_api_task_delay(100);
        return false;
    }
    return true;
}

/* @Description:
 * Release the sys_api lib thread mgr info
 * @Return value: 
 *  Success return true, fail return false
 */
static bool thread_info_quit()
{
    if (g_t_thread_mgr.b_init)
    {

        if( g_t_thread_mgr.pt_total_list != NULL)
        {
            free(g_t_thread_mgr.pt_total_list);
            g_t_thread_mgr.pt_total_list        = NULL;
        
            g_t_thread_mgr.pt_free_list_head      = NULL;
            g_t_thread_mgr.pt_free_list_tail          = NULL;           
        }

        if( g_t_thread_mgr.t_sem )
        {
            sys_api_sem_delete(g_t_thread_mgr.t_sem);
            g_t_thread_mgr.t_sem = NULL;
        }
        g_t_thread_mgr.b_init = false;
        g_t_thread_mgr.dw_total_cnt = 0;
        return true;
    }
    else
    {
        return false;
    }
}

bool sys_api_init(bool b_tel, u16 w_port)
{
    bool b_ret = false;
    
    if(g_b_sys_api_init)
    {
        printf("sys_api has been intiated before!");
        return false;
    }

    /*Thread list init */
    thread_info_init(SYS_THREAD_INFO_NUM);

    /*telnet server init*/
    b_ret=tel_init(b_tel,w_port);
    if(b_ret)
	{
	    printf("telnet init faild b_tel = %d, w_port = %d\n",b_tel,w_port);
        return false;
	}
    
    sys_api_printf("sys_api: initial success!\n");
    sys_api_reg_cmd("help", (void*)sys_api_help, " - Display this help and exit");
	g_b_sys_api_init = true;

    return true;
}


bool is_sys_api_initd(void)
{
    bool b_init = false;

    sys_api_sem_take(g_t_thread_mgr.t_sem);
    b_init = g_t_thread_mgr.b_init;
    sys_api_sem_give(g_t_thread_mgr.t_sem);

    return b_init;  
}

void sys_api_quit(void)
{
    tel_end();
    thread_info_quit();
}


void sys_api_ver()
{
    sys_api_printf("sys_api version: %s  compile time: %s %s\n", SYS_API_VER, __DATE__, __TIME__);
}


void sys_api_help()
{
    s32 indx = 0;
    
    sys_api_ver();

    /* Traverse the command array. --slei */
    for (indx = 0; indx < g_i_cmd_idx; indx++)
    {
        sys_api_printf("[%s] usage: %s.\n", 
            g_t_cmd_table[indx].ach_name, g_t_cmd_table[indx].ach_usage);
    }

    return;
}

void sys_api_printf2(const char* format, va_list args)
{
    char ach_msg[MAX_LOG_MSG_LEN]={0};
    u32 dw_len = 0;
    u16 w_ret = 0; 

    if(format == NULL)
    {
        return;
    }
    //add time and thread id
    time_t time_now = time(NULL);
    struct tm* ptm = localtime(&time_now);
    int size_time = sprintf(ach_msg,"[%d/%d/%d-%02d:%02d:%02d]",ptm->tm_mday, (ptm->tm_mon+1), (ptm->tm_year+1900),ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
	int size_thread = sprintf(ach_msg+size_time,"[%lu]",syscall(SYS_gettid));       
    dw_len = vsnprintf(ach_msg+size_time+size_thread, MAX_LOG_MSG_LEN-1-size_time-size_thread, format, args); 

    if (dw_len <= 0 || dw_len >= MAX_LOG_MSG_LEN)
    {
        return;
    }
    w_ret = tel_print(ach_msg);
}

void sys_api_printf(const char *pch_format, ...)
{
    va_list pvList;
    char ach_msg[MAX_LOG_MSG_LEN]={0};
    u32 dw_len = 0;
    u16 w_ret = 0; 

    if(pch_format == NULL)
    {
        return;
    }
    //add time and thread id
    time_t time_now = time(NULL);
    struct tm* ptm = localtime(&time_now);
    int size_time = sprintf(ach_msg,"[%d/%d/%d-%02d:%02d:%02d]",ptm->tm_mday, (ptm->tm_mon+1), (ptm->tm_year+1900),ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
	int size_thread = sprintf(ach_msg+size_time,"[%lu]",syscall(SYS_gettid));       
    va_start(pvList, pch_format); 
    dw_len = vsnprintf(ach_msg+size_time+size_thread, MAX_LOG_MSG_LEN-1-size_time-size_thread,pch_format, pvList); 
    va_end(pvList);
    if (dw_len <= 0 || dw_len >= MAX_LOG_MSG_LEN)
    {
        return;
    }
    w_ret = tel_print(ach_msg);
}

void sys_api_reg_cmd(const char* name, void* func, const char* usage)
{
    if (g_i_cmd_idx < CMD_TABLE_SIZE)
    {
        strncpy(g_t_cmd_table[g_i_cmd_idx].ach_name, name, CMD_NAME_SIZE);
        //memcpy(g_t_cmd_table[g_i_cmd_idx].ach_name,name,CMD_NAME_SIZE);
        strncpy(g_t_cmd_table[g_i_cmd_idx].ach_usage, usage, CMD_USAGE_SIZE);
	  //memcpy(g_t_cmd_table[g_i_cmd_idx].ach_name,name,CMD_USAGE_SIZE);
        g_t_cmd_table[g_i_cmd_idx].t_cmd = (reg_func_t)func;
        g_i_cmd_idx++;
    }
}

bool sys_api_add_threadinfo( sys_thread_info_t* pt_thread_info )
{
    sys_thread_list_t* pt_new_thread = NULL;
    sys_thread_list_t* pt_thread_list = g_t_thread_mgr.pt_used_list;
    bool b_find = false;
    if(pt_thread_info == NULL)
    {   
        return false;
    }
    if(g_t_thread_mgr.b_init)
    {   
        sys_api_sem_take(g_t_thread_mgr.t_sem);

        if (NULL == g_t_thread_mgr.pt_free_list_head)
        {
            sys_api_sem_give(g_t_thread_mgr.t_sem);
            return false;
        }

        pt_new_thread = g_t_thread_mgr.pt_free_list_head;
        g_t_thread_mgr.pt_free_list_head = g_t_thread_mgr.pt_free_list_head->pt_next;
    
        if (NULL == g_t_thread_mgr.pt_free_list_head) 
        {
            g_t_thread_mgr.pt_free_list_tail = NULL;
        }
        strcpy(pt_new_thread->t_thread.ach_thread_name, pt_thread_info->ach_thread_name);
        pt_new_thread->t_thread.dw_thread_id = pt_thread_info->dw_thread_id;
        pt_new_thread->pt_next = NULL;
        
        /* search the pid in used list */
        while(pt_thread_list != NULL)
        {
            if(pt_thread_list->t_thread.dw_thread_id == pt_new_thread->t_thread.dw_thread_id)
            {
                strcpy(pt_thread_list->t_thread.ach_thread_name, pt_new_thread->t_thread.ach_thread_name);
                b_find = true;
                break;
            }
            pt_thread_list = pt_thread_list->pt_next;
        }       

        if(!b_find)
        {
            /* add to used list */
            if (NULL != g_t_thread_mgr.pt_used_list)
            {
                pt_new_thread->pt_next = g_t_thread_mgr.pt_used_list;
            }
    
            g_t_thread_mgr.pt_used_list = pt_new_thread; 
        }
        sys_api_sem_give(g_t_thread_mgr.t_sem);
        return true;
    }
    else
    {
        return false;
    }   
}

bool sys_api_del_threadinfo( u32  dw_thread_id )
{
    sys_thread_list_t* pt_pre;
    sys_thread_list_t* pt_loop;
    sys_thread_list_t* pt_thread_list = NULL;
    if(g_t_thread_mgr.b_init)
    {
        sys_api_sem_take(g_t_thread_mgr.t_sem);

        pt_pre = NULL;
        pt_loop = g_t_thread_mgr.pt_used_list;
        while (NULL != pt_loop)
        {
            if (pt_loop->t_thread.dw_thread_id == dw_thread_id)
            {
                pt_thread_list = pt_loop;
                break;
            }
            pt_pre = pt_loop;
            pt_loop = pt_loop->pt_next;
        }
        if (NULL == pt_pre)
        {
            if (NULL != pt_loop)
            {
                g_t_thread_mgr.pt_used_list = pt_loop->pt_next;
            }
        }
        else if (NULL != pt_loop)
        {
            pt_pre->pt_next = pt_loop->pt_next;
        }
    
        if (NULL == pt_loop)
        {
            /*can't find the thread*/
            sys_api_sem_give(g_t_thread_mgr.t_sem);
            return false;
        }
    
        pt_thread_list->pt_next = NULL;

        /* add to free list */
        if (NULL != g_t_thread_mgr.pt_free_list_tail)
        {
            g_t_thread_mgr.pt_free_list_tail->pt_next = pt_thread_list;
            g_t_thread_mgr.pt_free_list_tail = pt_thread_list;
        }
        else
        {
            g_t_thread_mgr.pt_free_list_head = pt_thread_list;
            g_t_thread_mgr.pt_free_list_tail = pt_thread_list;
        }
        
    
        sys_api_sem_give(g_t_thread_mgr.t_sem);
    }

    return true;
}

void sys_api_print_threadinfo()
{
    sys_thread_list_t* pt_thread_info = g_t_thread_mgr.pt_used_list;
    if(g_t_thread_mgr.b_init)
    {   
        sys_api_sem_take(g_t_thread_mgr.t_sem);  
        sys_api_printf( "\nThe thread table is listed as below:\n");
        sys_api_printf( "\nThread ID         Thread Name\n");
        while(pt_thread_info != NULL)
        {
            sys_api_printf( "%6d            %s\n", pt_thread_info->t_thread.dw_thread_id,pt_thread_info->t_thread.ach_thread_name);
            pt_thread_info = pt_thread_info->pt_next;
        }   
        
        sys_api_sem_give(g_t_thread_mgr.t_sem);
    }   
}
