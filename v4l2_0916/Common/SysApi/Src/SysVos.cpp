/*================================================================================
                                                                               
Module Name:  SysVos.cpp

General Description: OS api wrapper module

==================================================================================*/

#include <SysApi/SysApi.h>
#include <SysApi/SysVos.h>
#include <time.h>

static u32 g_dw_sem_cnt = 0;
static u32 g_dw_max_sem_num = 0;
static u64 g_qw_sem_take_total_cycle = 0;
static u64 g_qw_sem_take_max_cycle = 0;

task_handle_t sys_api_task_create(
    void *pvTaskEntry, 
    const char *szName, 
    u8 byPriority, 
    u32 dwStacksize, 
    void *dw_data, 
    task_handle_t *pdwTaskID)
{
    task_handle_t  hTask;
    s32 ret = 0;
    struct sched_param tSchParam;   
    pthread_attr_t tThreadAttr;
    s32 nSchPolicy;
    
    pthread_attr_init(&tThreadAttr);
    pthread_attr_getschedpolicy(&tThreadAttr, &nSchPolicy);
    nSchPolicy = SCHED_OTHER;
    pthread_attr_setschedpolicy(&tThreadAttr, nSchPolicy);
    pthread_attr_getschedparam(&tThreadAttr, &tSchParam);
    byPriority = 255-byPriority;
    if(byPriority < 60)
    {
        byPriority = 60;
    }
    tSchParam.sched_priority = byPriority;
    pthread_attr_setschedparam(&tThreadAttr, &tSchParam);

    pthread_attr_setstacksize(&tThreadAttr, dwStacksize);

    ret = pthread_create(&hTask, &tThreadAttr, (LINUXFUNC)(pvTaskEntry), (void*)dw_data);
    
    if(ret == 0)
    {
        if(pdwTaskID != NULL)
        {
            *pdwTaskID = hTask;
        }
        return hTask;
    }
    return 0;
}

bool sys_api_task_exit(void)
{
    static s32 nRetCode = 0;
    pthread_exit(&nRetCode);
    return true;
}

bool sys_api_task_wait_end(task_handle_t hTask)
{
    void *temp;
    return (0 == pthread_join(hTask, &temp));
}

bool sys_api_task_terminate(task_handle_t hTask)
{
    return (0 == pthread_cancel(hTask));
}

bool sys_api_task_set_priority(task_handle_t hTask, u8 byPriority)
{
    struct sched_param tSchParam;   

    byPriority = 255-byPriority;
    if(byPriority < 60)
    {
        byPriority = 60;
    }
    tSchParam.sched_priority = byPriority;
    pthread_setschedparam(hTask, SCHED_OTHER, &tSchParam);
    return true;
}

s32 sys_api_task_get_priority(task_handle_t hTask, u8* puPri)
{
    struct sched_param tSchParam;
    s32 nPolicy;

    if(puPri == NULL)
    {
        return 0;
    }
    
    pthread_getschedparam(hTask, &nPolicy, &tSchParam);
    return 255 - tSchParam.sched_priority;
}

static bool sys_api_task_set_sched_do(task_handle_t t_task, sys_sched_t sched)
{
    s32 policy;
    struct sched_param scp;
    s32 prority = 0; 
    pthread_t thread = t_task;

    if (pthread_getschedparam(thread, &policy, &scp) != 0)
    {
        printf("sys_api_task_set_sched_do "
                    "pthread_getschedparam error %s\n", strerror(errno));
        return false;
    }

    switch(sched)
    {
        case SYS_SCHED_RR:
            policy = SCHED_RR;
            break;
        case SYS_SCHED_FIFO:
            policy = SCHED_FIFO;
            break;
        case SYS_SCHED_OTHER:
            policy = SCHED_OTHER;
            break;
        default:
            policy = SCHED_OTHER;
            break;
    }

    prority = sched_get_priority_max(policy)/2;
    
    scp.sched_priority = prority;
    if (pthread_setschedparam(thread, policy, &scp) != 0)
    {
        printf("sys_api_task_set_sched_do "
                    "pthread_setschedparam error %s\n", strerror(errno));
        return false;        
    }
    
    return true;    	
}

bool sys_api_task_set_sched(task_handle_t t_task, sys_sched_t sched)
{
	return sys_api_task_set_sched_do(t_task, sched);
}

bool sys_api_task_set_self_sched(sys_sched_t sched)
{
	return sys_api_task_set_sched_do(pthread_self(), sched);
}

task_handle_t sys_api_task_self_id(void)
{
    task_handle_t hTask = pthread_self();
    return hTask;
}

bool sys_api_b_sem_create(sem_handle_t *phSema)
{
    sys_cond_t* ptsys_api_Cond=NULL;
    
    if(phSema == NULL)
    {
        return false;
    }

    ptsys_api_Cond = ( sys_cond_t* )malloc(sizeof(sys_cond_t));
    if (ptsys_api_Cond == NULL)
    {
        return false;
    }
    memset(ptsys_api_Cond,0,sizeof(sys_cond_t));
        
    pthread_cond_init(&ptsys_api_Cond->t_cond, NULL);
    pthread_mutex_init(&ptsys_api_Cond->t_mutex, NULL);
    ptsys_api_Cond->n_cur_cnt = 1;
    ptsys_api_Cond->dw_max_cnt = 1;
    *phSema = (sem_handle_t)ptsys_api_Cond;

    g_dw_sem_cnt++;
    if( g_dw_max_sem_num < g_dw_sem_cnt )
    {
        g_dw_max_sem_num = g_dw_sem_cnt;
    }
    return true;

}

bool sys_api_c_sem_create(sem_handle_t *phSema, u32 dwInitCount, u32 dw_max_cnt)
{
    sys_cond_t* ptsys_api_Cond=NULL;
    if(phSema == NULL)
    {
        return false;
    }
    
    ptsys_api_Cond = ( sys_cond_t* )malloc(sizeof(sys_cond_t));
    if (ptsys_api_Cond == NULL)
    {
        return false;
    }

	memset(ptsys_api_Cond,0,sizeof(sys_cond_t));
    
    pthread_cond_init(&ptsys_api_Cond->t_cond, NULL);
    pthread_mutex_init(&ptsys_api_Cond->t_mutex, NULL);
    ptsys_api_Cond->n_cur_cnt = dwInitCount;
    ptsys_api_Cond->dw_max_cnt = dw_max_cnt;
    *phSema = (sem_handle_t)ptsys_api_Cond;

    g_dw_sem_cnt++;
    if( g_dw_max_sem_num < g_dw_sem_cnt )
    {
        g_dw_max_sem_num = g_dw_sem_cnt;
    }
    return true;
}

bool sys_api_sem_take(sem_handle_t hSema)
{
    if(hSema == NULL)
    {
        return false;
    }

    sys_cond_t* ptsys_api_Cond = (sys_cond_t* )hSema;
    s32 ret = 0;
    u32 dwCycleTimes = 0;
    pthread_mutex_lock( &ptsys_api_Cond->t_mutex );
    while( ptsys_api_Cond->n_cur_cnt <= 0 )
    {
        ret = pthread_cond_wait( &ptsys_api_Cond->t_cond, &ptsys_api_Cond->t_mutex );
        if( ptsys_api_Cond->n_cur_cnt <= 0)
        {
            dwCycleTimes++;
        }
        else
        {
            break;
        }
    }

    g_qw_sem_take_total_cycle += dwCycleTimes;
    if( g_qw_sem_take_max_cycle < dwCycleTimes )
    {
        g_qw_sem_take_max_cycle = dwCycleTimes;
    }
    
    if(ptsys_api_Cond->n_cur_cnt > 0)
    {
        ptsys_api_Cond->n_cur_cnt--;
    }
    
    pthread_mutex_unlock( &ptsys_api_Cond->t_mutex );
    
    return (0 == ret);

}

bool sys_api_sem_take_bytime(sem_handle_t hSema, u32 dwTimeout)
{
    if(hSema == NULL)
    {
        return false;
    }

    sys_cond_t* ptsys_api_Cond = (sys_cond_t* )hSema;
    s32 ret = 0;
    struct timeval now;
    struct timespec timeout;
    u32 dwCycleTimes = 0;

    gettimeofday(&now, NULL);
    timeout.tv_sec = now.tv_sec + dwTimeout/1000;
    int nTime = dwTimeout%1000;
    if(now.tv_usec+nTime*1000>1000000) 
    {
        timeout.tv_sec++;
        timeout.tv_nsec = (now.tv_usec+nTime*1000-1000000)*1000;
    }
    else
    {
        timeout.tv_nsec = (now.tv_usec + nTime*1000) * 1000;
    }
    
    pthread_mutex_lock( &ptsys_api_Cond->t_mutex );
    while( ptsys_api_Cond->n_cur_cnt <= 0 )
    {
        ret = pthread_cond_timedwait( &ptsys_api_Cond->t_cond, &ptsys_api_Cond->t_mutex, &timeout );
        if( ETIMEDOUT == ret )
        {
            break;
        }
        if( ptsys_api_Cond->n_cur_cnt <= 0)
        {
            dwCycleTimes++;
            sys_api_printf("May dead_lock SemTakeByTime repeated [%d] errno[%d]\n", dwCycleTimes, errno);
        }
    }
    g_qw_sem_take_total_cycle += dwCycleTimes;
    if( g_qw_sem_take_max_cycle < dwCycleTimes )
    {
        g_qw_sem_take_max_cycle = dwCycleTimes;
    }
    
    if(ptsys_api_Cond->n_cur_cnt > 0)
    {
        ptsys_api_Cond->n_cur_cnt--;
    }
    
    pthread_mutex_unlock( &ptsys_api_Cond->t_mutex );

    return (0 == ret);

}

bool sys_api_sem_delete(sem_handle_t hSema)
{
    if (NULL == hSema)
    {
        return false;
    }
    
    sys_cond_t* ptsys_api_Cond = (sys_cond_t* )hSema;
    pthread_cond_destroy( &ptsys_api_Cond->t_cond );
    pthread_mutex_destroy( &ptsys_api_Cond->t_mutex );

    free(ptsys_api_Cond);
    g_dw_sem_cnt--;
    return true;
}

bool sys_api_sem_give(sem_handle_t hSema)
{
    if(hSema == NULL)
    {
        return false;
    }

    sys_cond_t* ptsys_api_Cond = (sys_cond_t* )hSema;
    s32 ret = 0;
    pthread_mutex_lock( &ptsys_api_Cond->t_mutex );
    if( ptsys_api_Cond->dw_max_cnt > ptsys_api_Cond->n_cur_cnt )
    {
        if( 0 == pthread_cond_signal( &ptsys_api_Cond->t_cond ) )
        {
            ret = 0;
            ptsys_api_Cond->n_cur_cnt++;
        }
        else
        {
            ret = 1;
        }
    }
    else
    {
        ret = 2;
    }
    pthread_mutex_unlock( &ptsys_api_Cond->t_mutex );

    return (0 == ret);
}

void sys_api_sem_show()
{
    u32 tickHigh = 0;
    u32 tickLow = 0;    
    tickHigh = (u32)(g_qw_sem_take_total_cycle>>32);
    tickLow = (u32)g_qw_sem_take_total_cycle;
    printf( "g_qw_sem_take_total_cycle(%lu - %lu)\n " , tickHigh , tickLow );
    
    tickHigh = (u32)(g_qw_sem_take_max_cycle>>32);
    tickLow = (u32)g_qw_sem_take_max_cycle;
    printf( "g_qw_sem_take_max_cycle(%lu - %lu)\n" , tickHigh , tickLow );
        
    printf( "g_dw_sem_cnt(%lu)\n " , g_dw_sem_cnt );
    printf( "g_dw_max_sem_num(%lu)\n" , g_dw_max_sem_num );
}

void sys_api_task_delay(u32 dwMs)
{
    struct timespec req;
    int ret = -1;
    req.tv_sec = dwMs / 1000;
    req.tv_nsec = (dwMs % 1000) * 1000000;
    ret = nanosleep (&req, NULL);
}

u32 sys_api_tick_get()
{
    clock_t ret = times(NULL);
    //times always valid!
    if (-1==ret)
    {
        ret = 0xffffffff - errno;
    }
    return (u32)ret;
}

u32 sys_api_clk_rate_get()
{

    long ret = sysconf(_SC_CLK_TCK);
    /*assert(ret > 0);*/

    return (u32)ret;
}

bool sys_api_get_cpuinfo( sys_cpu_info_t* pt_cpu_info )
{
    if( NULL == pt_cpu_info )
    {
        return false;
    }

    static u32 dwLastTotalTick = 0;
    static u32 dwLastIdleTick = 0;
    static u8 byLastIdlePercent = 0;

    FILE* ptCpuFile = fopen( "/proc/stat" , "r" );
    if( NULL == ptCpuFile )
    {
        printf( "sys_api_get_cpuinfo fopen /proc/stat failed!\n" );
        return false;
    }

    rewind( ptCpuFile );
    
    char achBuffer[256] = {0};
    bool bFind = false;
    u32 dwCurrentTotalTick = 0;
    u32 dwCurrentUserTick = 0;
    u32 dwCurrentNiceTick = 0;
    u32 dwCurrentSysTick = 0;
    u32 dwCurrentIdleTick = 0;
    u32 dwCurrentIowaitTick = 0;
    u32 dwCurrentIrqTick = 0;
    u32 dwCurrentSoftirqTick = 0;
    
    //dwCurrentTotalTick = sys_api_tick_get();
    while( NULL != fgets(achBuffer , 256 , ptCpuFile))
    {
        if(sscanf(achBuffer , "cpu %lu %lu %lu %lu %lu %lu %lu" ,
            &dwCurrentUserTick ,
            &dwCurrentNiceTick ,
            &dwCurrentSysTick ,
            &dwCurrentIdleTick,
            &dwCurrentIowaitTick,
            &dwCurrentIrqTick,
            &dwCurrentSoftirqTick) > 0 )
        {
            bFind = true;
            break;
        }
    }
    dwCurrentTotalTick = dwCurrentUserTick + dwCurrentNiceTick + dwCurrentSysTick + dwCurrentIdleTick \
                     + dwCurrentIowaitTick + dwCurrentIrqTick + dwCurrentSoftirqTick;
    if( ( 0 == dwLastTotalTick ) ||
        ( false == bFind ) )
    {
        pt_cpu_info->by_idle_per = 0;
        dwLastTotalTick = dwCurrentTotalTick;
        dwLastIdleTick = dwCurrentIdleTick;
        byLastIdlePercent = 0;
    }
    else
    {
        if( dwLastTotalTick == dwCurrentTotalTick )
        {
            pt_cpu_info->by_idle_per = byLastIdlePercent;
        }
        else
        {
            pt_cpu_info->by_idle_per = ( dwCurrentIdleTick - dwLastIdleTick )*100/
                ( dwCurrentTotalTick - dwLastTotalTick );
            byLastIdlePercent = pt_cpu_info->by_idle_per;
        }
        dwLastTotalTick = dwCurrentTotalTick;
        dwLastIdleTick = dwCurrentIdleTick;
    }
    
    fclose( ptCpuFile );

    return bFind;
}

bool sys_api_get_meminfo( sys_mem_info_t* pt_mem_info )
{
    if( NULL == pt_mem_info )
    {
        return false;
    }


    FILE* ptMemFile = fopen( "/proc/meminfo" , "r" );
    if( NULL == ptMemFile )
    {
        return false;
    }

    char achBuffer[256] = {0};
    bool bFind = false;
    u32 dwMemTotal = 0;
    u32 dwMemFree = 0;
    
    rewind( ptMemFile );
    fflush( ptMemFile );
    while( NULL != fgets(achBuffer , 256 , ptMemFile))
    {
        if(sscanf(achBuffer , "MemTotal: %lu kB" , &dwMemTotal) > 0)
        {
            bFind = true;
            break;
        }
    }
    if( false == bFind )
    {
        fclose( ptMemFile );
        return false;
    }

    bFind = false;
    
    rewind( ptMemFile );
    fflush( ptMemFile );
    while( NULL != fgets(achBuffer , 256 , ptMemFile ) )
    {
        if( sscanf(achBuffer , "MemFree: %lu kB" , &dwMemFree ) > 0 )
        {
            bFind = true;
            break;
        }
    }
    if( false == bFind )
    {
        pt_mem_info->dw_phy_size = 0;
        pt_mem_info->dw_alloc_size = 0;
        pt_mem_info->dw_free_size = 0;
    }
    else
    {
        pt_mem_info->dw_phy_size = dwMemTotal;
        pt_mem_info->dw_alloc_size = dwMemTotal - dwMemFree;
        pt_mem_info->dw_free_size = dwMemFree;
    }

    fclose( ptMemFile );

    return bFind;
}

bool sys_api_get_timeinfo( sys_time_info* pt_time_info )
{

    struct tm tmTemp;
    struct timeval ttTemp;
    if( NULL == pt_time_info )
    {
        return false;
    }
    if( 0 != gettimeofday( &ttTemp , NULL ) )
    {
        return false;
    }
    tmTemp = *localtime( &ttTemp.tv_sec );
    pt_time_info->w_year = tmTemp.tm_year+1900;
    pt_time_info->w_month = tmTemp.tm_mon+1;
    pt_time_info->w_day = tmTemp.tm_mday;
    pt_time_info->w_hour = tmTemp.tm_hour;
    pt_time_info->w_minute = tmTemp.tm_min;
    pt_time_info->w_second = tmTemp.tm_sec;
    return true;
}

bool sys_api_set_timeinfo( sys_time_info* pt_time_info )
{
    struct tm tmTemp;
    if( NULL == pt_time_info )
    {
        return false;
    }
    
    memset( &tmTemp , 0 , sizeof(struct tm) );
    tmTemp.tm_year = pt_time_info->w_year-1900;
    tmTemp.tm_mon = pt_time_info->w_month-1;
    tmTemp.tm_mday = pt_time_info->w_day;
    tmTemp.tm_hour = pt_time_info->w_hour;
    tmTemp.tm_min = pt_time_info->w_minute;
    tmTemp.tm_sec = pt_time_info->w_second;
    
    struct timeval tvTemp;
    memset( &tvTemp , 0 , sizeof(struct timeval) );
    tvTemp.tv_sec = mktime( &tmTemp );
    return ( 0 == settimeofday( &tvTemp , NULL ) );
}
