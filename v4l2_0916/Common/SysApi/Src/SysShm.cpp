
/*================================================================================
                                                                               
Module Name:  SysShm.cpp

General Description: share memory module

==================================================================================*/



#include <SysApi/SysApi.h>
#include <SysApi/SysShm.h>

#define SYS_V_SEM_MOD 0644

/*share memory shm API*/
/*open share memory */
void *sys_api_shm_open(shm_sem_parm_t *pt_shm_sem,s32 *pn_shm_id,s32 *pn_sem_id)
{
    s32 n_shm_id = -1;
    s32 n_sem_id = -1;
    void *p_addr=NULL;
    s32 ret;
    struct stat buf;

	/*check if the path is already exit, if not exit ,create it*/
     ret=stat(pt_shm_sem->ch_path,&buf);
	if(ret<0)
	{
		printf("file not exit %s\n",pt_shm_sem->ch_path);
		ret=open(pt_shm_sem->ch_path ,(O_CREAT|O_WRONLY|O_TRUNC), 0644);
		if(ret<0)
		{
			sys_api_printf("create the path faild\n");
			return NULL;
		}
		close(ret);
	}
  
    p_addr=shm_create(pt_shm_sem->ch_path,pt_shm_sem->n_shm_size,&n_shm_id);
    if(p_addr == NULL)
    {
		sys_api_printf("Create shm faild\n");
		return NULL;
    }

    n_sem_id = sem_create(pt_shm_sem->ch_path,pt_shm_sem->n_sem_num,pt_shm_sem->n_sem_val);
    if(n_sem_id == -1)
    {
		sys_api_printf("Create sem faild\n");
		shm_del(n_shm_id,p_addr);
		return NULL;
    }

	*pn_shm_id = n_shm_id;
	*pn_sem_id = n_sem_id;
    return p_addr;
}


s32 sys_api_shm_close(s32 n_shm_id,s32 n_sem_id,const void *p_addr)
{
	s32 ret =-1;
	ret=sem_del(n_shm_id);
	printf("sem del return %d\n",ret);
	/*for temp I don't check the ret*/
  	ret=shm_del(n_sem_id,p_addr);
	printf("shm del return %d\n",ret);
	return ret;
}

/*wait the sem for the  share memory */
bool sys_api_shm_wait(s32 n_sem_id,s32 n_idx,u32 dw_wait_ms)
{
	/* u32 i=0;
	u32 dw_us_tmp; */
	while(semctl(n_sem_id, n_idx, GETVAL, 0) == 0)
    {
    		/* printf("can't get the sem cnt %lu\n",i); */
		/* dw_us_tmp=dw_wait_ms*1000;	*/
		/* i++; */
		usleep(dw_wait_ms*1000);
    } 

    return true;   
}

/*p operate the sem for the  share memory */
bool sys_api_shm_get(s32 n_sem_id,s32 n_idx)
{
	struct sembuf sbuf;
	if(n_idx < 0)
	{
		perror("index of array cannot equals a minus value!\n");
		return false;
	}

	memset(&sbuf,0,sizeof(sbuf));	
	sbuf.sem_num = n_idx;
	sbuf.sem_op = -1;		
	//sbuf.sem_flg &= ~IPC_NOWAIT;
	sbuf.sem_flg = 0;		/* block */

	if (-1 == semop(n_sem_id, &sbuf, 1))
	{
		perror("A wrong operation to semaphore occurred!\n");
		return false;
	}

	return true;
}

/*v operate the sem for the  share memory */
bool sys_api_shm_put(s32 n_sem_id,s32 n_idx)
{
	struct sembuf sbuf;
	/*  struct sembuf sbuf = {0, 1, SEM_UNDO}; */
	if(n_idx < 0)
	{
		perror("index of array cannot equals a minus value!\n");
		return false;
	}

	memset(&sbuf,0,sizeof(sbuf));	
	sbuf.sem_num = n_idx;
	sbuf.sem_op = 1;	
	//sbuf.sem_flg &= ~IPC_NOWAIT;
	sbuf.sem_flg = 0; 
	
	if(semop(n_sem_id, &sbuf, 1) == -1)
	{
		perror("A wrong operation to semaphore occurred!\n");
		return false;
	}
	return true;
}


void *shm_create(const char* ch_path,u32 n_shm_size,s32 *pn_shm_id)
{
    key_t shmkey;
    s32 n_shm_id = -1;
    void *p_addr=NULL;

	/*for temp we use the ftok to get the key,
	because it can reduce the message to transfer the key between different process*/
	#if 1
    if((shmkey = ftok(ch_path, 0)) == -1)
    {
        perror("ftok error!\n");
        return NULL;
    }
	#else
	shmkey = IPC_PRIVATE;
	#endif
    if((n_shm_id = shmget(shmkey, n_shm_size, IPC_CREAT|0666)) == -1)
    {
		printf("memsize %u key %d\n",n_shm_size,shmkey);
        perror("shm call failed!\n");
        return NULL;
    }

	if((p_addr = shmat(n_shm_id, (s8*)0, 0)) == (s8*)-1)
    {
        perror("attch shared memory error!\n");
        return NULL;
    }
	 *pn_shm_id=n_shm_id;
	 return p_addr;
}

s32 shm_del(s32 n_shm_id,const void *p_addr)
{
    s32 ret;
    if(p_addr==NULL)
	{
		return -1;
	}

    ret = shmdt(p_addr);
	//   if(ret == -1)
	//	return ret;
    return (shmctl(n_shm_id, IPC_RMID, NULL));
}

s32 sem_create(const char *ch_path,s32 n_sem_num, s32 n_sem_val)
{
    key_t semkey;
    s32 n_idx, n_sid;
    union semun semopts;
	/*for temp we use the ftok to get the key,
	because it can reduce the message to transfer the key between different process*/
    #if 1
    if((semkey = ftok(ch_path, 0)) == -1)
    {
        perror("ftok error!\n");
        return -1;
    }
	#else
	semkey = IPC_PRIVATE;
	#endif

	if((n_sid = semget(semkey,n_sem_num,IPC_CREAT| IPC_EXCL|SYS_V_SEM_MOD))>=0)
	{
		semopts.val = n_sem_val;
		for(n_idx = 0; n_idx < n_sem_num; n_idx++)
		{
			semctl(n_sid, n_idx, SETVAL, semopts);
		}
	}
	else if (errno == EEXIST)
	{
		n_sid = semget(semkey,n_sem_num,SYS_V_SEM_MOD);
		if(n_sid < 0)
		{
			perror("semget call failed.--1\n");
       	 	return -1;
		}
	}
	else
	{
		 perror("semget call failed.---2\n");
       	 return -1;
	}

    return n_sid;
}

s32 sem_del(s32 n_sem_id)
{
	return (semctl(n_sem_id, 0, IPC_RMID));
}
