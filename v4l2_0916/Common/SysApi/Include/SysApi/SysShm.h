
/*================================================================================
                                                                               
Module Name:  SysShm.h

General Description: share memory module header file

==================================================================================*/

#pragma once

union semun{
    s32 val;
    struct semid_ds *buf;
    u16 *array;
    struct seminfo *buf_info;
    void *pad;
};


void *shm_create(const char* ch_path,u32 n_shm_size,s32 *pn_shm_id);
s32 shm_del(s32 n_shm_id,const void *p_addr);
s32 sem_create(const char *ch_path,s32 n_sem_num, s32 n_sem_val);
s32 sem_del(s32 n_sem_id);

