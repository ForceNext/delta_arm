#ifndef _SEM_COM_H
#define _SEM_COM_H

#include <unistd.h>
#include <sys/sem.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/shm.h>
//信号量封装
class sem_com
{
public:
    sem_com(int keyid)
    {
        m_semid = 0;

        if (!create_sem(keyid) )
        {
            printf("create sem failed \n");
            exit(EXIT_FAILURE);
        }

        if( !init_sem() )
        {
            printf("init sem failed \n");
            exit(EXIT_FAILURE);
        }
    }
    ~sem_com()
    {
        del_sem();
    }

    void sem_p()
    {
        struct sembuf sem_arg;
        sem_arg.sem_num = 0;
        sem_arg.sem_op = -1;
        sem_arg.sem_flg = SEM_UNDO;
        if( -1 == semop(m_semid, &sem_arg, 1) )
        {
            printf("%s:can not do the sem_p\n",__func__);
        }
    }

    void sem_v()
    {
        struct sembuf sem_arg;
        sem_arg.sem_num = 0;
        sem_arg.sem_op = 1;
        sem_arg.sem_flg = SEM_UNDO;
        if( -1 == semop(m_semid, &sem_arg, 1) )
        {
            printf("%s:can not do the sem_v\n",__func__);
        }
    }

private:

    union semun
    {
        int val;
        struct semid_ds *buf;
        unsigned short *array;
    };

    bool create_sem(int keyid)
    {
        m_semid = semget( (key_t)(keyid), 1, IPC_CREAT | 0666);
        if( -1 == m_semid )
        {
            return false;
        }
        return true;
    }

    bool init_sem()
    {
        union semun sem_arg;
        sem_arg.val = 1;
        if( -1 == semctl(m_semid, 0, SETVAL, sem_arg) )
        {
            printf("sem_com SETVAL error\n");
            return false;
        }
        printf("sem_com arg val: %d\n",sem_arg.val);
        return true;
    }

    void del_sem()
    {
        semctl(m_semid, 0, IPC_RMID);
        printf("sem_com close\n");
    }

    int m_semid;

};


#endif
