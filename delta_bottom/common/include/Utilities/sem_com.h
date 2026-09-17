#ifndef _SEM_COM_H
#define _SEM_COM_H

#include <unistd.h>
#include <sys/sem.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <stdio.h>
#include <stdlib.h>

// System V 信号量封装（互斥锁用）
class sem_com
{
public:
    sem_com(int keyid) : m_semid(-1)
    {
        // 用 IPC_EXCL 判断是否首次创建：首次创建者负责初始化，其余只打开不重置
        m_semid = semget((key_t)keyid, 1, IPC_CREAT | IPC_EXCL | 0666);
        if (m_semid == -1) {
            // 已存在：直接打开，不重新初始化（否则会覆盖他人正在持有的锁）
            m_semid = semget((key_t)keyid, 1, 0666);
        } else {
            // 首次创建：初始化为 1（互斥锁，未上锁）
            union semun sem_arg;
            sem_arg.val = 1;
            if (semctl(m_semid, 0, SETVAL, sem_arg) == -1) {
                printf("sem_com SETVAL error\n");
                exit(EXIT_FAILURE);
            }
        }
        if (m_semid == -1) {
            printf("sem_com create/open failed\n");
            exit(EXIT_FAILURE);
        }
    }

    // 不删除信号量：其他进程可能仍在用；SEM_UNDO 保证进程退出时自动释放持有的锁
    ~sem_com() {}

    // P 操作（加锁）
    void sem_p()
    {
        struct sembuf sem_arg;
        sem_arg.sem_num = 0;
        sem_arg.sem_op = -1;
        sem_arg.sem_flg = SEM_UNDO;
        if (semop(m_semid, &sem_arg, 1) == -1)
            printf("%s: sem_p failed\n", __func__);
    }

    // V 操作（解锁）
    void sem_v()
    {
        struct sembuf sem_arg;
        sem_arg.sem_num = 0;
        sem_arg.sem_op = 1;
        sem_arg.sem_flg = SEM_UNDO;
        if (semop(m_semid, &sem_arg, 1) == -1)
            printf("%s: sem_v failed\n", __func__);
    }

private:
    union semun
    {
        int val;
        struct semid_ds *buf;
        unsigned short *array;
    };
    int m_semid;
};

#endif
