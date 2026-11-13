/**
 ******************************************************************************
 * @file    syscalls.c
 * @brief   newlib 系统调用桩
 *
 * newlib-nano 在链接时可能引用 _sbrk / _write / _read 等符号。本工程不使用
 * printf/malloc（FreeRTOS 用 heap_4 的静态数组），这些桩函数基本不会被调到，
 * 但保留它们可以避免"未定义符号"的链接错误，也方便以后临时打日志。
 ******************************************************************************
 */
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <stdint.h>
#include <stddef.h>   /* ptrdiff_t */
#undef errno
extern int errno;

/* 由链接脚本提供：堆区起点与结束 */
extern uint8_t _end;      /* ._user_heap_stack 段起始 */
extern uint8_t _estack;   /* 栈顶，也作为堆的上限 */

/**
 * @brief 堆增长
 * @note  上行堆（从 _end 往 _estack 长），与主栈共用空间，因此主栈要留够。
 *        caddr_t 在部分 newlib 版本里未定义，这里直接用 char*。
 */
void *_sbrk(ptrdiff_t incr)
{
    static uint8_t *heap_end = 0;
    uint8_t *prev_heap_end;

    if (heap_end == 0)
        heap_end = &_end;

    prev_heap_end = heap_end;

    if (heap_end + incr > &_estack)
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;
    return (void *)prev_heap_end;
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    return len;
}

int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

int _close(int file)
{
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

off_t _lseek(int file, off_t ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

int _getpid(void)
{
    return 1;
}
