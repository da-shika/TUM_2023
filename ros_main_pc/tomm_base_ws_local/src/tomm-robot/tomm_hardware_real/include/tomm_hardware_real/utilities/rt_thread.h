#ifndef RT_THREAD_H_
#define RT_THREAD_H_

#include <pthread.h>
#include <sys/mman.h>
#include <stdio.h>
#include <malloc.h>
#include <sys/resource.h>
#include <limits.h>

template<class C, void* (C::* run_mem)()> 
class RTThread
{
public:
    RTThread(void* data, int priority = 49, int stack_size = 8*1024, int stack_pre_fault_size = 8*1024) 
        : data(data),
          priority(priority),
          stack_size(stack_size),
          stack_pre_fault_size(stack_pre_fault_size),
          started(false)
    {
    }

    virtual ~RTThread() 
    {
    }

    bool is_finished() {
        return !started;
    }

    bool is_running() {
        return started;
    }

    void set_priority(int priority) {
        this->priority = priority;
    }

    void set_stack_size(int stack_size) {
        this->stack_size = stack_size;
    }

    void set_stack_pre_fault_size(int stack_pre_fault_size) {
        this->stack_pre_fault_size = stack_pre_fault_size;
    }

    bool start() {
        int ret = 0;

        // allready running
        if( started )
            return false;

        // init attr
        if( pthread_attr_init(&pthread_attr) )
            fprintf(stderr, "RTThread::start error at pthread_attr_init\n");

        // set stack size
        printf("stack_size = %d\n", stack_size);
        if( ret = pthread_attr_setstacksize(&pthread_attr, PTHREAD_STACK_MIN + stack_size) ) {
            fprintf(stderr, "RTThread::start error at pthread_attr_setstacksize: %d\n", ret);
        }

        // set sched, prio
        if( pthread_attr_setschedpolicy(&pthread_attr, SCHED_FIFO))
            fprintf(stderr, "RTThread::start error at pthread_attr_setschedpolicy\n");
        struct sched_param param;
        param.sched_priority = priority;
        if( pthread_attr_setschedparam(&pthread_attr, &param) )
            fprintf(stderr, "RTThread::start error at pthread_attr_setschedparam\n");

        // lock mem
        if (mlockall(MCL_CURRENT | MCL_FUTURE))
            perror("RTThread::start error at mlockall\n");

        // create new thread, call run fnc
        if( pthread_create(&pthread, &pthread_attr, pthread_member_wrapper, data) == 0) {
            started = true;
        }
        return started;   
    }

    void terminate() {
        pthread_cancel(pthread);
    }

private:
    static void* pthread_member_wrapper(void* data) 
    {
        C* obj = static_cast<C*>(data);
        return (obj->*run_mem)();
    }

private:
    int priority;
    int stack_size;
    int stack_pre_fault_size;
    bool started;

    pthread_t pthread;
    pthread_attr_t pthread_attr;

    void* data;
};

#endif