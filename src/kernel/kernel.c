#include "kernel.h"

#include <rtos.h>
#include <rtosc/assert.h>

#include "cache.h"
#include "interrupt.h"
#include "performance.h"
#include "scheduler.h"
#include "syscall.h"
#include "task.h"
#include "trap.h"

extern
VOID
KernelLeave
    (
        IN UINT* stack
    );

static
inline
RT_STATUS
KernelCreateTask
    (
        IN TASK_PRIORITY priority,
        IN TASK_START_FUNC startFunc
    )
{
    TASK_DESCRIPTOR* unused;

    return TaskCreate(NULL,
                      priority,
                      startFunc,
                      &unused);
}

static
inline
VOID
KernelpInit
    (
        VOID
    )
{
    CacheInit();
    InterruptInit();
    PerformanceInit();
    SchedulerInit();
    SyscallInit();
    TaskInit();
    TrapInstallHandler();

    VERIFY(RT_SUCCESS(KernelCreateTask(LowestSystemPriority, InitOsTasks)));
}

VOID
KernelRun
    (
        VOID
    )
{
    // Other group's kernel may have forgotten to disable an interrupt.
    // Disable all interrupts to handle this.
    InterruptDisableAll();

    // Bootstrap the kernel
    KernelpInit();

    // Run the kernel
    while(1)
    {
        TASK_DESCRIPTOR* nextTd;

        RT_STATUS status = SchedulerGetNextTask(&nextTd);

        if(RT_SUCCESS(status))
        {
            ASSERT(TaskValidate(nextTd));

            nextTd->state = RunningState;

            // Return to user mode
            PerformanceEnterTask();
            KernelLeave(nextTd->stackPointer);
            PerformanceExitTask(nextTd->taskId);

            // The task may have transitioned to a new state
            // due to interrupts, Exit(), etc.  Don't update
            // the state unless nothing happened to the task
            if(nextTd->state == RunningState)
            {
                nextTd->state = ReadyState;
            }
        }
        else if(STATUS_NOT_FOUND == status)
        {
            // No more tasks to run, quit the system
            break;
        }
        else
        {
            ASSERT(FALSE);
        }
    }

    // Be a good citizen and disable all interrupts so that we don't
    // mess with any other group's kernel
    InterruptDisableAll();
}
