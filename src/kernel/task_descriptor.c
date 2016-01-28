#include "task_descriptor.h"

#include "rtos.h"

#define TASK_INITIAL_CPSR   0x10

static UINT g_nextFreeTaskId;
static TASK_DESCRIPTOR g_taskDescriptors[NUM_TASK_DESCRIPTORS];

static
inline
VOID
TaskDescriptorReset
    (
        IN TASK_DESCRIPTOR* td
    )
{
    td->taskId = -1;
    td->parentTaskId = -1;
    td->state = ZombieState;
    td->priority = IdlePriority;
    td->startFunc = NULLPTR;
    td->stackPointer = NULLPTR;
}

VOID
TaskDescriptorInit
    (
        VOID
    )
{
    UINT i;

    g_nextFreeTaskId = 1;
    
    for(i = 0; i < NUM_TASK_DESCRIPTORS; i++)
    {
        TaskDescriptorReset(&g_taskDescriptors[i]);
    }
}

static
inline
VOID
TaskDescriptorUpdateStack
    (
        IN TASK_DESCRIPTOR* td,
        IN STACK* stack,
        IN TASK_START_FUNC startFunc
    )
{
    UINT* stackPointer = ((UINT*) ptr_add(stack->top, stack->size)) - sizeof(UINT);

    *stackPointer = (UINT) Exit;
    *(stackPointer - 10) = (UINT) startFunc;
    *(stackPointer - 11) = TASK_INITIAL_CPSR;
    stackPointer -= 12;

    td->stack = *stack;
    td->stackPointer = stackPointer;
}

static
inline
BOOLEAN
TaskDescriptorIsZombie
    (
        IN TASK_DESCRIPTOR* td
    )
{
    return (td->state == ZombieState);
}

static
RT_STATUS
TaskDescriptorGetZombie
    (
        OUT TASK_DESCRIPTOR** td
    )
{
    UINT i;
    UINT idx = g_nextFreeTaskId % NUM_TASK_DESCRIPTORS;

    for(i = 0; i < NUM_TASK_DESCRIPTORS; i++)
    {
        if (TaskDescriptorIsZombie(&g_taskDescriptors[idx]))
        {
            *td = &g_taskDescriptors[idx];

            return STATUS_SUCCESS;
        }

        idx = (idx + 1) % NUM_TASK_DESCRIPTORS;
    }

    return STATUS_NOT_FOUND;
}

RT_STATUS
TaskDescriptorCreate
    (
        IN INT parentTaskId,
        IN TASK_PRIORITY priority,
        IN TASK_START_FUNC startFunc,
        IN STACK* stack,
        OUT TASK_DESCRIPTOR** td
    )
{
    RT_STATUS status;
    TASK_DESCRIPTOR* zombieTd;

    status = TaskDescriptorGetZombie(&zombieTd);

    if (RT_SUCCESS(status))
    {
        zombieTd->taskId = g_nextFreeTaskId;
        zombieTd->parentTaskId = parentTaskId;
        zombieTd->state = ReadyState;
        zombieTd->priority = priority;
        zombieTd->startFunc = startFunc;

        TaskDescriptorUpdateStack(zombieTd, stack, startFunc);

        g_nextFreeTaskId += 1;

        *td = zombieTd;
    }

    return status;
}

RT_STATUS
TaskDescriptorDestroy
    (
        IN INT taskId
    )
{
    RT_STATUS status;
    TASK_DESCRIPTOR* td;

    status = TaskDescriptorGet(taskId, &td);

    if (RT_SUCCESS(status))
    {
        TaskDescriptorReset(td);
    }

    return status;
}

RT_STATUS
TaskDescriptorGet
    (
        IN INT taskId,
        OUT TASK_DESCRIPTOR** td
    )
{
    UINT i;
    UINT idx = taskId % NUM_TASK_DESCRIPTORS;

    for(i = 0; i < NUM_TASK_DESCRIPTORS; i++)
    {
        if (g_taskDescriptors[idx].taskId == taskId)
        {
            *td = &g_taskDescriptors[idx];

            return STATUS_SUCCESS;
        }
        
        idx = (idx + 1) % NUM_TASK_DESCRIPTORS;
    }

    return STATUS_NOT_FOUND;
}