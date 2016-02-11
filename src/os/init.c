#include "init.h"

#include <bwio/bwio.h>
#include <rtosc/assert.h>
#include "clock_server.h"
#include "idle.h"
#include "io.h"
#include "name_server.h"
#include "uart.h"

#define K3_TASKS 14

static
VOID
UserPerformanceTask
    (
        VOID
    )
{
    DelayUntil(300);

    bwprintf(BWCOM2, "\r\n");
    bwprintf(BWCOM2, "--PERFORMANCE--\r\n");
    bwprintf(BWCOM2, "TASK\t%%active\r\n");

    TASK_PERFORMANCE performanceCounters[K3_TASKS];
    UINT i;
    UINT totalTime = 0;
    for (i = 0; i < K3_TASKS; i++)
    {
        QueryPerformance(i, &performanceCounters[i]);
        totalTime += performanceCounters[i].activeTicks;
    }

    for (i = 0; i < K3_TASKS; i++)
    {
        float fraction = (performanceCounters[i].activeTicks / ((float) totalTime)) * 100;
        UINT integerPart = fraction;
        UINT decimalPart = (fraction - integerPart) * 100;

        bwprintf(BWCOM2, "%d\t%u.%u%u\r\n", i, integerPart, decimalPart / 10, decimalPart % 10);
    }

    Shutdown();
}

static
VOID
TestEchoTask
    (
        VOID
    )
{
    IO_DEVICE device;

    VERIFY(SUCCESSFUL(Open(UartDevice, ChannelCom2, &device)));

    while(1)
    {
        // This is just a demonstration of how the i/o server tries to be efficient
        // You can wait for a whole bunch of characters to be ready
        // This should echo in batches of 2, instead of every character
        CHAR buffer[2];

        VERIFY(SUCCESSFUL(Read(&device, buffer, sizeof(buffer))));
        VERIFY(SUCCESSFUL(Write(&device, buffer, sizeof(buffer))));
    }
}

VOID
InitTask
    (
        VOID
    )
{
    IdleCreateTask();
    NameServerCreateTask();
    ClockServerCreateTask();
    IoCreateTask();
    UartCreateTasks();
    Create(LowestUserPriority, UserPerformanceTask);
    Create(HighestUserPriority, TestEchoTask);
}
