#include "train_server.h"

#include <rtosc/assert.h>
#include <rtosc/buffer.h>
#include <rtosc/string.h>
#include <rtkernel.h>
#include <rtos.h>
#include <user/trains.h>

#include "display.h"
#include "location_server.h"

#define TRAIN_SERVER_NAME "train"
#define NUM_TRAINS 80

#define TRAIN_COMMAND_REVERSE 0xF
#define TRAIN_COMMAND_GO 0x60
#define TRAIN_COMMAND_STOP 0x61

typedef enum _TRAIN_REQUEST_TYPE
{
    ShutdownRequest = 0,
    SetSpeedRequest,
    GetSpeedRequest,
    ReverseRequest, 
    ReverseStoppedRequest, 
    AwaitSpeedChangeRequest, 
    AwaitDirectionChangeRequest
} TRAIN_REQUEST_TYPE;

typedef struct _TRAIN_REQUEST
{
    TRAIN_REQUEST_TYPE type;
    UCHAR train;
    UCHAR speed;
} TRAIN_REQUEST;

typedef struct _TRAIN_WORKER_REQUEST
{
    INT delay;
    TRAIN_REQUEST request;
} TRAIN_WORKER_REQUEST;

static
INT
TrainpSendRequest
    (
        IN TRAIN_REQUEST* request
    )
{
    INT result = WhoIs(TRAIN_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT trainServerId = result;

        result = Send(trainServerId, request, sizeof(*request), NULL, 0);
    }

    return result;
}

static
VOID
TrainpShutdownHook
    (
        VOID
    )
{
    TRAIN_REQUEST request = { ShutdownRequest };

    VERIFY(SUCCESSFUL(TrainpSendRequest(&request)));
}

static
INT
TrainpSendOneByteCommand
    (
        IN IO_DEVICE* device,
        IN UCHAR byte
    )
{
    return WriteChar(device, byte);
}

static
INT
TrainpSendTwoByteCommand
    (
        IN IO_DEVICE* device,
        IN UCHAR byte1,
        IN UCHAR byte2
    )
{
    UCHAR buffer[2] = { byte1, byte2 };

    return Write(device, buffer, sizeof(buffer));
}

static
INT
TrainpGo
    (
        IN IO_DEVICE* device
    )
{
    return TrainpSendOneByteCommand(device, TRAIN_COMMAND_GO);
}

static
INT
TrainpStop
    (
        IN IO_DEVICE* device
    )
{
    return TrainpSendOneByteCommand(device, TRAIN_COMMAND_STOP);
}

static
INT
TrainpSetSpeed
    (
        IN IO_DEVICE* device,
        IN UCHAR train,
        IN UCHAR speed
    )
{
    // Bytes must be sent in a weird order. Speed first, then train.
    return TrainpSendTwoByteCommand(device, speed, train);
}

static
INT
TrainpReverse
    (
        IN IO_DEVICE* device,
        IN UCHAR train
    )
{
    return TrainpSendTwoByteCommand(device, TRAIN_COMMAND_REVERSE, train);
}

static
VOID
TrainpWorkerTask
    (
        VOID
    )
{
    // TODO - Something like DelayedSend would be so much better than worker tasks
    while(1)
    {
        INT senderId;
        TRAIN_WORKER_REQUEST request;

        // Wait for a request from the train server
        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));
        VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

        // Delay for a bit
        VERIFY(SUCCESSFUL(Delay(request.delay)));

        // Notify the train server
        VERIFY(SUCCESSFUL(Send(senderId, &request.request, sizeof(request.request), NULL, 0)));
    }
}

static
VOID
TrainpNotifySpeedChange
    (
        RT_CIRCULAR_BUFFER* awaitingSpeedChangeTasks, 
        UCHAR train, 
        UCHAR speed
    )
{
    INT awaitingTask;
    TRAIN_SPEED trainSpeed = { train, speed };

    while(!RtCircularBufferIsEmpty(awaitingSpeedChangeTasks))
    {
        VERIFY(RT_SUCCESS(RtCircularBufferPeekAndPop(awaitingSpeedChangeTasks, &awaitingTask, sizeof(awaitingTask))));
        VERIFY(SUCCESSFUL(Reply(awaitingTask, &trainSpeed, sizeof(trainSpeed))));
    }
}

static
VOID
TrainpNotifyDirectionChange
    (
        RT_CIRCULAR_BUFFER* awaitingDirectionChangeTasks, 
        UCHAR train, 
        DIRECTION direction
    )
{
    INT awaitingTask;
    TRAIN_DIRECTION trainDirection = { train, direction };

    while(!RtCircularBufferIsEmpty(awaitingDirectionChangeTasks))
    {
        VERIFY(RT_SUCCESS(RtCircularBufferPeekAndPop(awaitingDirectionChangeTasks, &awaitingTask, sizeof(awaitingTask))));
        VERIFY(SUCCESSFUL(Reply(awaitingTask, &trainDirection, sizeof(trainDirection))));
    }
}

static
VOID
TrainpTask
    (
        VOID
    )
{
    // Make the server global visible
    VERIFY(SUCCESSFUL(RegisterAs(TRAIN_SERVER_NAME)));

    // On shutdown, stop all trains
    VERIFY(SUCCESSFUL(ShutdownRegisterHook(TrainpShutdownHook)));

    // Open a handle to the train
    IO_DEVICE com1;
    VERIFY(SUCCESSFUL(Open(UartDevice, ChannelCom1, &com1)));

    // Turn the train controller on
    VERIFY(SUCCESSFUL(TrainpGo(&com1)));

    // Stop all known trains, in case any group forgot to turn them off
    VERIFY(SUCCESSFUL(TrainpSetSpeed(&com1, 58, 0)));
    VERIFY(SUCCESSFUL(TrainpSetSpeed(&com1, 63, 0)));
    VERIFY(SUCCESSFUL(TrainpSetSpeed(&com1, 64, 0)));
    VERIFY(SUCCESSFUL(TrainpSetSpeed(&com1, 68, 0)));
    VERIFY(SUCCESSFUL(TrainpSetSpeed(&com1, 69, 0)));

    // Create worker tasks
    INT workerTasks[MAX_TRACKABLE_TRAINS];
    UINT nextWorkerTask = 0;
    for(UINT i = 0; i < MAX_TRACKABLE_TRAINS; i++)
    {
        workerTasks[i] = Create(Priority12, TrainpWorkerTask);
        ASSERT(SUCCESSFUL(workerTasks[i]));
    }

    // Initialize variables
    BOOLEAN running = TRUE;

    UCHAR speeds[NUM_TRAINS];
    RtMemset(speeds, sizeof(speeds), 0);

    DIRECTION directions[NUM_TRAINS];
    RtMemset(directions, sizeof(directions), DirectionForward);

    INT underlyingAwaitingSpeedChangeTasksBuffer[NUM_TASKS];
    RT_CIRCULAR_BUFFER awaitingSpeedChangeTasks;
    RtCircularBufferInit(&awaitingSpeedChangeTasks, underlyingAwaitingSpeedChangeTasksBuffer, sizeof(underlyingAwaitingSpeedChangeTasksBuffer));

    INT underlyingAwaitingDirectionChangeTasksBuffer[NUM_TASKS];
    RT_CIRCULAR_BUFFER awaitingDirectionChangeTasks;
    RtCircularBufferInit(&awaitingDirectionChangeTasks, underlyingAwaitingDirectionChangeTasksBuffer, sizeof(underlyingAwaitingDirectionChangeTasksBuffer));

    while(running)
    {
        INT sender;
        TRAIN_REQUEST request;

        VERIFY(SUCCESSFUL(Receive(&sender, &request, sizeof(request))));

        switch(request.type)
        {
            case ShutdownRequest:
            {
                running = FALSE;
                VERIFY(SUCCESSFUL(Reply(sender, NULL, 0)));
                break;
            }

            case GetSpeedRequest:
            {
                UCHAR speed = speeds[request.train - 1];
                VERIFY(SUCCESSFUL(Reply(sender, &speed, sizeof(speed))));
                break;
            }

            case SetSpeedRequest:
            {
                // Set the train's speed
                if(speeds[request.train - 1] != request.speed)
                {
                    speeds[request.train - 1] = request.speed;
                    VERIFY(SUCCESSFUL(TrainpSetSpeed(&com1, request.train, request.speed)));
                }
                
                VERIFY(SUCCESSFUL(Reply(sender, NULL, 0)));

                // Let tasks know about the train's new speed
                TrainpNotifySpeedChange(&awaitingSpeedChangeTasks, request.train, request.speed);
                break;
            }

            case ReverseRequest:
            {
                UCHAR oldSpeed = speeds[request.train - 1];

                // Stop the train
                VERIFY(SUCCESSFUL(TrainpSetSpeed(&com1, request.train, 0)));
                speeds[request.train - 1] = 0;
                VERIFY(SUCCESSFUL(Reply(sender, NULL, 0)));

                // Schedule a task to reverse the train's direction after it has come to a stop
                TRAIN_WORKER_REQUEST workerRequest;
                workerRequest.delay = 0 == oldSpeed ? 10 : 100 * (oldSpeed / 4 + 1); // TODO - More accurate stopping time
                workerRequest.request.type = ReverseStoppedRequest;
                workerRequest.request.train = request.train;
                workerRequest.request.speed = oldSpeed;

                INT nextWorkerTaskId = workerTasks[nextWorkerTask % MAX_TRACKABLE_TRAINS];
                nextWorkerTask++;

                VERIFY(SUCCESSFUL(Send(nextWorkerTaskId, &workerRequest, sizeof(workerRequest), NULL, 0)));

                // Let tasks know the train is stopping
                TrainpNotifySpeedChange(&awaitingSpeedChangeTasks, request.train, 0);
                break;
            }

            case ReverseStoppedRequest:
            {
                // Reverse the train
                VERIFY(SUCCESSFUL(TrainpReverse(&com1, request.train)));
                VERIFY(SUCCESSFUL(Reply(sender, NULL, 0)));

                // Figure out which direction this train is now travelling
                DIRECTION newDirection;

                if(DirectionForward == directions[request.train - 1])
                {
                    newDirection = DirectionReverse;
                }
                else
                {
                    newDirection = DirectionForward;
                }

                directions[request.train - 1] = newDirection;

                // Schedule a task to speed the train back up
                TRAIN_WORKER_REQUEST workerRequest;
                workerRequest.delay = 5; // TODO - Why do we need this?
                workerRequest.request.type = SetSpeedRequest;
                workerRequest.request.train = request.train;
                workerRequest.request.speed = request.speed;

                INT nextWorkerTaskId = workerTasks[nextWorkerTask % MAX_TRACKABLE_TRAINS];
                nextWorkerTask++;
                
                VERIFY(SUCCESSFUL(Send(nextWorkerTaskId, &workerRequest, sizeof(workerRequest), NULL, 0)));

                // Let tasks know about the new direction
                TrainpNotifyDirectionChange(&awaitingDirectionChangeTasks, request.train, newDirection);
                break;
            }

            case AwaitSpeedChangeRequest:
            {
                VERIFY(RT_SUCCESS(RtCircularBufferPush(&awaitingSpeedChangeTasks, &sender, sizeof(sender))));
                break;
            }

            case AwaitDirectionChangeRequest:
            {
                VERIFY(RT_SUCCESS(RtCircularBufferPush(&awaitingDirectionChangeTasks, &sender, sizeof(sender))));
                break;
            }

            default:
            {
                ASSERT(FALSE);
                break;
            }
        }
    }

    // Stop any trains that are moving
    for(UINT i = 0; i < NUM_TRAINS; i++)
    {
        if(0 != speeds[i])
        {
            VERIFY(SUCCESSFUL(TrainpSetSpeed(&com1, i + 1, 0)));
        }
    }

    // Turn the train controller off
    VERIFY(SUCCESSFUL(TrainpStop(&com1)));
}

INT
TrainGetSpeed
    (
        IN INT train,
        OUT UCHAR* speed
    )
{
    if (!(0 <= train && train < NUM_TRAINS))
    {
        return -1;
    }

    INT result = WhoIs(TRAIN_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT trainServerId = result;
        TRAIN_REQUEST request = { GetSpeedRequest, train };

        result = Send(trainServerId, &request, sizeof(request), speed, sizeof(*speed));
    }

    return result;
}

INT
TrainSetSpeed
    (
        IN INT train,
        IN INT speed
    )
{
    if (!(0 <= train && train < NUM_TRAINS))
    {
        return -1;
    }

    if (!(0 <= speed && speed < 15))
    {
        return -1;
    }

    TRAIN_REQUEST request = { SetSpeedRequest, (UCHAR) train, (UCHAR) speed };
    return TrainpSendRequest(&request);
}

INT
TrainSpeedChangeAwait
    (
        OUT TRAIN_SPEED* trainSpeed
    )
{
    INT result = WhoIs(TRAIN_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT trainServerId = result;

        TRAIN_REQUEST request;
        request.type = AwaitSpeedChangeRequest;

        result = Send(trainServerId, &request, sizeof(request), trainSpeed, sizeof(*trainSpeed));
    }

    return result;
}

INT
TrainReverse
    (
        IN INT train
    )
{
    if (!(0 <= train && train < NUM_TRAINS))
    {
        return -1;
    }

    TRAIN_REQUEST request = { ReverseRequest, (UCHAR) train };
    return TrainpSendRequest(&request);
}

INT
TrainDirectionChangeAwait
    (
        OUT TRAIN_DIRECTION* trainDirection
    )
{
    INT result = WhoIs(TRAIN_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT trainServerId = result;

        TRAIN_REQUEST request;
        request.type = AwaitDirectionChangeRequest;

        result = Send(trainServerId, &request, sizeof(request), trainDirection, sizeof(*trainDirection));
    }

    return result;
}

VOID
TrainServerCreate
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(Create(Priority24, TrainpTask)));
}
