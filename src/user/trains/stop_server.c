#include "stop_server.h"

#include "display.h"
#include "physics.h"
#include <rtosc/assert.h>
#include <rtosc/buffer.h>
#include <rtosc/string.h>
#include <rtkernel.h>
#include <rtos.h>
#include <user/trains.h>

#define STOP_SERVER_NAME "stop"
#define STOP_SERVER_REGISTRAR_NAME "stop_registrar"

typedef enum _STOP_SERVER_REQUEST_TYPE
{
    RouteUpdateRequest = 0,
    DirectionUpdateRequest,
    StopTrainAtLocationRequest
} STOP_SERVER_REQUEST_TYPE;

typedef struct _STOP_AT_LOCATION_REQUEST
{
    UCHAR train;
    LOCATION location;
} STOP_AT_LOCATION_REQUEST;

typedef struct _STOP_SERVER_REQUEST
{
    STOP_SERVER_REQUEST_TYPE type;

    union
    {
        ROUTE route;
        TRAIN_DIRECTION trainDirection;
        STOP_AT_LOCATION_REQUEST stopAtLocation;
    };
} STOP_SERVER_REQUEST;

typedef enum _STOP_SERVER_REGISTRAR_REQUEST_TYPE
{
    RegisterRequest = 0, 
    DestinationReachedRequest
} STOP_SERVER_REGISTRAR_REQUEST_TYPE;

typedef struct _STOP_SERVER_REGISTRAR_REQUEST
{
    STOP_SERVER_REGISTRAR_REQUEST_TYPE type;
    DESTINATION_REACHED destinationReached;
} STOP_SERVER_REGISTRAR_REQUEST;

typedef struct _STOP_SERVER_WORKER_REQUEST
{
    UINT delayTime;
    DESTINATION_REACHED destinationReached;
} STOP_SERVER_WORKER_REQUEST;

static
VOID
StopServerpRouteNotifierTask
    (
        VOID
    )
{
    INT stopServerId = MyParentTid();
    ASSERT(SUCCESSFUL(stopServerId));

    STOP_SERVER_REQUEST request;
    request.type = RouteUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(RouteAwait(&request.route)));
        VERIFY(SUCCESSFUL(Send(stopServerId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
StopServerpDirectionChangeNotifierTask
    (
        VOID
    )
{
    INT stopServerId = MyParentTid();
    ASSERT(SUCCESSFUL(stopServerId));

    STOP_SERVER_REQUEST request;
    request.type = DirectionUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(TrainDirectionChangeAwait(&request.trainDirection)));
        VERIFY(SUCCESSFUL(Send(stopServerId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
StopServerpRegistrarTask
    (
        VOID
    )
{
    INT underlyingAwaitingTasksBuffer[NUM_TASKS];
    RT_CIRCULAR_BUFFER awaitingTasks;
    RtCircularBufferInit(&awaitingTasks, underlyingAwaitingTasksBuffer, sizeof(underlyingAwaitingTasksBuffer));

    VERIFY(SUCCESSFUL(RegisterAs(STOP_SERVER_REGISTRAR_NAME)));

    while(1)
    {
        INT senderId;
        STOP_SERVER_REGISTRAR_REQUEST request;

        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));

        switch(request.type)
        {
            case RegisterRequest:
            {
                VERIFY(RT_SUCCESS(RtCircularBufferPush(&awaitingTasks, &senderId, sizeof(senderId))));
                break;
            }

            case DestinationReachedRequest:
            {
                // Send the event to any tasks waiting on the event
                INT awaitingTask;
                while(!RtCircularBufferIsEmpty(&awaitingTasks))
                {
                    VERIFY(RT_SUCCESS(RtCircularBufferPeekAndPop(&awaitingTasks, &awaitingTask, sizeof(awaitingTask))));
                    VERIFY(SUCCESSFUL(Reply(awaitingTask, &request.destinationReached, sizeof(request.destinationReached))));
                }

                // Reply to the worker task
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));
                break;
            }

            default:
            {
                ASSERT(FALSE);
                break;
            }
        }
    }
}

static
VOID
StopServerpWorkerTask
    (
        VOID
    )
{
    INT stopServerRegistrarId = WhoIs(STOP_SERVER_REGISTRAR_NAME);
    ASSERT(SUCCESSFUL(stopServerRegistrarId));

    STOP_SERVER_REGISTRAR_REQUEST registrarRequest;
    registrarRequest.type = DestinationReachedRequest;

    while(1)
    {
        INT senderId;
        STOP_SERVER_WORKER_REQUEST request;

        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));
        VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

        registrarRequest.destinationReached = request.destinationReached;

        VERIFY(SUCCESSFUL(Delay(request.delayTime)));
        VERIFY(SUCCESSFUL(Send(stopServerRegistrarId, &registrarRequest, sizeof(registrarRequest), NULL, 0)));
    }
}

static
VOID
StopServerpTask
    (
        VOID
    )
{
    LOCATION stopLocations[MAX_TRAINS];
    RtMemset(stopLocations, sizeof(stopLocations), 0);

    DIRECTION directions[MAX_TRAINS];
    RtMemset(directions, sizeof(directions), DirectionForward);

    VERIFY(SUCCESSFUL(RegisterAs(STOP_SERVER_NAME)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, StopServerpRouteNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, StopServerpDirectionChangeNotifierTask)));
    VERIFY(SUCCESSFUL(Create(Priority13, StopServerpRegistrarTask)));

    INT workerTasks[MAX_TRACKABLE_TRAINS];
    UINT nextWorkerTask = 0;

    for(UINT i = 0; i < MAX_TRACKABLE_TRAINS; i++)
    {
        workerTasks[i] = Create(Priority12, StopServerpWorkerTask);
        ASSERT(SUCCESSFUL(workerTasks[i]));
    }

    while(1)
    {
        INT senderId;
        STOP_SERVER_REQUEST request;

        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));

        switch(request.type)
        {
            case RouteUpdateRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                // Use the updated location to see if we need to stop the train yet
                LOCATION* stopLocation = &stopLocations[request.route.trainLocation.train];

                if(NULL != stopLocation->node && request.route.path.numNodes > 0)
                {
                    // Calculate how long it will take the train to stop
                    DIRECTION direction = directions[request.route.trainLocation.train];
                    UINT endingVelocity = PhysicsEndingVelocity(request.route.trainLocation.velocity, 
                                                                request.route.trainLocation.acceleration,
                                                                min(request.route.trainLocation.accelerationTicks, AVERAGE_TRAIN_COMMAND_LATENCY));
                    UINT stoppingDistance = PhysicsStoppingDistance(request.route.trainLocation.train, endingVelocity, direction);

                    // Calculate the distance between the train and the desired stop location
                    UINT distanceTravelledBeforeCommandExecuted = PhysicsDistanceTravelled(request.route.trainLocation.velocity, 
                                                                                           request.route.trainLocation.acceleration, 
                                                                                           request.route.trainLocation.accelerationTicks, 
                                                                                           AVERAGE_TRAIN_COMMAND_LATENCY);
                    UINT remainingDistance = request.route.path.totalDistance - request.route.trainLocation.location.distancePastNode - distanceTravelledBeforeCommandExecuted;

                    // Check for underflow and check if we should stop
                    if(remainingDistance < request.route.path.totalDistance && remainingDistance < stoppingDistance)
                    {
                        // Stop the train
                        VERIFY(SUCCESSFUL(TrainSetSpeed(request.route.trainLocation.train, 0)));

                        // Once the train has stopped, let other tasks know that the train has reached its destination
                        STOP_SERVER_WORKER_REQUEST workerRequest;
                        workerRequest.delayTime = PhysicsStoppingTime(request.route.trainLocation.train, endingVelocity);
                        workerRequest.destinationReached.train = request.route.trainLocation.train;
                        workerRequest.destinationReached.location = *stopLocation;

                        VERIFY(SUCCESSFUL(Send(workerTasks[nextWorkerTask], &workerRequest, sizeof(workerRequest), NULL, 0)));
                        nextWorkerTask = (nextWorkerTask + 1) % MAX_TRACKABLE_TRAINS;

                        // The train no longer has stop location
                        stopLocation->node = NULL;
                        stopLocation->distancePastNode = 0;
                    }
                }

                break;
            }
            
            case DirectionUpdateRequest:
            {
                directions[request.trainDirection.train] = request.trainDirection.direction;
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));
                break;
            }

            case StopTrainAtLocationRequest:
            {
                stopLocations[request.stopAtLocation.train] = request.stopAtLocation.location;
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));
                break;
            }

            default:
            {
                ASSERT(FALSE);
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));
                break;
            }
        }
    }
}

VOID
StopServerCreate
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(Create(Priority19, StopServerpTask)));
}

INT
StopTrainAtLocation
    (
        IN UCHAR train,
        IN LOCATION* location
    )
{
    INT result = WhoIs(STOP_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT stopServerId = result;

        STOP_SERVER_REQUEST request;
        request.type = StopTrainAtLocationRequest;
        request.stopAtLocation.train = train;
        request.stopAtLocation.location = *location;

        result = Send(stopServerId, &request, sizeof(request), NULL, 0);
    }

    return result;
}

INT
DestinationReachedAwait
    (
        OUT DESTINATION_REACHED* destinationReached
    )
{
    INT result = WhoIs(STOP_SERVER_REGISTRAR_NAME);

    if(SUCCESSFUL(result))
    {
        INT stopServerRegistrarId = result;

        STOP_SERVER_REGISTRAR_REQUEST request;
        request.type = RegisterRequest;

        result = Send(stopServerRegistrarId, &request, sizeof(request), destinationReached, sizeof(*destinationReached));
    }

    return result;
}
