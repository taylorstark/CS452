#include "conductor.h"

#include "display.h"
#include "physics.h"
#include <rtosc/assert.h>
#include <rtosc/buffer.h>
#include <rtosc/string.h>
#include <rtkernel.h>
#include <rtos.h>
#include <user/trains.h>

#define CONDUCTOR_TRAIN_CRUISING_SPEED 10
#define CONDUCTOR_MINIMUM_VELOCITY_TO_ACTUATE_SWITCH 500
#define CONDUCTOR_DISTANCE_TO_ACTUATE_SWITCH 100000 // 10 cm
#define CONDUCTOR_TIME_TO_ACTUATE_SWITCH 20 // 200 ms

typedef enum _CONDUCTOR_REQUEST_TYPE
{
    RouteUpdateRequest = 0, 
    SpeedUpdateRequest, 
    DirectionUpdateRequest
} CONDUCTOR_REQUEST_TYPE;

typedef struct _CONDUCTOR_REQUEST
{
    CONDUCTOR_REQUEST_TYPE type;

    union
    {
        ROUTE route;
        TRAIN_SPEED trainSpeed;
        TRAIN_DIRECTION trainDirection;
    };
} CONDUCTOR_REQUEST;

typedef struct _CONDUCTOR_DATA
{
    UCHAR reverseCount;
    UCHAR speed;
    DIRECTION direction;
} CONDUCTOR_DATA;

static
VOID
ConductorpRouteNotifierTask
    (
        VOID
    )
{
    INT conductorId = MyParentTid();
    ASSERT(SUCCESSFUL(conductorId));

    CONDUCTOR_REQUEST request;
    request.type = RouteUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(RouteAwait(&request.route)));
        VERIFY(SUCCESSFUL(Send(conductorId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
ConductorpSpeedChangeNotifierTask
    (
        VOID
    )
{
    INT conductorId = MyParentTid();
    ASSERT(SUCCESSFUL(conductorId));

    CONDUCTOR_REQUEST request;
    request.type = SpeedUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(TrainSpeedChangeAwait(&request.trainSpeed)));
        VERIFY(SUCCESSFUL(Send(conductorId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
ConductorpDirectionChangeNotifierTask
    (
        VOID
    )
{
    INT conductorId = MyParentTid();
    ASSERT(SUCCESSFUL(conductorId));

    CONDUCTOR_REQUEST request;
    request.type = DirectionUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(TrainDirectionChangeAwait(&request.trainDirection)));
        VERIFY(SUCCESSFUL(Send(conductorId, &request, sizeof(request), NULL, 0)));
    }
}

static
inline
BOOLEAN
ConductorpIsReversing
    (
        IN CONDUCTOR_DATA* data
    )
{
    return data->reverseCount > 0;
}

static
VOID
ConductorpTask
    (
        VOID
    )
{
    CONDUCTOR_DATA trainData[MAX_TRAINS];
    RtMemset(trainData, sizeof(trainData), 0);

    VERIFY(SUCCESSFUL(Create(HighestUserPriority, ConductorpRouteNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, ConductorpSpeedChangeNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, ConductorpDirectionChangeNotifierTask)));

    while(1)
    {
        INT senderId;
        CONDUCTOR_REQUEST request;

        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));

        switch(request.type)
        {
            case RouteUpdateRequest:
            {
                // Reply to the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                CONDUCTOR_DATA* data = &trainData[request.route.trainLocation.train];

                // Don't do anything if the train is already performing a manoeuvre
                if(!ConductorpIsReversing(data))
                {
                    // Check to see if this route performs a reverse
                    if(request.route.path.performsReverse)
                    {
                        data->reverseCount = 2;
                        VERIFY(SUCCESSFUL(TrainReverse(request.route.trainLocation.train)));
                    }
                    else if(request.route.path.numNodes > 0) // Forward route
                    {
                        // Is the train moving?
                        if(0 == request.route.trainLocation.velocity && 0 == request.route.trainLocation.accelerationTicks)
                        {
                            VERIFY(SUCCESSFUL(TrainSetSpeed(request.route.trainLocation.train, CONDUCTOR_TRAIN_CRUISING_SPEED)));
                        }
                        else if(request.route.trainLocation.velocity > CONDUCTOR_MINIMUM_VELOCITY_TO_ACTUATE_SWITCH)
                        {
                            // How far will the train move by the time we can issue a command?
                            UINT distanceBeforeCommand = PhysicsDistanceTravelled(request.route.trainLocation.velocity, 
                                                                                  request.route.trainLocation.acceleration, 
                                                                                  request.route.trainLocation.accelerationTicks, 
                                                                                  CONDUCTOR_TIME_TO_ACTUATE_SWITCH);
                            UINT distanceToFrontOfTrain = PhysicsDistanceFromPickupToFrontOfTrain(data->direction);
                            UINT distanceLowerBound = request.route.trainLocation.location.distancePastNode + distanceBeforeCommand + distanceToFrontOfTrain;
                            UINT distanceUpperBound = distanceLowerBound + CONDUCTOR_DISTANCE_TO_ACTUATE_SWITCH;
                            UINT distanceTravelled = 0;
                            UINT index = 0;

                            while(index < request.route.path.numNodes && distanceTravelled < distanceUpperBound)
                            {
                                PATH_NODE* pathNode = &request.route.path.nodes[index++];

                                if(distanceTravelled > distanceLowerBound && NODE_BRANCH == pathNode->node->type)
                                {
                                    SWITCH_DIRECTION switchDirection = DIR_STRAIGHT == pathNode->direction ? SwitchStraight : SwitchCurved;
                                    VERIFY(SUCCESSFUL(SwitchSetDirection(pathNode->node->num, switchDirection)));
                                }

                                distanceTravelled += pathNode->node->edge[pathNode->direction].dist * 1000; // convert units
                            }
                        }
                    }
                    else if(0 != data->speed) // No route and the train is moving -> stop it for safety
                    {
                        VERIFY(SUCCESSFUL(TrainSetSpeed(request.route.trainLocation.train, 0)));
                        Log("Stopping %d as it has no route", request.route.trainLocation.train);
                    }
                }

                break;
            }

            case SpeedUpdateRequest:
            {
                // Reply to the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                // Update the train's speed
                CONDUCTOR_DATA* data = &trainData[request.trainSpeed.train];
                data->speed = request.trainSpeed.speed;

                // If the train is reversing, we will receive 2 speed update requests
                if(ConductorpIsReversing(data))
                {
                    data->reverseCount--;
                }

                break;
            }

            case DirectionUpdateRequest:
            {
                trainData[request.trainDirection.train].direction = request.trainDirection.direction;
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

VOID
ConductorCreateTask
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(Create(Priority17, ConductorpTask)));
}
