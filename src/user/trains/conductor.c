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
#define CONDUCTOR_DISTANCE_TO_ACTUATE_SWITCH 100000 // 10 cm
#define CONDUCTOR_TIME_TO_ACTUATE_SWITCH 15 // 150 ms

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
                    else
                    {
                        // Is the train moving?
                        if(0 == request.route.trainLocation.velocity)
                        {
                            VERIFY(SUCCESSFUL(TrainSetSpeed(request.route.trainLocation.train, CONDUCTOR_TRAIN_CRUISING_SPEED)));
                        }
                        else if(request.route.path.numNodes > 0)
                        {
                            // How far will the train move by the time we can issue a command?
                            INT distanceBeforeCommand = PhysicsDistanceTravelled(request.route.trainLocation.velocity, 
                                                                                 request.route.trainLocation.acceleration, 
                                                                                 request.route.trainLocation.accelerationTicks, 
                                                                                 CONDUCTOR_TIME_TO_ACTUATE_SWITCH);
                            INT distanceToFrontOfTrain = PhysicsDistanceFromPickupToFrontOfTrain(data->direction);
                            INT distance = request.route.trainLocation.location.distancePastNode + distanceBeforeCommand + distanceToFrontOfTrain;
                            UINT index = 0;

                            // Skip over nodes in the path we have no hope of issuing a command in time for
                            do
                            {
                                PATH_NODE* pathNode = &request.route.path.nodes[index++];
                                distance -= pathNode->node.edge[pathNode->direction].dist * 1000; // convert units
                            } while(index < request.route.path.numNodes && distance > 0)

                            // Are we still on the path?
                            if(index < request.route.path.numNodes && )
                            {
                                PATH_NODE* pathNode = &request.route.path.nodes[index];

                                // Is the next node a branch and are we close enough to the switch that we should switch it
                                if(NODE_BRANCH == pathNode->node->type && abs(distance) < CONDUCTOR_DISTANCE_TO_ACTUATE_SWITCH)
                                {
                                    SWITCH_DIRECTION switchDirection = DIR_STRAIGHT == pathNode->direction ? SwitchStraight : SwitchCurved;
                                    VERIFY(SUCCESSFUL(SwitchSetDirection(pathNode->node->num, switchDirection)));
                                }
                            }
                        }
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
