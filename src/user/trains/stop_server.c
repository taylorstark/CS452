#include "stop_server.h"

#include "display.h"
#include "physics.h"
#include <rtosc/assert.h>
#include <rtosc/string.h>
#include <rtkernel.h>
#include <rtos.h>
#include <user/trains.h>

#define STOP_SERVER_NAME "stop"

typedef enum _STOP_SERVER_REQUEST_TYPE
{
    LocationUpdateRequest = 0,
    DirectionUpdateRequest,
    StopTrainAtLocationRequest
} STOP_SERVER_REQUEST_TYPE;

typedef struct _STOP_SERVER_REQUEST
{
    STOP_SERVER_REQUEST_TYPE type;

    union
    {
        TRAIN_LOCATION trainLocation;
        TRAIN_DIRECTION trainDirection;
    };
} STOP_SERVER_REQUEST;

static
VOID
StopServerpLocationNotifierTask
    (
        VOID
    )
{
    INT stopServerId = MyParentTid();
    ASSERT(SUCCESSFUL(stopServerId));

    STOP_SERVER_REQUEST request;
    request.type = LocationUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(LocationAwait(&request.trainLocation)));
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
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, StopServerpLocationNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, StopServerpDirectionChangeNotifierTask)));

    while(1)
    {
        INT senderId;
        STOP_SERVER_REQUEST request;

        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));

        switch(request.type)
        {
            case LocationUpdateRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                // Use the updated location to see if we need to stop the train yet
                LOCATION* stopLocation = &stopLocations[request.trainLocation.train];

                if(NULL != stopLocation->node)
                {
                    UINT distanceToTarget;
                    if(SUCCESSFUL(TrackDistanceBetween(request.trainLocation.location.node, stopLocation->node, &distanceToTarget)))
                    {
                        DIRECTION direction = directions[request.trainLocation.train];

                        UINT remainingDistance = distanceToTarget - request.trainLocation.location.distancePastNode;
                        UINT stoppingDistance = PhysicsStoppingDistance(request.trainLocation.train, request.trainLocation.velocity, direction);

                        if(remainingDistance < stoppingDistance)
                        {
                            Log("Stopping %d at %s (currently %d away)", request.trainLocation.train, stopLocation->node->name, remainingDistance);
                            VERIFY(SUCCESSFUL(TrainSetSpeed(request.trainLocation.train, 0)));
                            RtMemset(stopLocation, sizeof(*stopLocation), 0);
                        }
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
                stopLocations[request.trainLocation.train] = request.trainLocation.location;
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
    VERIFY(SUCCESSFUL(Create(Priority21, StopServerpTask)));
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
        request.trainLocation.train = train;
        request.trainLocation.location = *location;
        request.trainLocation.velocity = 0;

        result = Send(stopServerId, &request, sizeof(request), NULL, 0);
    }

    return result;
}
