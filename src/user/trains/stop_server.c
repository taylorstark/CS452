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
    StopTrainAtLocationRequest
} STOP_SERVER_REQUEST_TYPE;

typedef struct _STOP_SERVER_REQUEST
{
    STOP_SERVER_REQUEST_TYPE type;
    TRAIN_LOCATION trainLocation;
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
StopServerpTask
    (
        VOID
    )
{
    LOCATION stopLocations[MAX_TRAINS];
    RtMemset(stopLocations, sizeof(stopLocations), 0);

    VERIFY(SUCCESSFUL(RegisterAs(STOP_SERVER_NAME)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, StopServerpLocationNotifierTask)));

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
                    VERIFY(SUCCESSFUL(TrackDistanceBetween(request.trainLocation.location.node, stopLocation->node, &distanceToTarget)));

                    UINT remainingDistance = distanceToTarget - request.trainLocation.location.distancePastNode;
                    UINT stoppingDistance = PhysicsStoppingDistance(request.trainLocation.train, request.trainLocation.velocity);

                    if(remainingDistance < stoppingDistance)
                    {
                        Log("Stopping %d", request.trainLocation.train);
                        VERIFY(SUCCESSFUL(TrainSetSpeed(request.trainLocation.train, 0)));
                        RtMemset(stopLocation, sizeof(*stopLocation), 0);
                    }
                    else
                    {
                        Log("%d is %d from target", request.trainLocation.train, remainingDistance);
                    }

                    // TODO: What if the train is going in reverse? Longer distance between pickup and sensor
                }

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
