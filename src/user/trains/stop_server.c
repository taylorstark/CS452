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

                if(NULL != stopLocation->node)
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
                        VERIFY(SUCCESSFUL(TrainSetSpeed(request.route.trainLocation.train, 0)));

                        Log("Stopping %d at %s (Requires %d to stop and is %d away)", 
                            request.route.trainLocation.train, 
                            stopLocation->node->name, 
                            stoppingDistance, 
                            remainingDistance);

                        RtMemset(stopLocation, sizeof(*stopLocation), 0);
                    }
                }

                break;

                /*
                // Use the updated location to see if we need to stop the train yet
                LOCATION* stopLocation = &stopLocations[request.trainLocation.train];

                if(NULL != stopLocation->node)
                {
                    UINT distanceToTarget;
                    if(SUCCESSFUL(TrackDistanceBetween(request.trainLocation.location.node, stopLocation->node, &distanceToTarget)))
                    {
                        DIRECTION direction = directions[request.trainLocation.train];
                        UINT endingVelocity = PhysicsEndingVelocity(request.trainLocation.velocity, 
                                                                    request.trainLocation.acceleration,
                                                                    min(request.trainLocation.accelerationTicks, AVERAGE_TRAIN_COMMAND_LATENCY));
                        UINT stoppingDistance = PhysicsStoppingDistance(request.trainLocation.train, endingVelocity, direction);

                        UINT distanceTravelledBeforeCommandExecuted = PhysicsDistanceTravelled(request.trainLocation.velocity, 
                                                                                               request.trainLocation.acceleration, 
                                                                                               request.trainLocation.accelerationTicks, 
                                                                                               AVERAGE_TRAIN_COMMAND_LATENCY);
                        UINT remainingDistance = distanceToTarget - request.trainLocation.location.distancePastNode - distanceTravelledBeforeCommandExecuted;

                        // Check for underflow and check if we should stop
                        if(remainingDistance < distanceToTarget && remainingDistance < stoppingDistance)
                        {
                            Log("Stopping %d at %s (Requires %d to stop and is %d away)", request.trainLocation.train, stopLocation->node->name, stoppingDistance, remainingDistance);
                            VERIFY(SUCCESSFUL(TrainSetSpeed(request.trainLocation.train, 0)));
                            RtMemset(stopLocation, sizeof(*stopLocation), 0);
                        }
                    }
                }
                */
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
        request.stopAtLocation.train = train;
        request.stopAtLocation.location = *location;

        result = Send(stopServerId, &request, sizeof(request), NULL, 0);
    }

    return result;
}
