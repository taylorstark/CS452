#include "destination_server.h"

#include "display.h"
#include <rtosc/assert.h>
#include <rtosc/rand.h>
#include <rtosc/string.h>
#include <rtkernel.h>
#include <rtos.h>
#include <user/trains.h>

#define DESTINATION_SERVER_NAME "dest"
#define DESTINATION_SERVER_LOOKING_FOR_TRAIN_SPEED 10

typedef enum _DESTINATION_REQUEST_TYPE
{
    DestinationOnce = 0, 
    DestinationForever, 
    AttributedSensorUpdateRequest, 
    DestinationReachedRequest
} DESTINATION_REQUEST_TYPE;

typedef struct _DESTINATION_ONCE_REQUEST
{
    UCHAR train;
    LOCATION location;
} DESTINATION_ONCE_REQUEST;

typedef struct _DESTINATION_DATA
{
    BOOLEAN hasBeenFound;
    BOOLEAN forever;
    LOCATION destination;
    RT_RNG rng;
} DESTINATION_DATA;

typedef struct _DESTINATION_REQUEST
{
    DESTINATION_REQUEST_TYPE type;

    union
    {
        UCHAR train;
        DESTINATION_ONCE_REQUEST destinationOnce;
        ATTRIBUTED_SENSOR attributedSensor;
        DESTINATION_REACHED destinationReached;
    };
} DESTINATION_REQUEST;

static
VOID
DestinationServerpAttributedSensorNotifierTask
    (
        VOID
    )
{
    INT destinationServerId = MyParentTid();
    ASSERT(SUCCESSFUL(destinationServerId));

    DESTINATION_REQUEST request;
    request.type = AttributedSensorUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(AttributedSensorAwait(&request.attributedSensor)));
        VERIFY(SUCCESSFUL(Send(destinationServerId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
DestinationServerpDestinationReachedNotifierTask
    (
        VOID
    )
{
    INT destinationServerId = MyParentTid();
    ASSERT(SUCCESSFUL(destinationServerId));

    DESTINATION_REQUEST request;
    request.type = DestinationReachedRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(DestinationReachedAwait(&request.destinationReached)));
        VERIFY(SUCCESSFUL(Send(destinationServerId, &request, sizeof(request), NULL, 0)));
    }
}

static
BOOLEAN
DestinationServerpHasDestination
    (
        IN DESTINATION_DATA* destinationData
    )
{
    return NULL != destinationData->destination.node;
}

static
INT
DestinationServerpInitializeTrainRng
    (
        IN UCHAR train, 
        IN DESTINATION_DATA* destinationData
    )
{
    INT result = Time();

    if(SUCCESSFUL(result))
    {
        RtRngInit(&destinationData->rng, train * result);
    }

    return result;
}

static
LOCATION
DestinationServerpGenerateRandomLocation
    (
        IN DESTINATION_DATA* destinationData
    )
{
    INT randomNumber = RtRngGenerate(&destinationData->rng);
    randomNumber = abs(randomNumber) % 80;
    
    SENSOR sensor;
    sensor.module = 'A' + (randomNumber / 16);
    sensor.number = (randomNumber % 16) + 1;
    
    LOCATION location;
    location.node = TrackFindSensor(&sensor);
    location.distancePastNode = 0;

    // Check to see if the node is reachable
    if(NODE_ENTER == location.node->type)
    {
        return DestinationServerpGenerateRandomLocation(destinationData);
    }
    else
    {
        return location;
    }
}

static
VOID
DestinationServerpRouteToDestination
    (
        IN UCHAR train, 
        IN LOCATION* location
    )
{
    VERIFY(SUCCESSFUL(RouteTrainToDestination(train, location)));
    VERIFY(SUCCESSFUL(StopTrainAtLocation(train, location)));
    Log("Driving train %d to %s", train, location->node->name);
}

static
VOID
DestinationServerpTask
    (
        VOID
    )
{
    DESTINATION_DATA destinations[MAX_TRAINS];
    RtMemset(destinations, sizeof(destinations), 0);

    VERIFY(SUCCESSFUL(RegisterAs(DESTINATION_SERVER_NAME)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, DestinationServerpAttributedSensorNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, DestinationServerpDestinationReachedNotifierTask)));

    while(1)
    {
        INT senderId;
        DESTINATION_REQUEST request;

        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));

        switch(request.type)
        {
            case AttributedSensorUpdateRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                DESTINATION_DATA* destinationData = &destinations[request.attributedSensor.train];

                if(!destinationData->hasBeenFound)
                {
                    destinationData->hasBeenFound = TRUE;

                    if(DestinationServerpHasDestination(destinationData))
                    {
                        DestinationServerpRouteToDestination(request.attributedSensor.train, &destinationData->destination);
                    }
                }

                break;
            }

            case DestinationOnce:
            {
                DESTINATION_DATA* destinationData = &destinations[request.destinationOnce.train];
                destinationData->destination = request.destinationOnce.location;
                destinationData->forever = FALSE;

                if(destinationData->hasBeenFound)
                {
                    DestinationServerpRouteToDestination(request.destinationOnce.train, &destinationData->destination);
                }
                else
                {
                    VERIFY(SUCCESSFUL(TrainSetSpeed(request.destinationOnce.train, DESTINATION_SERVER_LOOKING_FOR_TRAIN_SPEED)));
                }

                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));
                break;
            }

            case DestinationForever:
            {
                DESTINATION_DATA* destinationData = &destinations[request.train];
                VERIFY(SUCCESSFUL(DestinationServerpInitializeTrainRng(request.train, destinationData)));
                destinationData->destination = DestinationServerpGenerateRandomLocation(destinationData);
                destinationData->forever = TRUE;

                if(destinationData->hasBeenFound)
                {
                    DestinationServerpRouteToDestination(request.train, &destinationData->destination);
                }
                else
                {
                    VERIFY(SUCCESSFUL(TrainSetSpeed(request.train, DESTINATION_SERVER_LOOKING_FOR_TRAIN_SPEED)));
                }

                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));
                break;
            }

            case DestinationReachedRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                // Find the train that has reached its destination
                DESTINATION_DATA* destinationData = &destinations[request.destinationReached.train];
                ASSERT(DestinationServerpHasDestination(destinationData) && request.destinationReached.location.node == destinationData->destination.node);

                // Let the user know that the train has arrived
                Log("Train %d arrived at %s", request.destinationReached.train, request.destinationReached.location.node->name);

                // The train no longer has a destination
                destinationData->destination.node = NULL;
                destinationData->destination.distancePastNode = 0;

                // TODO: These servers can't wait on the stop server, so we'll have to let them know
                VERIFY(SUCCESSFUL(SetLocation(request.destinationReached.train, &request.destinationReached.location)));
                VERIFY(SUCCESSFUL(RouteClearDestination(request.destinationReached.train)));

                // Check to see if we should pick a new destination for this train
                if(destinationData->forever)
                {
                    destinationData->destination = DestinationServerpGenerateRandomLocation(destinationData);
                    DestinationServerpRouteToDestination(request.destinationReached.train, &destinationData->destination);
                }

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
DestinationServerCreate
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(Create(Priority16, DestinationServerpTask)));
}

INT
TrainDestinationOnce
    (
        IN UCHAR train, 
        IN LOCATION* location
    )
{
    INT result = WhoIs(DESTINATION_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT destinationServerId = result;
        DESTINATION_REQUEST request;
        request.type = DestinationOnce;
        request.destinationOnce.train = train;
        request.destinationOnce.location = *location;

        result = Send(destinationServerId, &request, sizeof(request), NULL, 0);
    }

    return result;
}

INT
TrainDestinationForever
    (
        IN UCHAR train
    )
{
    INT result = WhoIs(DESTINATION_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT destinationServerId = result;
        DESTINATION_REQUEST request;
        request.type = DestinationForever;
        request.train = train;

        result = Send(destinationServerId, &request, sizeof(request), NULL, 0);
    }

    return result;
}
