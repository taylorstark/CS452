#include "destination_server.h"

#include "display.h"
#include <rtosc/assert.h>
#include <rtosc/rand.h>
#include <rtosc/string.h>
#include <rtkernel.h>
#include <rtos.h>
#include <user/trains.h>

#define DESTINATION_SERVER_NAME "dest"
#define DESTINATION_SERVER_LOOKING_FOR_TRAIN_SPEED 8

typedef enum _DESTINATION_REQUEST_TYPE
{
    DestinationOnce = 0, 
    DestinationForever, 
    AttributedSensorUpdateRequest
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
    UINT randomNumber = abs(RtRngGenerate(&destinationData->rng)) % 80;

    SENSOR sensor;
    sensor.module = 'A' + (randomNumber / 16);
    sensor.number = (randomNumber % 16) + 1;
    
    LOCATION location;
    location.node = TrackFindSensor(&sensor);
    location.distancePastNode = 0;

    return location;
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
                        // TODO
                        Log("Destination server found %d", request.attributedSensor.train);
                    }
                }

                break;
            }

            case DestinationOnce:
            {
                DESTINATION_DATA* destinationData = &destinations[request.destinationOnce.train];
                destinationData->destination = request.destinationOnce.location;
                destinationData->forever = FALSE;
                Log("%d going to %s", request.train, destinationData->destination.node->name);
                if(destinationData->hasBeenFound)
                {
                    // TODO
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
                Log("%d going forever to %s", request.train, destinationData->destination.node->name);
                if(destinationData->hasBeenFound)
                {
                    // TODO
                }
                else
                {
                    VERIFY(SUCCESSFUL(TrainSetSpeed(request.train, DESTINATION_SERVER_LOOKING_FOR_TRAIN_SPEED)));
                }

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
