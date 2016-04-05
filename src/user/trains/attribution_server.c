#include "attribution_server.h"

#include "display.h"
#include <rtosc/assert.h>
#include <rtosc/buffer.h>
#include <rtosc/string.h>
#include <rtkernel.h>
#include <rtos.h>
#include <user/trains.h>

#define ATTRIBUTION_SERVER_NAME "attribution"

typedef enum _ATTRIBUTION_SERVER_REQUEST_TYPE
{
    SensorChangedRequest = 0,
    SpeedChangedRequest,
    DirectionChangedRequest,
    SwitchChangedRequest,
    AttributedSensorAwaitRequest,
    GetTrackedTrainsRequest,
    NextExpectedNodeRequest
} ATTRIBUTION_SERVER_REQUEST_TYPE;

typedef struct _ATTRIBUTION_SERVER_REQUEST
{
    ATTRIBUTION_SERVER_REQUEST_TYPE type;

    union
    {
        UCHAR train;
        SENSOR sensor;
        TRAIN_SPEED trainSpeed;
        TRAIN_DIRECTION trainDirection;
        INT sw;
    };
} ATTRIBUTION_SERVER_REQUEST;

typedef struct _ATTRIBUTION_DATA
{
    UCHAR train;
    TRACK_NODE* currentNode;
    TRACK_NODE* nextNode;
} ATTRIBUTION_DATA;

static
VOID
AttributionServerpSensorNotifierTask
    (
        VOID
    )
{
    INT attributionServerId = MyParentTid();
    ASSERT(SUCCESSFUL(attributionServerId));

    ATTRIBUTION_SERVER_REQUEST request;
    request.type = SensorChangedRequest;

    while(1)
    {
        SENSOR_DATA sensorData;
        VERIFY(SUCCESSFUL(SensorAwait(&sensorData)));

        if(sensorData.isOn)
        {
            request.sensor = sensorData.sensor;

            VERIFY(SUCCESSFUL(Send(attributionServerId, &request, sizeof(request), NULL, 0)));
        }
    }
}

static
VOID
AttributionServerpSpeedNotifierTask
    (
        VOID
    )
{
    INT attributionServerId = MyParentTid();
    ASSERT(SUCCESSFUL(attributionServerId));

    ATTRIBUTION_SERVER_REQUEST request;
    request.type = SpeedChangedRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(TrainSpeedChangeAwait(&request.trainSpeed)));
        VERIFY(SUCCESSFUL(Send(attributionServerId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
AttributionServerpDirectionNotifierTask
    (
        VOID
    )
{
    INT attributionServerId = MyParentTid();
    ASSERT(SUCCESSFUL(attributionServerId));

    ATTRIBUTION_SERVER_REQUEST request;
    request.type = DirectionChangedRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(TrainDirectionChangeAwait(&request.trainDirection)));
        VERIFY(SUCCESSFUL(Send(attributionServerId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
AttributionServerpSwitchNotifierTask
    (
        VOID
    )
{
    INT attributionServerId = MyParentTid();
    ASSERT(SUCCESSFUL(attributionServerId));

    ATTRIBUTION_SERVER_REQUEST request;
    request.type = SwitchChangedRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(SwitchChangeAwait(&request.sw)));
        VERIFY(SUCCESSFUL(Send(attributionServerId, &request, sizeof(request), NULL, 0)));
    }
}

static
ATTRIBUTION_DATA*
AttributionServerpFindTrainByNextSensor
    (
        IN ATTRIBUTION_DATA* attributionData,
        IN UINT numTrains,
        IN TRACK_NODE* node
    )
{
    // Check to see if we can find the node
    for(UINT i = 0; i < numTrains; i++)
    {
        if(node == attributionData[i].nextNode)
        {
            return &attributionData[i];
        }
    }

    // There are unreliable sensors on the track - check for off by one sensors
    for(UINT i = 0; i < numTrains; i++)
    {
        TRACK_NODE* offByOneNode;
        VERIFY(SUCCESSFUL(TrackFindNextSensor(attributionData[i].nextNode, &offByOneNode)));

        if(node == offByOneNode)
        {
            return &attributionData[i];
        }
    }

    return NULL;
}

static
ATTRIBUTION_DATA*
AttributionServerpFindTrainById
    (
        IN ATTRIBUTION_DATA* attributionData,
        IN UINT numTrains,
        IN UCHAR train
    )
{
    for(UINT i = 0; i < numTrains; i++)
    {
        if(train == attributionData[i].train)
        {
            return &attributionData[i];
        }
    }

    return NULL;
}

static
VOID
AttributionServerpTask
    (
        VOID
    )
{
    INT underlyingAwaitingTasksBuffer[NUM_TASKS];
    RT_CIRCULAR_BUFFER awaitingTasks;
    RtCircularBufferInit(&awaitingTasks, underlyingAwaitingTasksBuffer, sizeof(underlyingAwaitingTasksBuffer));
    
    UCHAR underlyingLostTrainsBuffer[MAX_TRACKABLE_TRAINS];
    RT_CIRCULAR_BUFFER lostTrains;
    RtCircularBufferInit(&lostTrains, underlyingLostTrainsBuffer, sizeof(underlyingLostTrainsBuffer));

    UINT numTrains = 0;
    ATTRIBUTION_DATA trackedTrains[MAX_TRACKABLE_TRAINS];
    RtMemset(trackedTrains, sizeof(trackedTrains), 0);

    VERIFY(SUCCESSFUL(RegisterAs(ATTRIBUTION_SERVER_NAME)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, AttributionServerpSensorNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, AttributionServerpSpeedNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, AttributionServerpDirectionNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, AttributionServerpSwitchNotifierTask)));

    while(1)
    {
        INT senderId;
        ATTRIBUTION_SERVER_REQUEST request;

        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));

        switch(request.type)
        {
            case SensorChangedRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                TRACK_NODE* sensorNode = TrackFindSensor(&request.sensor);
                ATTRIBUTION_DATA* attributionData = AttributionServerpFindTrainByNextSensor(trackedTrains, numTrains, sensorNode);

                // If we couldn't find a train we expected to arrive at this sensor,
                // then maybe a train we're looking for tripped the sensor
                if(NULL == attributionData && !RtCircularBufferIsEmpty(&lostTrains))
                {
                    attributionData = &trackedTrains[numTrains];
                    numTrains = numTrains + 1;

                    VERIFY(RT_SUCCESS(RtCircularBufferPeekAndPop(&lostTrains, &attributionData->train, sizeof(attributionData->train))));

                    Log("Found train %d", attributionData->train);
                }

                // Make sure we matched the sensor to a train.  If not, just ignore the sensor
                if(NULL != attributionData)
                {
                    // Update the next sensor we expect for this train
                    attributionData->currentNode = sensorNode;

                    // Try to find where we expect the train to appear next
                    if(!SUCCESSFUL(TrackFindNextSensor(attributionData->currentNode, &attributionData->nextNode)))
                    {
                        attributionData->nextNode = NULL;
                        Log("Attribution server unable to find sensor after %s", sensorNode->name);
                    }

                    // Let any awaiting tasks know about the sensor
                    INT currentTime = Time();
                    ASSERT(SUCCESSFUL(currentTime));

                    ATTRIBUTED_SENSOR attributedSensor;
                    attributedSensor.train = attributionData->train;
                    attributedSensor.timeTripped = currentTime - AVERAGE_SENSOR_LATENCY;
                    attributedSensor.sensor = request.sensor;

                    INT awaitingTask;
                    while(!RtCircularBufferIsEmpty(&awaitingTasks))
                    {
                        VERIFY(RT_SUCCESS(RtCircularBufferPeekAndPop(&awaitingTasks, &awaitingTask, sizeof(awaitingTask))));
                        VERIFY(SUCCESSFUL(Reply(awaitingTask, &attributedSensor, sizeof(attributedSensor))));
                    }
                }
                else
                {
                    Log("Unexpected sensor %s", sensorNode->name);
                }

                break;
            }

            case SpeedChangedRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                // Check if this is a new train (i.e. a train we're not currently tracking)
                ATTRIBUTION_DATA* attributionData = AttributionServerpFindTrainById(trackedTrains, numTrains, request.trainSpeed.train);

                if(NULL == attributionData && 0 != request.trainSpeed.speed)
                {
                    VERIFY(RT_SUCCESS(RtCircularBufferPush(&lostTrains, &request.trainSpeed.train, sizeof(request.trainSpeed.train))));
                    Log("Searching for train %d", request.trainSpeed.train);
                }

                break;
            }

            case DirectionChangedRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                // Update where we expect the train to reach next
                ATTRIBUTION_DATA* attributionData = AttributionServerpFindTrainById(trackedTrains, numTrains, request.trainDirection.train);

                if(NULL != attributionData)
                {
                    TRACK_NODE* temp = attributionData->currentNode;
                    attributionData->currentNode = attributionData->nextNode->reverse;
                    attributionData->nextNode = temp->reverse;
                }
                else
                {
                    Log("UNTESTED: Train changed direction before we know where it is");
                }

                break;
            }

            case SwitchChangedRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));
                
                for(UINT i = 0; i < numTrains; i++)
                {
                    TRACK_NODE* nextBranch;
                    VERIFY(SUCCESSFUL(TrackFindNextBranch(trackedTrains[i].currentNode, &nextBranch)));

                    // Check if the switch that was just changed is between the current and next nodes
                    if(nextBranch->num == request.sw)
                    {
                        UINT distanceTillNextBranch;
                        VERIFY(SUCCESSFUL(TrackDistanceBetween(trackedTrains[i].currentNode, nextBranch, &distanceTillNextBranch)));

                        LOCATION location;
                        VERIFY(SUCCESSFUL(GetLocation(trackedTrains[i].train, &location)));

                        ASSERT(trackedTrains[i].currentNode == location.node);

                        // Figure out if the train is still before the branch (it could be before, on, or past the branch)
                        if(location.distancePastNode < distanceTillNextBranch)
                        {
                            // Train is before the branch, so the next sensor we expect to be tripped has changed
                            VERIFY(SUCCESSFUL(TrackFindNextSensor(trackedTrains[i].currentNode, &trackedTrains[i].nextNode)));
                        }
                    }
                }

                break;
            }

            case AttributedSensorAwaitRequest:
            {
                VERIFY(RT_SUCCESS(RtCircularBufferPush(&awaitingTasks, &senderId, sizeof(senderId))));
                break;
            }

            case GetTrackedTrainsRequest:
            {
                TRACKED_TRAINS trains;
                trains.numTrackedTrains = numTrains;

                for(UINT i = 0; i < numTrains; i++)
                {
                    trains.trains[i] = trackedTrains[i].train;
                }

                VERIFY(SUCCESSFUL(Reply(senderId, &trains, sizeof(trains))));
                break;
            }

            case NextExpectedNodeRequest:
            {
                ATTRIBUTION_DATA* attributionData = AttributionServerpFindTrainById(trackedTrains, numTrains, request.train);

                if(NULL != attributionData)
                {
                    VERIFY(SUCCESSFUL(Reply(senderId, &attributionData->nextNode, sizeof(attributionData->nextNode))));
                }
                else
                {
                    ASSERT(FALSE);
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
AttributionServerCreate
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(Create(Priority22, AttributionServerpTask)));
}

INT
AttributionServerGetTrackedTrains
    (
        OUT TRACKED_TRAINS* trackedTrains
    )
{
    INT result = WhoIs(ATTRIBUTION_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT attributionServerId = result;

        ATTRIBUTION_SERVER_REQUEST request;
        request.type = GetTrackedTrainsRequest;

        result = Send(attributionServerId, &request, sizeof(request), trackedTrains, sizeof(*trackedTrains));
    }

    return result;
}

INT
AttributionServerNextExpectedNode
    (
        IN UCHAR train,
        OUT TRACK_NODE** nextExpectedNode
    )
{
    INT result = WhoIs(ATTRIBUTION_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT attributionServerId = result;

        ATTRIBUTION_SERVER_REQUEST request;
        request.type = NextExpectedNodeRequest;
        request.train = train;

        result = Send(attributionServerId, &request, sizeof(request), nextExpectedNode, sizeof(*nextExpectedNode));
    }

    return result;
}

INT
AttributedSensorAwait
    (
        OUT ATTRIBUTED_SENSOR* attributedSensor
    )
{
    INT result = WhoIs(ATTRIBUTION_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT attributionServerId = result;

        ATTRIBUTION_SERVER_REQUEST request;
        request.type = AttributedSensorAwaitRequest;

        result = Send(attributionServerId, &request, sizeof(request), attributedSensor, sizeof(*attributedSensor));
    }

    return result;
}
