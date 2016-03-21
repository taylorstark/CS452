#include "scheduler.h"

#include "display.h"
#include "physics.h"
#include <rtosc/assert.h>
#include <rtosc/string.h>
#include <rtkernel.h>
#include <rtos.h>
#include <user/trains.h>

#define SCHEDULER_NAME "scheduler"

#define SCHEDULER_TRAIN_NOT_MOVING_THRESHOLD 100

// Debug builds are slower than release builds
#ifdef NDEBUG
#define SCHEDULER_ALLOWABLE_ARRIVAL_THRESHOLD 5 // 50 ms
#else
#define SCHEDULER_ALLOWABLE_ARRIVAL_THRESHOLD 10 // 100 ms
#endif

typedef enum _SCHEDULER_REQUEST_TYPE
{
    TrainChangedNextNodeRequest = 0,
    LocationUpdateRequest, 
    AttributedSensorRequest
} SCHEDULER_REQUEST_TYPE;

typedef struct _SCHEDULER_REQUEST
{
    SCHEDULER_REQUEST_TYPE type;

    union
    {
        ATTRIBUTED_SENSOR attributedSensor;
        TRAIN_LOCATION trainLocation;
    };
} SCHEDULER_REQUEST;

typedef struct _TRAIN_SCHEDULE
{
    TRACK_NODE* nextNode;
    INT expectedArrivalTime;
} TRAIN_SCHEDULE;

static
VOID
SchedulerpLocationNotifierTask
    (
        VOID
    )
{
    INT schedulerId = MyParentTid();
    ASSERT(SUCCESSFUL(schedulerId));

    SCHEDULER_REQUEST request;
    request.type = LocationUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(LocationAwait(&request.trainLocation)));
        VERIFY(SUCCESSFUL(Send(schedulerId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
SchedulerpAttributedSensorNotifierTask
    (
        VOID
    )
{
    INT schedulerId = MyParentTid();
    ASSERT(SUCCESSFUL(schedulerId));

    SCHEDULER_REQUEST request;
    request.type = AttributedSensorRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(AttributedSensorAwait(&request.attributedSensor)));
        VERIFY(SUCCESSFUL(Send(schedulerId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
SchedulerpTask
    (
        VOID
    )
{
    TRAIN_SCHEDULE trainSchedules[MAX_TRAINS];
    RtMemset(trainSchedules, sizeof(trainSchedules), 0);

    VERIFY(SUCCESSFUL(RegisterAs(SCHEDULER_NAME)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, SchedulerpLocationNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, SchedulerpAttributedSensorNotifierTask)));

    while(1)
    {
        INT senderId;
        SCHEDULER_REQUEST request;

        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));

        switch(request.type)
        {
            case LocationUpdateRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                // Use the train's updated location to compute the next expected arrival time
                TRAIN_SCHEDULE* trainSchedule = &trainSchedules[request.trainLocation.train];
                VERIFY(SUCCESSFUL(AttributionServerNextExpectedNode(request.trainLocation.train, &trainSchedule->nextNode)));

                // We may be in the middle of a reverse, in which case we may not be able to find a path to the next node
                UINT distanceBetweenNodes;
                if(SUCCESSFUL(TrackDistanceBetween(request.trainLocation.location.node, trainSchedule->nextNode, &distanceBetweenNodes)))
                {
                    if(request.trainLocation.velocity > SCHEDULER_TRAIN_NOT_MOVING_THRESHOLD)
                    {
                        // Due to sensor latency, we may believe we've gone past the sensor
                        // If we think we've gone past the sensor, then just use our last arrival time guess
                        if(distanceBetweenNodes > request.trainLocation.location.distancePastNode)
                        {
                            INT currentTime = Time();
                            ASSERT(SUCCESSFUL(currentTime));

                            UINT timeTillNextNode = (distanceBetweenNodes - request.trainLocation.location.distancePastNode) / request.trainLocation.velocity;

                            trainSchedule->expectedArrivalTime = currentTime + timeTillNextNode;
                        }
                    }
                    else
                    {
                        trainSchedule->expectedArrivalTime = 0;
                    }
                }
                else
                {
                    trainSchedule->expectedArrivalTime = 0;
                }

                break;
            }

            case AttributedSensorRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                // Use the attributed sensor to see if the train arrived on time
                TRAIN_SCHEDULE* trainSchedule = &trainSchedules[request.attributedSensor.train];
                TRACK_NODE* trippedSensor = TrackFindSensor(&request.attributedSensor.sensor);

                // This might be the first time we've seen this train
                if(trainSchedule->expectedArrivalTime > 0)
                {                    
                    if(trippedSensor == trainSchedule->nextNode)
                    {
                        INT diff = request.attributedSensor.timeTripped - trainSchedule->expectedArrivalTime;

                        if(abs(diff) > SCHEDULER_ALLOWABLE_ARRIVAL_THRESHOLD)
                        {
                            ShowTrainArrival(request.attributedSensor.train, (STRING) trainSchedule->nextNode->name, diff);
                        }
                    }
                    else
                    {
                        Log("Scheduler expected train %d to arrive at %s but arrived at %s",
                            request.attributedSensor.train,
                            trainSchedule->nextNode->name, 
                            trippedSensor->name);
                    }                    
                }

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
SchedulerCreateTask
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(Create(Priority15, SchedulerpTask)));
}
