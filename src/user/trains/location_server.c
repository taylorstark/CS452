#include "location_server.h"

#include "display.h"
#include "physics.h"
#include <rtosc/assert.h>
#include <rtosc/buffer.h>
#include <rtosc/string.h>
#include <rtkernel.h>
#include <rtos.h>
#include <track/track_node.h>
#include <user/trains.h>

#define LOCATION_SERVER_NAME "location"
#define LOCATION_SERVER_REGISTRAR_NAME "location_registrar"

#define LOCATION_SERVER_NOTIFIER_UPDATE_INTERVAL 2 // 20 ms
#define LOCATION_SERVER_ALPHA 5

typedef enum _LOCATION_SERVER_REQUEST_TYPE
{
    VelocityUpdateRequest = 0,
    AttributedSensorUpdateRequest,
    SpeedUpdateRequest,
    DirectionUpdateRequest, 
    GetLocationRequest
} LOCATION_SERVER_REQUEST_TYPE;

typedef struct _LOCATION_SERVER_REQUEST
{
    LOCATION_SERVER_REQUEST_TYPE type;

    union
    {
        UCHAR train;
        ATTRIBUTED_SENSOR attributedSensor;
        TRAIN_SPEED trainSpeed;
        TRAIN_DIRECTION trainDirection;
    };
} LOCATION_SERVER_REQUEST;

typedef enum _ACCELERATION_TYPE
{
    ACCELERATING = 0, 
    DECELERATING, 
    STOPPING
} ACCELERATION_TYPE;

typedef struct _TRAIN_DATA
{
    UCHAR train;
    LOCATION location;
    UINT velocity; // in micrometers / tick
    ACCELERATION_TYPE accelerationType;
    UINT accelerationTicks;
    INT accelerationStartTime;
    INT lastArrivalTime;
    INT lastTimeLocationUpdated;
} TRAIN_DATA;

typedef enum _LOCATION_SERVER_REGISTRAR_REQUEST_TYPE
{
    LocationAwaitRequest = 0, 
    LocationUpdateRequest
} LOCATION_SERVER_REGISTRAR_REQUEST_TYPE;

typedef struct _LOCATION_SERVER_REGISTRAR_REQUEST
{
    LOCATION_SERVER_REGISTRAR_REQUEST_TYPE type;
    TRAIN_LOCATION trainLocation;
} LOCATION_SERVER_REGISTRAR_REQUEST;

static
VOID
LocationServerpVelocityNotifierTask
    (
        VOID
    )
{
    INT locationServerId = MyParentTid();
    ASSERT(SUCCESSFUL(locationServerId));

    LOCATION_SERVER_REQUEST request;
    request.type = VelocityUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(Delay(LOCATION_SERVER_NOTIFIER_UPDATE_INTERVAL)));
        VERIFY(SUCCESSFUL(Send(locationServerId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
LocationServerpAttributedSensorNotifierTask
    (
        VOID
    )
{
    INT locationServerId = MyParentTid();
    ASSERT(SUCCESSFUL(locationServerId));

    LOCATION_SERVER_REQUEST request;
    request.type = AttributedSensorUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(AttributedSensorAwait(&request.attributedSensor)));
        VERIFY(SUCCESSFUL(Send(locationServerId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
LocationServerpSpeedChangeNotifierTask
    (
        VOID
    )
{
    INT locationServerId = MyParentTid();
    ASSERT(SUCCESSFUL(locationServerId));

    LOCATION_SERVER_REQUEST request;
    request.type = SpeedUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(TrainSpeedChangeAwait(&request.trainSpeed)));
        VERIFY(SUCCESSFUL(Send(locationServerId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
LocationServerpDirectionChangeNotifierTask
    (
        VOID
    )
{
    INT locationServerId = MyParentTid();
    ASSERT(SUCCESSFUL(locationServerId));

    LOCATION_SERVER_REQUEST request;
    request.type = DirectionUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(TrainDirectionChangeAwait(&request.trainDirection)));
        VERIFY(SUCCESSFUL(Send(locationServerId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
LocationServerpRegistrarTask
    (
        VOID
    )
{
    INT underlyingAwaitingTasksBuffer[NUM_TASKS];
    RT_CIRCULAR_BUFFER awaitingTasks;
    RtCircularBufferInit(&awaitingTasks, underlyingAwaitingTasksBuffer, sizeof(underlyingAwaitingTasksBuffer));

    VERIFY(SUCCESSFUL(RegisterAs(LOCATION_SERVER_REGISTRAR_NAME)));

    while(1)
    {
        INT senderId;
        LOCATION_SERVER_REGISTRAR_REQUEST request;

        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));

        switch(request.type)
        {
            case LocationAwaitRequest:
            {
                VERIFY(RT_SUCCESS(RtCircularBufferPush(&awaitingTasks, &senderId, sizeof(senderId))));
                break;
            }

            case LocationUpdateRequest:
            {
                INT awaitingTask;
                while(!RtCircularBufferIsEmpty(&awaitingTasks))
                {
                    VERIFY(RT_SUCCESS(RtCircularBufferPeekAndPop(&awaitingTasks, &awaitingTask, sizeof(awaitingTask))));
                    VERIFY(SUCCESSFUL(Reply(awaitingTask, &request.trainLocation, sizeof(request.trainLocation))));
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

static
TRAIN_DATA*
LocationServerpFindTrainById
    (
        IN TRAIN_DATA* trains,
        IN UINT numTrains,
        IN UCHAR train
    )
{
    for(UINT i = 0; i < numTrains; i++)
    {
        TRAIN_DATA* trainData = &trains[i];

        if(train == trainData->train)
        {
            return trainData;
        }
    }

    return NULL;
}

static
inline
BOOLEAN
LocationServerpIsAccelerating
    (
        IN TRAIN_DATA* trainData
    )
{
    return trainData->accelerationTicks > 0;
}

static
VOID
LocationServerpTask
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(RegisterAs(LOCATION_SERVER_NAME)));
    VERIFY(SUCCESSFUL(Create(Priority23, LocationServerpVelocityNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, LocationServerpAttributedSensorNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, LocationServerpSpeedChangeNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, LocationServerpDirectionChangeNotifierTask)));

    INT locationServerRegistrarId = Create(Priority13, LocationServerpRegistrarTask);
    ASSERT(SUCCESSFUL(locationServerRegistrarId));

    UINT numTrackedTrains = 0;

    TRAIN_DATA trackedTrains[MAX_TRACKABLE_TRAINS];
    RtMemset(trackedTrains, sizeof(trackedTrains), 0);

    while(1)
    {
        INT senderId;
        LOCATION_SERVER_REQUEST request;

        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));

        switch(request.type)
        {
            case VelocityUpdateRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                // Calculate all tracked train's updated locations
                INT currentTime = Time();
                ASSERT(SUCCESSFUL(currentTime));

                LOCATION_SERVER_REGISTRAR_REQUEST request;
                request.type = LocationUpdateRequest;
                
                for(UINT i = 0; i < numTrackedTrains; i++)
                {
                    TRAIN_DATA* trainData = &trackedTrains[i];

                    // Do we know where this train is yet?
                    if(NULL != trainData->location.node)
                    {
                        INT diff = currentTime - trainData->lastTimeLocationUpdated;

                        if(diff > 0)
                        {
                            // Handle acceleration
                            if(trainData->accelerationTicks > 0 && currentTime >= trainData->accelerationStartTime)
                            {
                                UINT timeSpentAccelerating = min(diff, trainData->accelerationTicks);
                                trainData->accelerationTicks -= timeSpentAccelerating;

                                UINT dv = timeSpentAccelerating * PhysicsAcceleration(trainData->train);

                                if(ACCELERATING == trainData->accelerationType)
                                {
                                    trainData->velocity += dv;
                                }
                                else
                                {
                                    if(dv < trainData->velocity)
                                    {
                                        trainData->velocity -= dv;

                                        if(STOPPING == trainData->accelerationType && 0 == trainData->accelerationTicks)
                                        {
                                            trainData->velocity = 0;
                                            Log("Stopped %d %s %d", trainData->train, trainData->location.node->name, trainData->location.distancePastNode);
                                        }
                                    }
                                    else
                                    {
                                        trainData->velocity = 0;
                                    }
                                }
                            }

                            // Update the train's location
                            trainData->location.distancePastNode += diff * trainData->velocity;
                            trainData->lastTimeLocationUpdated = currentTime;
                            
                            // Send the updated location to the registrar to send to any registrants
                            request.trainLocation.train = trainData->train;
                            request.trainLocation.location = trainData->location;
                            request.trainLocation.velocity = trainData->velocity;

                            VERIFY(SUCCESSFUL(Send(locationServerRegistrarId, &request, sizeof(request), NULL, 0)));
                        }
                    }
                }

                break;
            }

            case AttributedSensorUpdateRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                // Get the data we have on this train
                TRACK_NODE* sensorNode = TrackFindSensor(&request.attributedSensor.sensor);
                TRAIN_DATA* trainData = LocationServerpFindTrainById(trackedTrains, numTrackedTrains, request.attributedSensor.train);

                if(NULL != trainData)
                {
                    // Update the velocity if we have a point of reference
                    INT currentTime = Time();
                    ASSERT(SUCCESSFUL(currentTime));

                    if(NULL != trainData->location.node && !LocationServerpIsAccelerating(trainData))
                    {
                        UINT dx;
                        if(SUCCESSFUL(TrackDistanceBetween(trainData->location.node, sensorNode, &dx)))
                        {
                            UINT dt = request.attributedSensor.timeTripped - trainData->lastArrivalTime;
                            UINT v = dx / dt;

                            UINT newVelocityFactor = LOCATION_SERVER_ALPHA * v;
                            UINT oldVelocityFactor = (100 - LOCATION_SERVER_ALPHA) * trainData->velocity;
                            trainData->velocity = (newVelocityFactor + oldVelocityFactor) / 100;
                        }                        
                    }

                    // Update the train's location
                    trainData->location.node = sensorNode;
                    trainData->location.distancePastNode = (currentTime - request.attributedSensor.timeTripped) * trainData->velocity;
                    trainData->lastArrivalTime = request.attributedSensor.timeTripped;
                    trainData->lastTimeLocationUpdated = currentTime;
                }
                
                break;
            }

            case SpeedUpdateRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                // Update the train's velocity and acceleration
                TRAIN_DATA* trainData = LocationServerpFindTrainById(trackedTrains, numTrackedTrains, request.trainSpeed.train);

                if(NULL == trainData)
                {
                    trainData = &trackedTrains[numTrackedTrains];
                    numTrackedTrains = numTrackedTrains + 1;

                    trainData->train = request.trainSpeed.train;
                }

                // Figure out how long it will take to reach the new velocity
                // TODO: This is a pretty bad approximation
                UINT targetVelocity = PhysicsSteadyStateVelocity(request.trainSpeed.train, request.trainSpeed.speed);
                UINT acceleration = PhysicsAcceleration(request.trainSpeed.train);

                INT currentTime = Time();
                ASSERT(SUCCESSFUL(currentTime));

                trainData->accelerationStartTime = currentTime + AVERAGE_TRAIN_COMMAND_LATENCY;

                if(0 == targetVelocity)
                {
                    trainData->accelerationType = STOPPING;
                    trainData->accelerationTicks = trainData->velocity / acceleration;
                }
                else
                {
                    if(targetVelocity > trainData->velocity)
                    {
                        // Accelerating from stop takes a pretty long time
                        if(0 == trainData->velocity)
                        {
                            acceleration *= 75;
                            acceleration /= 100;
                        }
                        
                        trainData->accelerationType = ACCELERATING;
                        trainData->accelerationTicks = (targetVelocity - trainData->velocity) / acceleration;
                    }
                    else
                    {
                        trainData->accelerationType = DECELERATING;
                        trainData->accelerationTicks = (trainData->velocity - targetVelocity) / acceleration;
                    }
                }

                break;
            }

            case DirectionUpdateRequest:
            {
                // Unblock the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                // Update the train's point of reference
                TRAIN_DATA* trainData = LocationServerpFindTrainById(trackedTrains, numTrackedTrains, request.trainDirection.train);

                if(NULL != trainData)
                {
                    TRACK_NODE* newPointOfReferenceNode;
                    VERIFY(SUCCESSFUL(TrackFindNextSensor(trainData->location.node, &newPointOfReferenceNode)));

                    // Find the distance between our current point of reference and our new point of reference
                    UINT distance;
                    VERIFY(SUCCESSFUL(TrackDistanceBetween(trainData->location.node, newPointOfReferenceNode, &distance)));

                    // Update the point of reference
                    trainData->location.node = newPointOfReferenceNode->reverse;
                    trainData->location.distancePastNode = distance - trainData->location.distancePastNode;
                }

                break;
            }

            case GetLocationRequest:
            {
                TRAIN_DATA* trainData = LocationServerpFindTrainById(trackedTrains, numTrackedTrains, request.train);

                if(NULL != trainData)
                {
                    VERIFY(SUCCESSFUL(Reply(senderId, &trainData->location, sizeof(trainData->location))));
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
LocationServerCreateTask
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(Create(Priority23, LocationServerpTask)));
}

INT
GetLocation
    (
        IN UCHAR train, 
        OUT LOCATION* location
    )
{
    INT result = WhoIs(LOCATION_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT locationServerId = result;

        LOCATION_SERVER_REQUEST request;
        request.type = GetLocationRequest;
        request.train = train;

        result = Send(locationServerId, &request, sizeof(request), location, sizeof(*location));
    }

    return result;
}

INT
LocationAwait
    (
        OUT TRAIN_LOCATION* trainLocation
    )
{
    INT result = WhoIs(LOCATION_SERVER_REGISTRAR_NAME);

    if(SUCCESSFUL(result))
    {
        INT locationServerRegistrarId = result;

        LOCATION_SERVER_REGISTRAR_REQUEST request;
        request.type = LocationAwaitRequest;

        result = Send(locationServerRegistrarId, &request, sizeof(request), trainLocation, sizeof(*trainLocation));
    }

    return result;
}
