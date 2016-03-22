#include "safety.h"

#include "display.h"
#include <rtosc/assert.h>
#include <rtosc/string.h>
#include <rtkernel.h>
#include <rtos.h>
#include <user/trains.h>

typedef enum _SAFETY_REQUEST_TYPE
{
    CheckTrain = 0, 
    CheckAllTrains
} SAFETY_REQUEST_TYPE;

typedef struct _SAFETY_REQUEST
{
    SAFETY_REQUEST_TYPE type;
    UCHAR train;
} SAFETY_REQUEST;

static
VOID
SafetypAttributedSensorNotifierTask
    (
        VOID
    )
{
    INT safetyTaskId = MyParentTid();
    ASSERT(SUCCESSFUL(safetyTaskId));

    SAFETY_REQUEST request;
    request.type = CheckTrain;

    while(1)
    {
        ATTRIBUTED_SENSOR attributedSensor;
        VERIFY(SUCCESSFUL(AttributedSensorAwait(&attributedSensor)));

        request.train = attributedSensor.train;
        VERIFY(SUCCESSFUL(Send(safetyTaskId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
SafetypSwitchNotifierTask
    (
        VOID
    )
{
    INT safetyTaskId = MyParentTid();
    ASSERT(SUCCESSFUL(safetyTaskId));

    SAFETY_REQUEST request;
    request.type = CheckAllTrains;

    while(1)
    {
        INT unused;
        VERIFY(SUCCESSFUL(SwitchChangeAwait(&unused)));
        VERIFY(SUCCESSFUL(Send(safetyTaskId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
SafetypCheckTrain
    (
        IN UCHAR train
    )
{
    TRACK_NODE* nextNode;
    VERIFY(SUCCESSFUL(AttributionServerNextExpectedNode(train, &nextNode)));

    if(NULL == nextNode)
    {
        VERIFY(SUCCESSFUL(TrainSetSpeed(train, 0)));
        Log("Safety: Stopping %d", train);
    }
}

static
VOID
SafetypTask
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, SafetypAttributedSensorNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, SafetypSwitchNotifierTask)));

    while(1)
    {
        INT senderId;
        SAFETY_REQUEST request;

        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));

        // Reply to notifier right away
        VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

        switch(request.type)
        {
            case CheckTrain:
            {
                SafetypCheckTrain(request.train);
                break;
            }

            case CheckAllTrains:
            {
                TRACKED_TRAINS trackedTrains;
                VERIFY(SUCCESSFUL(AttributionServerGetTrackedTrains(&trackedTrains)));

                for(UINT i = 0; i < trackedTrains.numTrackedTrains; i++)
                {
                    SafetypCheckTrain(trackedTrains.trains[i]);
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
SafetyCreateTask
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(Create(Priority22, SafetypTask)));
}
