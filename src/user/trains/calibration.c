#include "calibration.h"

#include "display.h"
#include <rtkernel.h>
#include <rtos.h>
#include <rtosc/assert.h>
#include <user/trains.h>

#define TRAIN_NUMBER 58

static
VOID
CalibrationpSteadyStateVelocityTask
    (
        VOID
    )
{
    // Setup the track
    VERIFY(SUCCESSFUL(SwitchSetDirection(15, SwitchStraight)));
    VERIFY(SUCCESSFUL(SwitchSetDirection(14, SwitchStraight)));
    VERIFY(SUCCESSFUL(SwitchSetDirection(9, SwitchStraight)));
    VERIFY(SUCCESSFUL(SwitchSetDirection(8, SwitchStraight)));
    VERIFY(SUCCESSFUL(SwitchSetDirection(7, SwitchStraight)));
    VERIFY(SUCCESSFUL(SwitchSetDirection(6, SwitchStraight)));

    // Wait for the track to set up
    VERIFY(SUCCESSFUL(Delay(100)));

    // Have the train go
    //UINT startTime = 0;
    UCHAR currentSpeed = 14;
    VERIFY(SUCCESSFUL(TrainSetSpeed(TRAIN_NUMBER, currentSpeed)));

    while(1)
    {
        SENSOR_DATA data;
        VERIFY(SUCCESSFUL(SensorAwait(&data)));

        if(!data.isOn)
        {
            continue;
        }

        if('C' == data.sensor.module && 13 == data.sensor.number)
        {
            //startTime = Time();
            VERIFY(SUCCESSFUL(TrainSetSpeed(TRAIN_NUMBER, 0)));
        }
        else if('E' == data.sensor.module && 7 == data.sensor.number)
        {
            /*
            UINT totalTime = Time() - startTime;
            Log("%d", totalTime);

            if(currentSpeed == 6)
            {
                Log("");
                currentSpeed = 14;
            }
            else
            {
                currentSpeed = currentSpeed - 1;
            }

            VERIFY(SUCCESSFUL(TrainSetSpeed(TRAIN_NUMBER, currentSpeed)));
            */
        }
    }
}

VOID
CalibrationCreateTask
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, CalibrationpSteadyStateVelocityTask)));
}
