#include "calibration.h"

#include <bwio/bwio.h>
#include <rtkernel.h>
#include <rtos.h>
#include <rtosc/assert.h>
#include <user/trains.h>

#define TRAIN_NUMBER 63

static
VOID
CalibrationpSteadyStateVelocityTask
    (
        VOID
    )
{
    // Setup the track
    // This currently assumes track B
    VERIFY(SUCCESSFUL(SwitchSetDirection(15, SwitchStraight)));
    VERIFY(SUCCESSFUL(SwitchSetDirection(14, SwitchStraight)));
    VERIFY(SUCCESSFUL(SwitchSetDirection(9, SwitchStraight)));
    VERIFY(SUCCESSFUL(SwitchSetDirection(8, SwitchStraight)));
    VERIFY(SUCCESSFUL(SwitchSetDirection(7, SwitchStraight)));
    VERIFY(SUCCESSFUL(SwitchSetDirection(6, SwitchStraight)));

    // Wait for the track to set up
    VERIFY(SUCCESSFUL(Delay(100)));

    // Have the train go
    UINT startTime = 0;
    UINT currentSpeed = 14;
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
            startTime = Time();
        }
        else if('E' == data.sensor.module && 7 == data.sensor.number)
        {
            UINT totalTime = Time() - startTime;
            bwprintf(BWCOM2, "%d\r\n", totalTime);

            if(currentSpeed == 5)
            {
                bwprintf(BWCOM2, "\r\n");
                currentSpeed = 14;
            }
            else
            {
                currentSpeed = currentSpeed - 1;
            }

            VERIFY(SUCCESSFUL(TrainSetSpeed(TRAIN_NUMBER, currentSpeed)));
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
