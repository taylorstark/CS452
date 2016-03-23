#include "sensor_server.h"

#include <rtkernel.h>
#include <rtos.h>
#include <user/trains.h>
#include <rtosc/assert.h>
#include <rtosc/buffer.h>
#include <rtosc/bitset.h>
#include <rtosc/string.h>

#include "display.h"

#define SENSOR_SERVER_NAME "sensor"

#define NUM_SENSORS 10
#define SENSOR_COMMAND_QUERY 0x85

typedef enum _SENSOR_SERVER_REQUEST_TYPE
{
    RegisterRequest = 0,
    DataRequest
} SENSOR_SERVER_REQUEST_TYPE;

typedef struct _SENSOR_SERVER_REQUEST
{
    SENSOR_SERVER_REQUEST_TYPE type;
    SENSOR_DATA sensorData;
} SENSOR_SERVER_REQUEST;

static
VOID
SensorServerpNotifierTask
    (
        VOID
    )
{
    INT sensorDeltaTaskId = MyParentTid();
    ASSERT(SUCCESSFUL(sensorDeltaTaskId));

    IO_DEVICE com1Device;
    VERIFY(SUCCESSFUL(Open(UartDevice, ChannelCom1, &com1Device)));

    Log("Waiting 5 seconds for junk sensor data.  Please wait.");
    VERIFY(SUCCESSFUL(Delay(500)));
    VERIFY(SUCCESSFUL(FlushInput(&com1Device)));
    Log("Flushed junk sensor data");

    while (1)
    {
        UCHAR sensors[NUM_SENSORS];

        VERIFY(SUCCESSFUL(WriteChar(&com1Device, SENSOR_COMMAND_QUERY)));
        VERIFY(SUCCESSFUL(Read(&com1Device, sensors, sizeof(sensors))));
        VERIFY(SUCCESSFUL(Send(sensorDeltaTaskId, sensors, sizeof(sensors), NULL, 0)));
    }
}

static
VOID
SensorServerpDeltaTask
    (
        VOID
    )
{
    SENSOR_SERVER_REQUEST request;
    request.type = DataRequest;

    UCHAR previousSensors[NUM_SENSORS];
    RtMemset(previousSensors, sizeof(previousSensors), 0);

    INT sensorServerId = MyParentTid();
    ASSERT(SUCCESSFUL(sensorServerId));    
    
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, SensorServerpNotifierTask)));

    while(1)
    {
        INT senderId;
        UCHAR currentSensors[NUM_SENSORS];

        VERIFY(SUCCESSFUL(Receive(&senderId, currentSensors, sizeof(currentSensors))));
        VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

        // Go through each module
        for(UINT i = 0; i < NUM_SENSORS; i++)
        {
            UCHAR previousValues = previousSensors[i];
            UCHAR currentValues = currentSensors[i];

            // Go through each sensor in this module
            for(UINT j = 0; j < 8; j++)
            {
                BOOLEAN previousValue = BIT_CHECK(previousValues, j);
                BOOLEAN currentValue = BIT_CHECK(currentValues, j);

                // Check to see if the sensor has changed
                if(previousValue != currentValue)
                {
                    request.sensorData.sensor.module = 'A' + (i / 2);
                    request.sensorData.sensor.number = (8 - j) + ((i % 2) * 8);
                    request.sensorData.isOn = currentValue;

                    VERIFY(SUCCESSFUL(Send(sensorServerId, &request, sizeof(request), NULL, 0)));
                }
            }
        }

        // Remember the sensor values for next time
        RtMemcpy(previousSensors, currentSensors, sizeof(previousSensors));
    }
}

static
VOID
SensorServerpTask
    (
        VOID
    )
{
    INT underlyingSubscriberBuffer[NUM_TASKS];
    RT_CIRCULAR_BUFFER subscriberBuffer;
    RtCircularBufferInit(&subscriberBuffer, underlyingSubscriberBuffer, sizeof(underlyingSubscriberBuffer));

    VERIFY(SUCCESSFUL(RegisterAs(SENSOR_SERVER_NAME)));
    VERIFY(SUCCESSFUL(Create(Priority13, SensorServerpDeltaTask)));

    while (1)
    {
        INT senderId;
        SENSOR_SERVER_REQUEST request;
        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));

        switch (request.type)
        {
            case RegisterRequest:
            {
                VERIFY(RT_SUCCESS(RtCircularBufferPush(&subscriberBuffer, &senderId, sizeof(senderId))));
                break;
            }

            case DataRequest:
            {
                // Tell any registrants about the tripped sensor
                INT subscriberId;
                while(RT_SUCCESS(RtCircularBufferPeekAndPop(&subscriberBuffer, &subscriberId, sizeof(subscriberId))))
                {
                    VERIFY(SUCCESSFUL(Reply(subscriberId, &request.sensorData, sizeof(request.sensorData))));
                }

                // Unblock the delta task
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));                
                break;
            }
        }
    }
}

VOID
SensorServerCreateTask
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(Create(Priority25, SensorServerpTask)));
}

INT
SensorAwait
    (
        OUT SENSOR_DATA* sensorData
    )
{
    INT result = WhoIs(SENSOR_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT sensorServerId = result;
        SENSOR_SERVER_REQUEST request;
        request.type = RegisterRequest;

        result = Send(sensorServerId, &request, sizeof(request), sensorData, sizeof(*sensorData));
    }

    return result;
}
