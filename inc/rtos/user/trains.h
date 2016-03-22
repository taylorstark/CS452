#pragma once

#include <rt.h>
#include <track/track_node.h>

#define MAX_TRAINS 80
#define MAX_TRACKABLE_TRAINS 6

/************************************
 *          TRAIN API               *
 ************************************/

#define MAX_SPEED 14

typedef struct _TRAIN_SPEED
{
    UCHAR train;
    UCHAR speed;
} TRAIN_SPEED;

typedef enum _DIRECTION
{
    DirectionForward = 0, 
    DirectionReverse
} DIRECTION;

typedef struct _TRAIN_DIRECTION
{
    UCHAR train;
    DIRECTION direction;
} TRAIN_DIRECTION;

INT
TrainGetSpeed
    (
        IN INT train,
        OUT UCHAR* speed
    );

INT
TrainSetSpeed
    (
        IN INT train,
        IN INT speed
    );

INT
TrainSpeedChangeAwait
    (
        OUT TRAIN_SPEED* trainSpeed
    );

INT
TrainReverse
    (
        IN INT train
    );

INT
TrainDirectionChangeAwait
    (
        OUT TRAIN_DIRECTION* trainDirection
    );

/************************************
 *          SWITCH API              *
 ************************************/

typedef enum _SWITCH_DIRECTION
{
    SwitchCurved = 0,
    SwitchStraight
} SWITCH_DIRECTION;

INT
SwitchSetDirection
    (
        IN INT sw,
        IN SWITCH_DIRECTION direction
    );

INT
SwitchGetDirection
    (
        IN INT sw, 
        OUT SWITCH_DIRECTION* direction
    );

INT
SwitchChangeAwait
    (
        OUT INT* sw
    );

/************************************
 *           SENSOR API             *
 ************************************/

#define AVERAGE_SENSOR_LATENCY 7 // 70ms

typedef struct _SENSOR
{
    CHAR module;
    UINT number;
} SENSOR;

typedef struct _SENSOR_DATA
{
    SENSOR sensor;
    BOOLEAN isOn;
} SENSOR_DATA;

INT
SensorAwait
    (
        OUT SENSOR_DATA* sensorData
    );

/************************************
 *         ATTRIBUTION API          *
 ************************************/

typedef struct _ATTRIBUTED_SENSOR
{
    UCHAR train;
    INT timeTripped;
    SENSOR sensor;
} ATTRIBUTED_SENSOR;

typedef struct _TRACKED_TRAINS
{
    UCHAR trains[MAX_TRACKABLE_TRAINS];
    UINT numTrackedTrains;
} TRACKED_TRAINS;

INT
AttributionServerGetTrackedTrains
    (
        OUT TRACKED_TRAINS* trackedTrains
    );

INT
AttributionServerNextExpectedNode
    (
        IN UCHAR train,
        OUT TRACK_NODE** nextExpectedNode
    );

INT
AttributedSensorAwait
    (
        OUT ATTRIBUTED_SENSOR* attributedSensor
    );

/************************************
 *          LOCATION API            *
 ************************************/

typedef struct _LOCATION
{
    TRACK_NODE* node;
    UINT distancePastNode; // in micrometers
} LOCATION;

typedef struct _TRAIN_LOCATION
{
    UCHAR train;
    LOCATION location;
    UINT velocity; // in micrometers / tick
} TRAIN_LOCATION;

INT
LocationAwait
    (
        OUT TRAIN_LOCATION* trainLocation
    );

/************************************
 *          STOP API                *
 ************************************/

INT
StopTrainAtLocation
    (
        IN UCHAR train,
        IN LOCATION* location
    );

/************************************
 *            TRACK API             *
 ************************************/
typedef enum _TRACK
{
    TrackA = 0, 
    TrackB
} TRACK;

VOID
TrackInit
    (
        IN TRACK track
    );

TRACK_NODE*
TrackFindSensor
    (
        IN SENSOR* sensor
    );

INT
TrackFindNextSensor
    (
        IN TRACK_NODE* node,
        OUT TRACK_NODE** nextSensor
    );

INT
TrackDistanceBetween
    (
        IN TRACK_NODE* n1,
        IN TRACK_NODE* n2,
        OUT UINT* distance
    );

/************************************
 *           INIT TASK              *
 ************************************/

VOID
InitTrainTasks
    (
        VOID
    );
