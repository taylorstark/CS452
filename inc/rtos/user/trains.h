#pragma once

#include <rt.h>
#include <track/track_data.h>
#include <track/track_node.h>

#define MAX_TRAINS 80
#define MAX_TRACKABLE_TRAINS 6

/************************************
 *          TRAIN API               *
 ************************************/

#define MAX_SPEED 14
#define AVERAGE_TRAIN_COMMAND_LATENCY 12 // 120 ms

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
    UINT acceleration;
    UINT accelerationTicks;
} TRAIN_LOCATION;

INT
GetLocation
    (
        IN UCHAR train, 
        OUT LOCATION* location
    );

INT
LocationAwait
    (
        OUT TRAIN_LOCATION* trainLocation
    );

/************************************
 *       DESTINATION API            *
 ************************************/

INT
TrainDestinationOnce
    (
        IN UCHAR train, 
        IN LOCATION* location
    );

INT
TrainDestinationForever
    (
        IN UCHAR train
    );

/************************************
 *          ROUTE API               *
 ************************************/

typedef struct _PATH_NODE
{
    TRACK_NODE* node;
    UINT direction;
    UINT expectedArrivalTime;
} PATH_NODE;

typedef struct _PATH
{
    PATH_NODE nodes[TRACK_MAX];
    UINT numNodes;
    UINT totalDistance;
    BOOLEAN performsReverse;
} PATH;

typedef struct _ROUTE
{
    TRAIN_LOCATION trainLocation;
    PATH path;
} ROUTE;

INT
RouteTrainToDestination
    (
        IN UCHAR train, 
        IN LOCATION* destination
    );

INT
RouteClearDestination
    (
        IN UCHAR train
    );

INT
RouteAwait
    (
        OUT ROUTE* route
    );

/************************************
 *          STOP API                *
 ************************************/

typedef struct _DESTINATION_REACHED
{
    UCHAR train;
    LOCATION location;
} DESTINATION_REACHED;

INT
StopTrainAtLocation
    (
        IN UCHAR train,
        IN LOCATION* location
    );

INT
DestinationReachedAwait
    (
        OUT DESTINATION_REACHED* destinationReached
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
GetTrack
    (
        VOID
    );

TRACK_NODE*
TrackFindSensor
    (
        IN SENSOR* sensor
    );

TRACK_EDGE*
TrackNextEdge
    (
        IN TRACK_NODE* node
    );

TRACK_NODE*
TrackNextNode
    (
        IN TRACK_NODE* node
    );

INT
TrackFindNextBranch
    (
        IN TRACK_NODE* node, 
        OUT TRACK_NODE** nextBranch
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
