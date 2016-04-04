#include <rtosc/assert.h>
#include <rtos.h>
#include <user/trains.h>

#include "attribution_server.h"
#include "calibration.h"
#include "conductor.h"
#include "clock.h"
#include "destination_server.h"
#include "display.h"
#include "input_parser.h"
#include "location_server.h"
#include "performance.h"
#include "physics.h"
#include "route_server.h"
#include "safety.h"
#include "scheduler.h"
#include "sensor_server.h"
#include "stop_server.h"
#include "switch_server.h"
#include "train_server.h"

VOID
InitTrainTasks
    (
        VOID
    )
{
    // Initialize libraries
    PhysicsInit();
    TrackInit(TrackA);

    // Setup the display
    DisplayCreateTask();
    ClockCreateTask();
    PerformanceCreateTask();
    InputParserCreateTask();

    // Setup the track
    TrainServerCreate();
    SwitchServerCreate();

    // Initialize remaining tasks
    SensorServerCreateTask();
    AttributionServerCreate();
    LocationServerCreateTask();
    SafetyCreateTask();
    SchedulerCreateTask();
    RouteServerCreate();
    ConductorCreateTask();
    StopServerCreate();
    DestinationServerCreate();

    //CalibrationCreateTask();
}
