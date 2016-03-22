#include "physics.h"

#include <rtosc/assert.h>
#include <rtosc/string.h>
#include <user/trains.h>

static UINT g_steadyStateVelocities[MAX_TRAINS + 1][MAX_SPEED + 1]; // in micrometers per tick
static UINT g_decelerations[MAX_TRAINS + 1]; // in micrometers per tick^2

VOID
PhysicsInit
    (
        VOID
    )
{
    RtMemset(g_steadyStateVelocities, sizeof(g_steadyStateVelocities), 0);

    g_steadyStateVelocities[58][6] = 2834;
    g_steadyStateVelocities[58][7] = 3401;
    g_steadyStateVelocities[58][8] = 3833;
    g_steadyStateVelocities[58][9] = 4435;
    g_steadyStateVelocities[58][10] = 4861;
    g_steadyStateVelocities[58][11] = 5417;
    g_steadyStateVelocities[58][12] = 5962;
    g_steadyStateVelocities[58][13] = 5962;
    g_steadyStateVelocities[58][14] = 5962;

    g_steadyStateVelocities[63][6] = 2834;
    g_steadyStateVelocities[63][7] = 3401;
    g_steadyStateVelocities[63][8] = 3833;
    g_steadyStateVelocities[63][9] = 4435;
    g_steadyStateVelocities[63][10] = 4861;
    g_steadyStateVelocities[63][11] = 5417;
    g_steadyStateVelocities[63][12] = 5962;
    g_steadyStateVelocities[63][13] = 5962;
    g_steadyStateVelocities[63][14] = 5962;

    g_steadyStateVelocities[69][6] = 2912;
    g_steadyStateVelocities[69][7] = 3521;
    g_steadyStateVelocities[69][8] = 3949;
    g_steadyStateVelocities[69][9] = 4448;
    g_steadyStateVelocities[69][10] = 5025;
    g_steadyStateVelocities[69][11] = 5476;
    g_steadyStateVelocities[69][12] = 5924;
    g_steadyStateVelocities[69][13] = 5924;
    g_steadyStateVelocities[69][14] = 5924;

    RtMemset(g_decelerations, sizeof(g_decelerations), 0);
    g_decelerations[63] = 1635;
    g_decelerations[69] = 1820;
}

UINT
PhysicsSteadyStateVelocity
    (
        IN UCHAR train,
        IN UCHAR speed
    )
{
    UINT steadyStateVelocity = g_steadyStateVelocities[train][speed];

    if(0 != speed && 0 == steadyStateVelocity)
    {
        ASSERT(FALSE);
    }

    return steadyStateVelocity;
}

static
INT
PhysicspDeceleration
    (
        IN UCHAR train
    )
{
    UINT deceleration = g_decelerations[train];

    if(0 == deceleration)
    {
        ASSERT(FALSE);
    }

    return deceleration;
}

static
inline
UINT
PhysicspDistanceFromPickupToFrontOfTrain
    (
        IN DIRECTION direction
    )
{
    return DirectionForward == direction ? 20000 : 140000;
}

UINT
PhysicsStoppingDistance
    (
        IN UCHAR train, 
        IN UINT velocity, 
        IN DIRECTION direction
    )
{
    UINT stoppingDistance = (velocity * velocity * 50) / (PhysicspDeceleration(train));

    return stoppingDistance + PhysicspDistanceFromPickupToFrontOfTrain(direction);
}

