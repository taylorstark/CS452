#include "physics.h"

#include <rtosc/assert.h>
#include <rtosc/string.h>
#include <user/trains.h>

static UINT g_steadyStateVelocities[MAX_TRAINS + 1][MAX_SPEED + 1]; // in micrometers per tick
static UINT g_accelerations[MAX_TRAINS + 1]; // in micrometers per tick^2

VOID
PhysicsInit
    (
        VOID
    )
{
    RtMemset(g_steadyStateVelocities, sizeof(g_steadyStateVelocities), 0);

    g_steadyStateVelocities[58][6] = 1185;
    g_steadyStateVelocities[58][7] = 1699;
    g_steadyStateVelocities[58][8] = 2200;
    g_steadyStateVelocities[58][9] = 2799;
    g_steadyStateVelocities[58][10] = 3348;
    g_steadyStateVelocities[58][11] = 4075;
    g_steadyStateVelocities[58][12] = 4780;
    g_steadyStateVelocities[58][13] = 5528;
    g_steadyStateVelocities[58][14] = 5937;

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

    RtMemset(g_accelerations, sizeof(g_accelerations), 0);
    g_accelerations[58] = 1580;
    g_accelerations[63] = 1850;
    g_accelerations[69] = 2120;
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

UINT
PhysicsAcceleration
    (
        IN UCHAR train
    )
{
    UINT acceleration = g_accelerations[train];

    if(0 == acceleration)
    {
        ASSERT(FALSE);
    }

    return acceleration;
}

INT
PhysicsCorrectAccelerationUnits
    (
        IN INT val
    )
{
    return val / 100;
}

INT
PhysicsCorrectAccelerationUnitsInverse
    (
        IN INT val
    )
{
    return val * 100;
}

UINT
PhysicsDistanceTravelled
    (
        IN UINT velocity, 
        IN UINT acceleration, 
        IN UINT accelerationTime, 
        IN UINT totalTime
    )
{
    accelerationTime = min(accelerationTime, totalTime);

    UINT distanceTravelledDueToAcceleration = 0;

    for(UINT i = 0; i < accelerationTime; i++)
    {
        for(UINT j = i; j < accelerationTime; j++)
        {
            distanceTravelledDueToAcceleration += acceleration;
        }
    }

    UINT accelerationDistance = (velocity * accelerationTime) + PhysicsCorrectAccelerationUnits(distanceTravelledDueToAcceleration);
    UINT steadyStateDistance = PhysicsEndingVelocity(velocity, acceleration, accelerationTime) * (totalTime - accelerationTime);

    return accelerationDistance + steadyStateDistance;
}

UINT
PhysicsDistanceFromPickupToFrontOfTrain
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
    UINT stoppingDistance = PhysicsCorrectAccelerationUnitsInverse(velocity * velocity) / (2 * g_accelerations[train]);

    return stoppingDistance + PhysicsDistanceFromPickupToFrontOfTrain(direction);
}

UINT
PhysicsStoppingTime
    (
        IN UCHAR train, 
        IN UINT velocity
    )
{
    return PhysicsCorrectAccelerationUnitsInverse(velocity) / g_accelerations[train];
}

UINT
PhysicsEndingVelocity
    (
        IN UINT startingVelocity, 
        IN UINT acceleration, 
        IN UINT accelerationTime
    )
{
    return startingVelocity + PhysicsCorrectAccelerationUnits(accelerationTime * acceleration);
}
