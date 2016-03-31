#pragma once

#include <rt.h>
#include <user/trains.h>

VOID
PhysicsInit
    (
        VOID
    );

UINT
PhysicsSteadyStateVelocity
    (
        IN UCHAR train,
        IN UCHAR speed
    );

UINT
PhysicsAcceleration
    (
        IN UCHAR train
    );

UINT
PhysicsStoppingDistance
    (
        IN UCHAR train, 
        IN UINT velocity, 
        IN DIRECTION direction
    );
