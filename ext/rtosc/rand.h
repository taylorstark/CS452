#pragma once

#include <rt.h>

typedef struct _RT_RNG
{
    INT seed;
} RT_RNG;

VOID
RtRngInit
    (
        IN RT_RNG* rng, 
        IN INT seed
    );

INT
RtRngGenerate
    (
        IN RT_RNG* rng
    );
