#include "rand.h"

VOID
RtRngInit
    (
        IN RT_RNG* rng, 
        IN INT seed
    )
{
    rng->seed = seed;
}

INT
RtRngGenerate
    (
        IN RT_RNG* rng
    )
{
    rng->seed = (1103515245 * rng->seed) + 12345;
    return rng->seed;
}
