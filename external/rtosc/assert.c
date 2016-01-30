#include "assert.h"

#include <bwio/bwio.h>

VOID
assert
    (
        BOOLEAN expr,
        STRING msg,
        INT line,
        STRING file
    )
{
    if(expr)
    {
        return;
    }

    bwprintf(BWCOM2, "Assert triggered on line %d of file %s \r\n", line, file);
    bwputstr(BWCOM2, msg);
    bwputstr(BWCOM2, "\r\n");

    while(1) { }
}
