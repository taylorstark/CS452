#include "route_server.h"

#include "display.h"
#include <rtosc/assert.h>
#include <rtosc/string.h>
#include <rtkernel.h>
#include <rtos.h>
#include <track/track_data.h>
#include <user/trains.h>

#define INFINITY 0xFFFFFFFF

static
inline
UINT
RouteServerpIndex
    (
        IN TRACK_NODE* graph, 
        IN TRACK_NODE* node
    )
{
    return node - graph;
}

static
inline
UINT
RouteServerpNumNeighbours
    (
        IN TRACK_NODE* node
    )
{
    switch(node->type)
    {
        case NODE_BRANCH:
            return 2;

        case NODE_EXIT:
            return 0;

        default:
            return 1;
    }
}

static
VOID
RouteServerpSort
    (
        IN TRACK_NODE* graph, 
        IN UINT* distance, 
        IN TRACK_NODE** array, 
        IN UINT arraySize
    )
{
    // Bubble sort since I'm lazy
    BOOLEAN swapped = TRUE;

    while(swapped)
    {
        swapped = FALSE;

        for(UINT i = 1; i < arraySize; i++)
        {
            UINT currentIndex = RouteServerpIndex(graph, array[i]);
            UINT previousIndex = RouteServerpIndex(graph, array[i - 1]);

            if(distance[previousIndex] < distance[currentIndex])
            {
                TRACK_NODE* temp = array[i];
                array[i] = array[i - 1];
                array[i - 1] = temp;

                swapped = TRUE;
            }
        }

        arraySize--;
    }
}

static
INT
RouteServerpFindRoute
    (
        IN TRACK_NODE* graph, 
        IN TRACK_NODE* start, 
        IN TRACK_NODE* dest, 
        OUT PATH* path
    )
{
    UINT numUnvisitedNodes = 1;
    TRACK_NODE* unvisitedNodes[TRACK_MAX];
    unvisitedNodes[0] = start;

    UINT distance[TRACK_MAX];
    PATH_NODE previous[TRACK_MAX];

    // Initialize all costs to be infinity
    for(UINT i = 0; i < TRACK_MAX; i++)
    {
        distance[i] = INFINITY;
        previous[i].node = NULL;
        previous[i].direction = 0;
    }

    // Distance from start to start is 0
    distance[RouteServerpIndex(graph, start)] = 0;

    // Walk the graph
    while(numUnvisitedNodes > 0)
    {
        RouteServerpSort(graph, distance, unvisitedNodes, numUnvisitedNodes);
        TRACK_NODE* currentNode = unvisitedNodes[--numUnvisitedNodes];

        // If we've reached the destination then we already have the shortest path
        if(currentNode == dest)
        {
            break;
        }

        UINT numNeighbours = RouteServerpNumNeighbours(currentNode);

        for(UINT i = 0; i < numNeighbours; i++)
        {
            TRACK_EDGE* neighbourEdge = &currentNode->edge[i];
            TRACK_NODE* neighbour = neighbourEdge->dest;
            UINT neighbourIndex = RouteServerpIndex(graph, neighbour);
            UINT cost = distance[RouteServerpIndex(graph, currentNode)] + neighbourEdge->dist;

            if(cost < distance[neighbourIndex])
            {
                if(NULL == previous[neighbourIndex].node)
                {
                    unvisitedNodes[numUnvisitedNodes++] = neighbour;
                }

                distance[neighbourIndex] = cost;
                previous[neighbourIndex].node = currentNode;
                previous[neighbourIndex].direction = i;
            }
        }
    }

    if(NULL != previous[RouteServerpIndex(graph, dest)].node)
    {
        path->nodes[0].node = dest;
        path->nodes[0].direction = 0;
        path->numNodes = 1;

        PATH_NODE* iterator = &previous[RouteServerpIndex(graph, dest)];

        // Build the discovered path
        while(NULL != iterator->node)
        {
            path->nodes[path->numNodes++] = *iterator;
            iterator = &previous[RouteServerpIndex(graph, iterator->node)];
        }

        // The path is in reverse order
        for(UINT i = 0; i < path->numNodes / 2; i++)
        {
            PATH_NODE temp = path->nodes[i];
            path->nodes[i] = path->nodes[path->numNodes - i - 1];
            path->nodes[path->numNodes - i - 1] = temp;
        }

        return 0;
    }
    else
    {
        return -1;
    }
}

static
VOID
RouteServerpTask
    (
        VOID
    )
{
    /*
    PATH test;

    SENSOR e8 = { 'E', 8 };
    SENSOR c3 = { 'C', 3 };
    SENSOR c8 = { 'C', 8 };
    SENSOR c14 = { 'C', 14 };

    RouteServerpFindRoute(GetTrack(), TrackFindSensor(&c3), TrackFindSensor(&c8), &test);
    RouteServerpFindRoute(GetTrack(), TrackFindSensor(&e8), TrackFindSensor(&c8), &test);
    RouteServerpFindRoute(GetTrack(), TrackFindSensor(&e8), TrackFindSensor(&c14), &test);
    */

    PATH test;

    SENSOR e8 = { 'E', 8 };
    SENSOR c8 = { 'C', 8 };

    if(SUCCESSFUL(RouteServerpFindRoute(GetTrack(), TrackFindSensor(&e8), TrackFindSensor(&c8), &test)))
    {
        for(UINT i = 0; i < test.numNodes; i++)
        {
            if(NODE_SENSOR == test.nodes[i].node->type)
            {
                Log("%s", test.nodes[i].node->name);
            }
        }
    }
    else
    {
        Log("No path");
    }
}

VOID
RouteServerCreate
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(Create(Priority16, RouteServerpTask)));
}
