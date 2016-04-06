#include "route_server.h"

#include "display.h"
#include "physics.h"
#include <rtosc/assert.h>
#include <rtosc/buffer.h>
#include <rtosc/string.h>
#include <rtkernel.h>
#include <rtos.h>
#include <track/track_data.h>
#include <user/trains.h>

#define ROUTE_SERVER_NAME "route"
#define INFINITY 0xFFFFFFFF
#define ROUTE_SERVER_MINIMUM_VELOCITY_TO_BE_CONFIDENT_IN_POSITION 3000
#define ROUTE_SERVER_BLOCKING_DISTANCE 200 // 20 cm
#define ROUTE_SERVER_ALLOWABLE_OVERLAP 100 // 1 second

typedef enum _ROUTE_REQUEST_TYPE
{
    LocationUpdateRequest = 0, 
    DirectionUpdateRequest, 
    SetDestinationRequest, 
    ClearDestinationRequest,
    AwaitRouteRequest
} ROUTE_REQUEST_TYPE;

typedef struct _ROUTE_TO_DESTINATION_REQUEST
{
    UCHAR train;
    LOCATION destination;
} ROUTE_TO_DESTINATION_REQUEST;

typedef struct _ROUTE_REQUEST
{
    ROUTE_REQUEST_TYPE type;

    union
    {
        UCHAR train;
        TRAIN_LOCATION trainLocation;
        TRAIN_DIRECTION trainDirection;
        ROUTE_TO_DESTINATION_REQUEST routeToDestination;
    };
} ROUTE_REQUEST;

typedef struct _ROUTE_DATA
{
    UCHAR train;
    LOCATION destination;
    TRAIN_LOCATION currentLocation;
    PATH path;
} ROUTE_DATA;

static
VOID
RouteServerpLocationNotifierTask
    (
        VOID
    )
{
    INT routeServerId = MyParentTid();
    ASSERT(SUCCESSFUL(routeServerId));

    ROUTE_REQUEST request;
    request.type = LocationUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(LocationAwait(&request.trainLocation)));
        VERIFY(SUCCESSFUL(Send(routeServerId, &request, sizeof(request), NULL, 0)));
    }
}

static
VOID
RouteServerpDirectionChangeNotifierTask
    (
        VOID
    )
{
    INT routeServerId = MyParentTid();
    ASSERT(SUCCESSFUL(routeServerId));

    ROUTE_REQUEST request;
    request.type = DirectionUpdateRequest;

    while(1)
    {
        VERIFY(SUCCESSFUL(TrainDirectionChangeAwait(&request.trainDirection)));
        VERIFY(SUCCESSFUL(Send(routeServerId, &request, sizeof(request), NULL, 0)));
    }
}

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
        IN TRAIN_LOCATION* trainLocation, 
        IN BOOLEAN* blockedNodes, 
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
                if(!blockedNodes[neighbourIndex] && NULL == previous[neighbourIndex].node)
                {
                    unvisitedNodes[numUnvisitedNodes++] = neighbour;
                }

                distance[neighbourIndex] = cost;
                previous[neighbourIndex].node = currentNode;
                previous[neighbourIndex].direction = i;

                if(trainLocation->velocity > 0 && cost * 1000 > trainLocation->location.distancePastNode)
                {
                    previous[neighbourIndex].expectedArrivalTime = (cost * 1000 - trainLocation->location.distancePastNode) / trainLocation->velocity;
                }
                else
                {
                    previous[neighbourIndex].expectedArrivalTime = 0;
                }
            }
        }
    }

    UINT destIndex = RouteServerpIndex(graph, dest);

    if(NULL != previous[destIndex].node)
    {
        path->numNodes = 1;
        path->totalDistance = distance[destIndex] * 1000; // need to perform unit conversion
        path->nodes[0].node = dest;
        path->nodes[0].direction = 0;
        path->nodes[0].expectedArrivalTime = trainLocation->velocity > 0 ? path->totalDistance / trainLocation->velocity : 0;        

        PATH_NODE* iterator = &previous[destIndex];

        // Build the discovered path
        while(NULL != iterator->node)
        {
            path->nodes[path->numNodes++] = *iterator;
            iterator = &previous[RouteServerpIndex(graph, iterator->node)];
        }

        // The path is in reverse order
        for(UINT i = 0; i < path->numNodes / 2; i++)
        {
            PATH_NODE temp;
            RtMemcpy(&temp, &path->nodes[i], sizeof(temp));
            RtMemcpy(&path->nodes[i], &path->nodes[path->numNodes - i - 1], sizeof(path->nodes[i]));
            RtMemcpy(&path->nodes[path->numNodes - i - 1], &temp, sizeof(&path->nodes[path->numNodes - i - 1]));
        }

        return 0;
    }
    else if(start == dest)
    {
        path->nodes[0].node = dest;
        path->nodes[0].direction = 0;
        path->nodes[0].expectedArrivalTime = 0;
        path->numNodes = 1;
        path->totalDistance = 0;

        return 0;
    }
    else
    {
        return -1;
    }
}

static
VOID
RouteServerpBlockNodes
    (
        IN TRACK_NODE* graph, 
        IN TRACK_NODE* start, 
        IN UINT distance, 
        OUT BOOLEAN* blockedNodes
    )
{
    // Block the start node
    blockedNodes[RouteServerpIndex(graph, start)] = TRUE;
    blockedNodes[RouteServerpIndex(graph, start->reverse)] = TRUE;

    // Block any nodes near the start node
    TRACK_EDGE* nextEdge = TrackNextEdge(start);
    while(distance > nextEdge->dist)
    {
        blockedNodes[RouteServerpIndex(graph, nextEdge->dest)] = TRUE;
        blockedNodes[RouteServerpIndex(graph, nextEdge->dest->reverse)] = TRUE;
        distance -= nextEdge->dist;
        nextEdge = TrackNextEdge(nextEdge->dest);
    }
}

static
VOID
RouteServerpCalculateBlockedNodes
    (
        IN TRACK_NODE* graph, 
        IN UCHAR thisTrain, 
        IN ROUTE_DATA* trackedTrains, 
        IN UINT numTrackedTrains, 
        OUT BOOLEAN* blockedNodes
    )
{
    for(UINT i = 0; i < numTrackedTrains; i++)
    {
        if(trackedTrains[i].train != thisTrain && 0 == trackedTrains[i].currentLocation.velocity)
        {
            RouteServerpBlockNodes(graph, 
                                   trackedTrains[i].currentLocation.location.node, 
                                   ROUTE_SERVER_BLOCKING_DISTANCE + (trackedTrains[i].currentLocation.location.distancePastNode / 1000), // unit conversion 
                                   blockedNodes);
            RouteServerpBlockNodes(graph, 
                                   trackedTrains[i].currentLocation.location.node->reverse, 
                                   ROUTE_SERVER_BLOCKING_DISTANCE, 
                                   blockedNodes);
        }
    }
}

static
UINT
RouteServerpCalculateDirectionTaken
    (
        IN TRACK_NODE* node
    )
{
    if(NODE_BRANCH == node->type)
    {
        SWITCH_DIRECTION direction;
        VERIFY(SUCCESSFUL(SwitchGetDirection(node->num, &direction)));

        if(SwitchStraight == direction)
        {
            return DIR_STRAIGHT;
        }
        else
        {
            return DIR_CURVED;
        }
    }
    else
    {
        return DIR_AHEAD;
    }
}

static
BOOLEAN
RouteServerpFixupReversePath
    (
        IN TRACK_NODE* graph, 
        IN TRAIN_LOCATION* currentLocation, 
        IN DIRECTION direction, 
        IN UINT velocity, 
        IN BOOLEAN* blockedNodes, 
        OUT PATH* reversePath
    )
{
    PATH fixedPath;
    fixedPath.nodes[0].node = currentLocation->location.node;
    fixedPath.nodes[0].direction = RouteServerpCalculateDirectionTaken(fixedPath.nodes[0].node);
    fixedPath.numNodes = 1;
    fixedPath.totalDistance = 0;

    // Compute how long it would take to stop and perform a reverse
    UINT endingVelocity = PhysicsEndingVelocity(currentLocation->velocity, 
                                                currentLocation->acceleration,
                                                min(currentLocation->accelerationTicks, AVERAGE_TRAIN_COMMAND_LATENCY));
    INT stoppingDistance = PhysicsStoppingDistance(currentLocation->train, endingVelocity, direction);

    // Compute the nodes travelled while performing a stop
    TRACK_NODE* iterator = currentLocation->location.node;

    while(NODE_EXIT != iterator->type && stoppingDistance > 0)
    {
        if(blockedNodes[RouteServerpIndex(graph, iterator)])
        {
            return FALSE;
        }

        UINT direction = RouteServerpCalculateDirectionTaken(iterator);
        TRACK_EDGE* edgeTaken = &iterator->edge[direction];
        UINT distanceTaken = edgeTaken->dist * 1000; // Perform unit conversions

        stoppingDistance -= distanceTaken;
        iterator = edgeTaken->dest;

        fixedPath.nodes[fixedPath.numNodes].node = iterator;
        fixedPath.nodes[fixedPath.numNodes].direction = direction;
        fixedPath.totalDistance += distanceTaken; // must increment before calculating arrival time
        fixedPath.nodes[fixedPath.numNodes].expectedArrivalTime = velocity > 0 ? fixedPath.totalDistance / velocity : 0;
        fixedPath.numNodes++;
        
    }

    // Compute the nodes travelled while performing the reverse
    fixedPath.totalDistance *= 2;

    for(INT i = fixedPath.numNodes - 2; i > 0; i--)
    {
        TRACK_NODE* reversedNode = fixedPath.nodes[i].node->reverse;
        UINT direction = RouteServerpCalculateDirectionTaken(reversedNode);

        fixedPath.nodes[fixedPath.numNodes].node = reversedNode;
        fixedPath.nodes[fixedPath.numNodes].direction = direction;
        fixedPath.nodes[fixedPath.numNodes].expectedArrivalTime = velocity > 0 ? (reversedNode->edge[direction].dist * 1000) / velocity : 0;
        fixedPath.numNodes++;
    }

    // Copy over the rest of the path
    fixedPath.totalDistance += reversePath->totalDistance;
    UINT timeOffset = fixedPath.numNodes > 0 ? fixedPath.nodes[fixedPath.numNodes - 1].expectedArrivalTime : 0;

    for(UINT i = 0; i < reversePath->numNodes; i++)
    {
        fixedPath.nodes[fixedPath.numNodes].node = reversePath->nodes[i].node;
        fixedPath.nodes[fixedPath.numNodes].direction = reversePath->nodes[i].direction;
        fixedPath.nodes[fixedPath.numNodes].expectedArrivalTime = reversePath->nodes[i].expectedArrivalTime + timeOffset;
        fixedPath.numNodes++;
    }

    // And now copy the fixed up path to the original path
    RtMemcpy(reversePath, &fixedPath, sizeof(fixedPath));
    return TRUE;
}

static
TRACK_NODE*
RouteServerpFindCollision
    (
        IN PATH* p1, 
        IN PATH* p2
    )
{
    UINT minNumNodes = min(p1->numNodes, p2->numNodes);

    for(UINT i = 0; i < minNumNodes; i++)
    {
        if(p1->nodes[i].node == p2->nodes[i].node || p1->nodes[i].node == p2->nodes[i].node->reverse)
        {
            INT diff = ((INT) p1->nodes[i].expectedArrivalTime) - ((INT) p2->nodes[i].expectedArrivalTime);

            if(abs(diff) < ROUTE_SERVER_ALLOWABLE_OVERLAP)
            {
                return p1->nodes[i].node;
            }
        }
    }

    return NULL;
}

static
TRACK_NODE*
RouteServerpFindFirstCollision
    (
        IN PATH* path, 
        IN UCHAR thisTrain, 
        IN ROUTE_DATA* trackedTrains, 
        IN UINT numTrackedTrains
    )
{
    for(UINT i = 0; i < numTrackedTrains; i++)
    {
        if(trackedTrains[i].train != thisTrain)
        {
            TRACK_NODE* collision = RouteServerpFindCollision(path, &trackedTrains[i].path);

            if(NULL != collision)
            {
                return collision;
            }
        }
    }

    return NULL;
}

static
PATH*
RouteServerpSelectOptimalPath
    (
        IN TRACK_NODE* graph, 
        IN ROUTE_DATA* trackedTrains, 
        IN UINT numTrackedTrains,
        IN TRAIN_LOCATION* currentLocation, 
        IN TRACK_NODE* dest, 
        IN DIRECTION direction, 
        IN PATH* forwardPath, 
        IN PATH* reversePath
    )
{
    BOOLEAN blockedNodes[TRACK_MAX];
    RtMemset(blockedNodes, sizeof(blockedNodes), FALSE);
    RouteServerpCalculateBlockedNodes(graph, currentLocation->train, trackedTrains, numTrackedTrains, blockedNodes);

    for(UINT i = 0; i < 3; i++)
    {
        RtMemset(forwardPath, sizeof(*forwardPath), 0);
        RtMemset(reversePath, sizeof(*reversePath), 0);
        
        BOOLEAN hasForwardPath = SUCCESSFUL(RouteServerpFindRoute(graph, currentLocation->location.node, dest, currentLocation, blockedNodes, forwardPath));
        BOOLEAN hasReversePath = FALSE;

        if(0 == currentLocation->velocity || currentLocation->velocity > ROUTE_SERVER_MINIMUM_VELOCITY_TO_BE_CONFIDENT_IN_POSITION)
        {
            hasReversePath = SUCCESSFUL(RouteServerpFindRoute(graph, currentLocation->location.node->reverse, dest, currentLocation, blockedNodes, reversePath));

            if(hasReversePath && reversePath->numNodes > 0)
            {
                hasReversePath = RouteServerpFixupReversePath(graph, currentLocation, direction, currentLocation->velocity, blockedNodes, reversePath);
            }
        }

        forwardPath->performsReverse = FALSE;
        reversePath->performsReverse = TRUE;

        PATH* selectedPath = NULL;

        if(hasForwardPath && hasReversePath)
        {
            UINT reversePathWeight = reversePath->totalDistance + (currentLocation->velocity * 400);

            // Compare the cost of going forward against the cost of going in reverse
            if(reversePathWeight < forwardPath->totalDistance)
            {
                selectedPath = reversePath;
            }
            else
            {
                selectedPath = forwardPath;
            }
        }
        else if(hasForwardPath)
        {
            selectedPath = forwardPath;
        }
        else if(hasReversePath)
        {
            selectedPath = reversePath;
        }

        if(NULL != selectedPath)
        {
            TRACK_NODE* firstCollision = RouteServerpFindFirstCollision(selectedPath, currentLocation->train, trackedTrains, numTrackedTrains);

            if(NULL != firstCollision)
            {
                blockedNodes[RouteServerpIndex(graph, firstCollision)] = TRUE;
                blockedNodes[RouteServerpIndex(graph, firstCollision->reverse)] = TRUE;
                Log("Col %s", firstCollision->name);
            }
            else
            {
                return selectedPath;
            }
        }
        else
        {
            return NULL;
        }
    }

    return NULL;
}

static
ROUTE_DATA*
RouteServerpFindTrainById
    (
        IN ROUTE_DATA* trackedTrains, 
        IN UINT numTrackedTrains, 
        IN UCHAR train
    )
{
    for(UINT i = 0; i < numTrackedTrains; i++)
    {
        if(trackedTrains[i].train == train)
        {
            return &trackedTrains[i];
        }
    }

    return NULL;
}

static
BOOLEAN
RouteServerpHasDestination
    (
        IN ROUTE_DATA* train
    )
{
    return NULL != train->destination.node;
}

static
VOID
RouteServerpTask
    (
        VOID
    )
{
    TRACK_NODE* graph = GetTrack();

    UINT numTrackedTrains = 0;
    ROUTE_DATA trackedTrains[MAX_TRACKABLE_TRAINS];
    RtMemset(trackedTrains, sizeof(trackedTrains), 0);

    DIRECTION directions[MAX_TRAINS];
    RtMemset(directions, sizeof(directions), DirectionForward);

    INT underlyingAwaitingTasksBuffer[NUM_TASKS];
    RT_CIRCULAR_BUFFER awaitingTasks;
    RtCircularBufferInit(&awaitingTasks, underlyingAwaitingTasksBuffer, sizeof(underlyingAwaitingTasksBuffer));

    VERIFY(SUCCESSFUL(RegisterAs(ROUTE_SERVER_NAME)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, RouteServerpLocationNotifierTask)));
    VERIFY(SUCCESSFUL(Create(HighestUserPriority, RouteServerpDirectionChangeNotifierTask)));

    while(1)
    {
        INT senderId;
        ROUTE_REQUEST request;

        VERIFY(SUCCESSFUL(Receive(&senderId, &request, sizeof(request))));

        switch(request.type)
        {
            case LocationUpdateRequest:
            {
                // Reply to the notifier ASAP
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));

                // See if this train has a destination
                ROUTE_DATA* trainData = RouteServerpFindTrainById(trackedTrains, numTrackedTrains, request.trainLocation.train);

                if(NULL != trainData)
                {
                    trainData->currentLocation = request.trainLocation;

                    if(RouteServerpHasDestination(trainData))
                    {
                        // Clear the old path
                        RtMemset(&trainData->path, sizeof(trainData->path), 0);

                        // Find a new path
                        DIRECTION direction = directions[request.trainLocation.train];
                        PATH forwardPath;
                        PATH reversePath;

                        PATH* optimalPath = RouteServerpSelectOptimalPath(graph, 
                                                                          trackedTrains, 
                                                                          numTrackedTrains, 
                                                                          &request.trainLocation, 
                                                                          trainData->destination.node, 
                                                                          direction, 
                                                                          &forwardPath, 
                                                                          &reversePath);

                        if(NULL == optimalPath)
                        {
                            forwardPath.numNodes = 0;
                            forwardPath.totalDistance = 0;
                            forwardPath.performsReverse = FALSE;

                            optimalPath = &forwardPath;
                        }

                        // Remember the path
                        RtMemcpy(&trainData->path, optimalPath, sizeof(trainData->path));

                        // Send the path to any registrants
                        ROUTE route;
                        RtMemcpy(&route.trainLocation, &trainData->currentLocation, sizeof(route.trainLocation));
                        RtMemcpy(&route.path, &trainData->path, sizeof(route.path));

                        INT awaitingTask;
                        while(!RtCircularBufferIsEmpty(&awaitingTasks))
                        {
                            VERIFY(RT_SUCCESS(RtCircularBufferPeekAndPop(&awaitingTasks, &awaitingTask, sizeof(awaitingTask))));
                            VERIFY(SUCCESSFUL(Reply(awaitingTask, &route, sizeof(route))));
                        }
                    }
                }

                break;
            }

            case DirectionUpdateRequest:
            {
                directions[request.trainDirection.train] = request.trainDirection.direction;
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));
                break;
            }

            case SetDestinationRequest:
            {
                ROUTE_DATA* trainData = RouteServerpFindTrainById(trackedTrains, numTrackedTrains, request.routeToDestination.train);

                if(NULL == trainData)
                {
                    trainData = &trackedTrains[numTrackedTrains++];
                    trainData->train = request.routeToDestination.train;
                }

                trainData->destination = request.routeToDestination.destination;
                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));
                break;
            }

            case ClearDestinationRequest:
            {
                ROUTE_DATA* trainData = RouteServerpFindTrainById(trackedTrains, numTrackedTrains, request.train);

                if(NULL != trainData)
                {
                    // Clear the train's destination
                    trainData->destination.node = NULL;
                    trainData->destination.distancePastNode = 0;

                    // Clear the train's path
                    RtMemset(&trainData->path, sizeof(trainData->path), 0);
                }

                VERIFY(SUCCESSFUL(Reply(senderId, NULL, 0)));
                break;
            }

            case AwaitRouteRequest:
            {
                VERIFY(RT_SUCCESS(RtCircularBufferPush(&awaitingTasks, &senderId, sizeof(senderId))));
                break;
            }

            default:
            {
                ASSERT(FALSE);
                break;
            }
        }
    }
}

VOID
RouteServerCreate
    (
        VOID
    )
{
    VERIFY(SUCCESSFUL(Create(Priority20, RouteServerpTask)));
}

INT
RouteTrainToDestination
    (
        IN UCHAR train, 
        IN LOCATION* destination
    )
{
    INT result = WhoIs(ROUTE_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT routeServerId = result;

        ROUTE_REQUEST request;
        request.type = SetDestinationRequest;
        request.routeToDestination.train = train;
        request.routeToDestination.destination = *destination;

        result = Send(routeServerId, &request, sizeof(request), NULL, 0);
    }

    return result;
}

INT
RouteClearDestination
    (
        IN UCHAR train
    )
{
    INT result = WhoIs(ROUTE_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT routeServerId = result;

        ROUTE_REQUEST request;
        request.type = ClearDestinationRequest;
        request.train = train;

        result = Send(routeServerId, &request, sizeof(request), NULL, 0);
    }

    return result;
}

INT
RouteAwait
    (
        OUT ROUTE* route
    )
{
    INT result = WhoIs(ROUTE_SERVER_NAME);

    if(SUCCESSFUL(result))
    {
        INT routeServerId = result;

        ROUTE_REQUEST request;
        request.type = AwaitRouteRequest;

        result = Send(routeServerId, &request, sizeof(request), route, sizeof(*route));
    }

    return result;
}
