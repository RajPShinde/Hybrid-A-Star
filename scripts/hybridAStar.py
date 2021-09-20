import math
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from heapdict import heapdict
import scipy.spatial.kdtree as kd
import CurvesGenerator.reeds_shepp as rsCurve

class Car:
    maxSteerAngle = 0.6
    steerPresion = 10
    wheelBase = 1
    wheelRadius = 0.2
    wheelWidth = 0.2

class Cost:
    reverse = 10
    directionChange = 10 
    steerAngle = 5
    steerAngleChange = 5

class Node:
    def __init__(self, gridIndex, traj, steeringAngle, direction, cost, parentGridIndex):
        self.gridIndex = gridIndex         # grid block x, y, yaw index
        self.traj = traj                   # trajectory x, y  of a simulated node
        self.steeringAngle = steeringAngle # steering angle throughout the trajectory
        self.direction = direction         # direction throughout the trajectory
        self.cost = cost                   # node cost
        self.parentGridIndex = self.parentGridIndex

class HolonomicNode:
    def __init__(self, gridIndex, cost, parentIndex):
        self.gridIndex = gridIndex
        self.cost = cost
        self.parentIndex = parentIndex

class MapParameters:
    def __init__(self, mapMinX, mapMinY, mapMaxX, mapMaxY, xyResolution, yawResolution, ObstacleKDTree):
        self.mapMinX = mapMinX               # map min x coordinate(0)
        self.mapMinY = mapMinY               # map min y coordinate(0)
        self.mapMaxX = mapMaxX               # map max x coordinate
        self.mapMaxY = mapMaxY               # map max y coordinate
        self.xyResolution = xyResolution     # grid block length
        self.yawResolution = yawResolution   # grid block possible yaws
        self.ObstacleKDTree = ObstacleKDTree # KDTree representating obstacles

def calculateMapParameters(obstacleX, obstacleY, xyResolution, yawResolution):
        
        # calculate min max map grid index based on obstacles in map
        mapMinX = round(min(obstacleX) / xyResolution)
        mapMinY = round(min(obstacleY) / xyResolution)
        mapMaxX = round(max(obstacleX) / xyResolution)
        mapMaxY = round(max(obstacleY) / xyResolution)

        # create a KDTree to represent obstacles
        ObstacleKDTree = kd.KDTree([[x, y] for x, y in zip(obstacleX, obstacleY)])

        return MapParameters(mapMinX, mapMinY, mapMaxX, mapMaxY, xyResolution, yawResolution, ObstacleKDTree)  

def index(Node):
    # Index is a tuple consisting grid index, used for checking if two nodes are near/same
    return tuple([Node.gridIndex[0], Node.gridIndex[1], Node.gridIndex[2]])

def motionCommands():
    direction = 1
    motionCommand = []
    for i in np.arange(Car.maxSteerAngle, -(Car.maxSteerAngle + Car.maxSteerAngle/Car.steerPresion), -Car.maxSteerAngle/Car.steerPresion):
        motionCommand.append([i, direction])
        motionCommand.append([i, -direction])
    return motionCommand

def holonomicMotionCommands():
    holonomicMotionCommand = [[-1, 0], [-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1]]
    return holonomicMotionCommand


def kinematicSimulationNode(currentNode, motionCommand, mapParameters, simulationLength=4, step = 0.8 ):

    # Simulate node using given current Node and Motion Commands
    traj = []
    traj.append([currentNode.traj[-1][0] + motionCommand[1] * step * math.cos(currentNode.traj[-1][2]),
                currentNode.traj[-1][1] + motionCommand[1] * step * math.sin(currentNode.traj[-1][2]),
                rsCurve.pi_2_pi(currentNode.traj[-1][2] + motionCommand[1] * step / Car.wheelBase * math.tan(motionCommand[0]))])
    for i in range(int((simulationLength/step)-1)):
        traj.append([traj[i][0] + motionCommand[1] * step * math.cos(traj[i][2]),
                    traj[i][1] + motionCommand[1] * step * math.sin(traj[i][2]),
                    rsCurve.pi_2_pi(traj[i][2] + motionCommand[1] * step / Car.wheelBase * math.tan(motionCommand[0]))])

    # Find grid index
    gridIndex = [round(traj[-1][0]/mapParameters.xyResolution), \
                 round(traj[-1][1]/mapParameters.xyResolution), \
                 round(traj[-1][2]/mapParameters.yawResolution)]

    # if not isValid(traj, gridIndex, mapParameters):
    #     return None

    # Calculate Cost of the node
    cost = simulatedPathCost(currentNode, motionCommand, simulationLength)

    return Node(gridIndex, traj, motionCommand[0], motionCommand[1], cost, index(currentNode))

def reedsSheppNode():
    return Node([], [], 0)

def analyticExpansion():
    return 0

def isValid(traj, gridIndex, mapParameters):
    if gridIndex[0]<mapParameters.mapMinX or gridIndex[0]>mapParameters.mapMaxX or \
       gridIndex[1]<mapParameters.mapMinY or gridIndex[1]>mapParameters.mapMaxY:
        return False
    if collision(traj, mapParameters):
        return False
    return True

def collision(traj, mapParameters):

    return False

def reedsSheppCost():
    return 0

def simulatedPathCost(currentNode, motionCommand, simulationLength):
    # Previos Node Cost
    cost = currentNode.cost

    # Distance cost
    if motionCommand[1] == 1:
        cost += simulationLength 
    else:
        cost += simulationLength * Cost.reverse

    # Direction change cost
    if currentNode.direction != motionCommand[1]:
        cost += Cost.directionChange

    # Steering Angle Cost
    cost += motionCommand[0] * Cost.steerAngle

    # Steering Angle change cost
    cost += abs(motionCommand[0] - currentNode.steeringAngle) * Cost.steerAngleChange

    return cost

def cost(holonomicMotionCommand):
    return math.hypot(motionCommand[0], motionCommand[1])

def holonomicNodeIndex(HolonomicNode):
    return tuple(HolonomicNode.GridIndex[0], HolonomicNode.GridIndex[1])

def obstaclesMap(obstacleX, obstacleY):
    # Set all Grid locations to No Obstacle
    obstacles =[[False for i in range(max(obstacleY))]for i in range(max(obstacleX))]

    # Set Grid Locations with obstacles to True
    for x in range():
        for y in range():
            for i, j in zip(obstacleX, obstacleY):
                if math.hypot(i-obstacleX, j-obstacleY) < = 1:
                    obstacles[i][j] == True
                    break

    return obstacles

def holonomicCostsWithObstacles(goalNode, mapParameters):

    GridIndex = [round(gNode.traj[-1][0]/mapParameters.xyResolution), round(gNode.traj[-1][1]/mapParameters.xyResolution)]
    gNode =HolonomicNode(GridIndex, 0, tupe(GridIndex))

    obstacles = obstaclesMap(mapParameters.obstacleX, mapParameters.obstaclesY)

    holonomicMotionCommand = holonomicMotionCommands()

    openSet = {holonomicNodeIndex(gNode): gNode}
    closedSet = {}

    priorityQueue =[]
    heapq.heappush(priorityQueue, (gNode.cost, holonomicNodeIndex(gNode)))

    while(True):
        if not openSet:
            break

        currentNodeCost, currentNodeIndex = heapq.heappop(priorityQueue)
        currentNode = openSet[currentNodeIndex]
        openSet.pop(currentNodeIndex)
        closedSet[currentNodeIndex] = currentNode

        for i in range(len(holonomicMotionCommand)):
            neighbourNode = Holonomic(currentNode.GridIndex[0] + holonomicMotionCommand[i][0],\
                                      currentNode.GridIndex[1] + holonomicMotionCommand[i][1],\
                                      currentNode.cost + cost(holonomicMotionCommand[i]), currentNodeIndex)

            if not holonomicNodeIsValid(neighbourNode):
                continue

            neighbourNodeIndex = holonomicNodeIndex(neighbourNode)

            if neighbourNode not in closedSet:
                if neighbourNode in openSet:
                    if neighbourNode.cost < openSet(neighbourNodeIndex).cost:
                        openSet[neighbourNodeIndex] = neighbourNode
                        heapq.heappush(priorityQueue, (neighbourNode.cost, neighbourNodeIndex))
                else:
                    openSet[neighbourNodeIndex] = neighbourNode
                    heapq.heappush(priorityQueue, (neighbourNode.cost, neighbourNodeIndex))

    holonomicCost = [[np.inf for i in range(max(obstacleY))]for i in range(max(obstacleX))]

    for nodes in closedSet.values():
        holonomicCost[nodes.x][nodes.y]=nodes.cost

    return holonomicCost

def map():
    # Build Map
    obstacleX, obstacleY = [], []

    for i in range(50):
        obstacleX.append(i)
        obstacleY.append(0)

    for i in range(50):
        obstacleX.append(0)
        obstacleY.append(i)

    return obstacleX, obstacleY

def drawFootprint(path, plot):
    return 0

def run(s, g, mapParameters, plt):
    sGridIndex = [round(s[0] / mapParameters.xyResolution), \
                  round(s[1] / mapParameters.xyResolution), \
                  round(s[2]/mapParameters.yawResolution)]
    gGridIndex = [round(g[0] / mapParameters.xyResolution), \
                  round(g[1] / mapParameters.xyResolution), \
                  round(g[2]/mapParameters.yawResolution)]

    # Generate all Possible motion commands to car
    motionCommand = motionCommands()

    # Create start and end Node
    startNode = Node(sGridIndex, [s], 0, 1, 0 , sGridIndex)
    goalNode = Node(gGridIndex, [g], 0, 1, 0, gGridIndex)

    # Find Holonomic Heuristric
    holonomicHeuristics = calculateholonomicHeuristics(goalNode, mapParameters)

    # Add start node to open Set
    openSet = {index(startNode):startNode}
    closedSet = {}

    # Create a priority queue for acquiring nodes based on their cost's
    costQueue = heapdict()

    # Add start mode into priority queue
    costQueue[index(startNode)] = 0
    counter = 0

    # Run loop while path is found or open set is empty
    while(True):
        counter +=1
        # Check if openSet is empty, if empty no solution available
        if not openSet:
            return none

        # Get first node in the priority queue
        currentNodeIndex = costQueue.popitem()[0]
        currentNode = openSet[currentNodeIndex]

        # Revove currentNode from openSet and add it to closedSet
        openSet.pop(currentNodeIndex)
        closedSet[currentNodeIndex] = currentNode

        # Get Reed-Shepp Node if available
        # reedSheppNode = analyticExpansion(currentNode, goalNode)

        # if not reedSheppNode:

        if currentNodeIndex == index(goalNode):
            print("Path Found")
            print(currentNode.traj[-1])
            plt.show()

        # Get all simulated Nodes from current node
        for i in range(len(motionCommand)):
            simulatedNode = kinematicSimulationNode(currentNode, motionCommand[i], mapParameters)

            # Check if path is within map bounds and is collision free
            if not simulatedNode:
                continue

            # Draw Simulated Node
            x,y,z =zip(*simulatedNode.traj)
            plt.plot(x, y)

            # Check if simulated node is already in closed set
            simulatedNodeIndex = index(simulatedNode)
            if simulatedNodeIndex not in closedSet: 

                # Check if simulated node is already in open set, if not add it open set as well as in priority queue
                if simulatedNodeIndex not in openSet:
                    openSet[simulatedNodeIndex] = simulatedNode
                    costQueue[simulatedNodeIndex] = simulatedNode.cost
                else:
                    if simulatedNode.cost < openSet[simulatedNodeIndex].cost:
                        openSet[simulatedNodeIndex] = simulatedNode
                        costQueue[simulatedNodeIndex] = simulatedNode.cost
    
    plt.show()

    # Backtrack

    # Draw Path
    # plt.plot(path[0], path[1], linewidth=1.5, color='b')

    # Draw Car Footprint
    # drawFootprint(path, plot)

def main():
    # Set Start, Goal x, y, theta
    s = [10, 10, np.deg2rad(45)] 
    g = [15, 15, np.deg2rad(90)]

    # Get Obstacle Map
    obstacleX, obstacleY = map()

    # Calculate map Paramaters
    mapParameters = calculateMapParameters(obstacleX, obstacleY, 2, 0.6)

    # Draw Map
    plt.xlim(min(obstacleX), max(obstacleX)) 
    plt.ylim(min(obstacleY), max(obstacleY))
    plt.plot(obstacleX, obstacleY, "sk")

    # Run Hybrid A*
    run(s, g, mapParameters, plt)

if __name__ == '__main__':
    main()
