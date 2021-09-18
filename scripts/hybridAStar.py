import math
import numpy as np
import matplotlib.pyplot as plot
import heapq
from heapdict import heapdict
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../CurvesGenerator/")
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
    def __int__(self, gridIndex, traj, steeringAngle, direction, cost):
        self.gridIndex = gridIndex
        self.traj = traj
        self.steeringAngle = steeringAngle
        self.direction = direction
        self.cost = cost

class MapParamaters:
    def __int__(self, mapMinX, mapMinY, mapMaxX, mapMaxY, xyResolution, yawResolution, ObstacleKDTree):
        self.mapMinX = mapX
        self.mapMinY = mapY
        self.mapMaxX = mapMaxX
        self.mapMaxY = mapMaxX
        self.xyResolution = xResolution # grid block length
        self.yawResolution = yawResolution # grid block possible yaws
        self.ObstacleKDTree = ObstacleKDTree

def calculateMapParameters(obstacleX, obstacleY, xyResolution, yawResolution)
        
        # min max map grid index
        mapMinX = round(min(obstacleX) / xyResolution)
        mapMinY = round(min(obstacleY) / xyResolution)
        mapMaxX = round(max(obstacleX) / xyResolution)
        mapMaxX = round(max(obstacleY) / xyResolution)

        ObstacleKDTree = kd.KDTree([[x, y] for x, y in zip(obstacleX, obstacleY)])

        return MapParamaters(mapMinX, mapMinY, mapMaxX, mapMaxY, xyResolution, yawResolution, ObstacleKDTree)  

def index(Node):

    return 0

def motionCommands():
    direction = 1
    motionCommands = []
    for i in np.arange(Car.maxSteerAngle, -(Car.maxSteerAngle + Car.maxSteerAngle/Car.steerPresion), -Car.maxSteerAngle/Car.steerPresion):
        motionCommands.append([i, direction])
        motionCommands.append([i, -direction])
    return motionCommands

def kinematicSimulationNode(currentNode, motionCommands):

    # Simulate node using given current Node and Motion Commands

    # Calculate Cost of the node


    gridIndex =
    node = Node(gridIndex, traj, motionCommands[0], motionCommands[1], cost)
    return node

def reedsSheppNode():
    return Node([], [], 0)

def analyticExpansion():
    return 0

def isValid():
    return True

def reedsSheppCost():
    return 0

def simulatedPathCost(currentNode, simulatedPath, motionCommands):
    cost = currentNode.cost
    if currentNode.direction!=motionCommands[1]:
        cost = cost + Cost.directionChange

    if motionCommands[1]==-1:
        cost = cost + Cost.reverse

    cost = cost + ((currentNode.steeringAngle - motionCommands[0]) * Cost.steerAngleChange) + motionCommands[0] * Cost.steerAngle

    return cost

def map():
    return 0

def drawFootprint(path, plot):
    return 0

def run(s, g, plot):

    # Generate all Possible motion commands to car
    motionCommands = motionCommands()

    # Create start and end Node
    startNode = Node([s], 1, 0)
    goalNode = Node([g], 1, 0)

    # Add start node to open Set
    openSet = {index(startNode):startNode}
    closedSet = {}

    # Create a priority queue for acquiring nodes based on their cost's
    costQueue = heapdict.heapdict()

    # Add start mode into priority queue
    costQueue[index(startNode)] = 0

    # Run loop while path is found or open set is empty
    while(true):
        if openSet.empty():
            return none

        # Get first node in the priority queue
        currentNode = 

        # Get Reed-Shepp Node if available
        reedSheppNode = analyticExpansion(currentNode, goalNode)

        if not reedSheppNode:


        # Get all simulated Nodes from current node
        for i in range(len(motionCommands)):
            simulatedPath= kinematicSimulationNode(currentNode, motionCommands[i])

            # Check if path is within map bounds and is collision free
            if not isValid(simulatedPath)
                continue

            simulatedPathCost = simulatedPathCost(currentNode, simulatedPath, motionCommands[i])

            simulatedNode = Node(path, motionCommands[i][1], simulatedPathCost)

            # Check if simulated node is already in closed set
            if index(simulatedNode) in closedSet: 
                continue

            # Check if simulated node is already in open set, if not add it open set as well as in priority queue
            if index(simulatedNode) not in openSet:
                openSet[index(simulatedNode)] = simulatedNode
                put
            else:
                if simulatedNode.cost < openSet[index(simulatedNode)].cost:
                    openSet[index(simulatedNode)] = simulatedNode
                    put


def main():
    # Set Start, Goal x, y, theta
    s = [10, 10, np,deg2rad(90)] 
    g = [90, 90, np.deg2rad(90)]

    # Draw Map

    # Calculate map Paramaters
    mapParamaters = calculateMapParameters(obstacleX, obstacleY)

    # Run Hybrid A*
    path, plot = run(s, g, mapParamaters, plot)

    # Draw Car Footprint
    drawFootprint(path, plot)

if __name__ == '__main__':
    main()
