import math
import numpy as np
import matplotlib.pyplot as plot
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
        self.gridIndex = gridIndex         # grid block x, y, yaw index
        self.traj = traj                   # trajectory x, y  of a simulated node  
        self.steeringAngle = steeringAngle # steering angle throughout the trajectory
        self.direction = direction         # direction throughout the trajectory
        self.cost = cost                   # node cost

class MapParamaters:
    def __int__(self, mapMinX, mapMinY, mapMaxX, mapMaxY, xyResolution, yawResolution, ObstacleKDTree):
        self.mapMinX = mapX                  # map min x coordinate(0)
        self.mapMinY = mapY                  # map min y coordinate(0)
        self.mapMaxX = mapMaxX               # map max x coordinate
        self.mapMaxY = mapMaxX               # map max y coordinate
        self.xyResolution = xResolution      # grid block length
        self.yawResolution = yawResolution   # grid block possible yaws
        self.ObstacleKDTree = ObstacleKDTree # KDTree representating obstacles

def calculateMapParameters(obstacleX, obstacleY, xyResolution, yawResolution):
        
        # calculate min max map grid index based on obstacles in map
        mapMinX = round(min(obstacleX) / xyResolution)
        mapMinY = round(min(obstacleY) / xyResolution)
        mapMaxX = round(max(obstacleX) / xyResolution)
        mapMaxX = round(max(obstacleY) / xyResolution)

        # create a KDTree to represent obstacles
        ObstacleKDTree = kd.KDTree([[x, y] for x, y in zip(obstacleX, obstacleY)])

        return MapParamaters(mapMinX, mapMinY, mapMaxX, mapMaxY, xyResolution, yawResolution, ObstacleKDTree)  

def index(Node):
    # Index is a tuple consisting grid index, used for checking if two nodes are near/same
    return tuple([Node.gridIndex[0], Node.gridIndex[1], Node.gridIndex[2]])

def motionCommands():
    direction = 1
    motionCommands = []
    for i in np.arange(Car.maxSteerAngle, -(Car.maxSteerAngle + Car.maxSteerAngle/Car.steerPresion), -Car.maxSteerAngle/Car.steerPresion):
        motionCommands.append([i, direction])
        motionCommands.append([i, -direction])
    return motionCommands

def kinematicSimulationNode(currentNode, motionCommands, simulationLength=4, step = 0.4 ):

    # Simulate node using given current Node and Motion Commands
    traj.append(currentNode.traj[-1][0] + motionCommands[1] * step * math.cos(currentNode.traj[-1][2]),
                currentNode.traj[-1][1] + motionCommands[1] * step * math.sin(currentNode.traj[-1][2]),
                rsCurve.pi_2_pi(currentNode.traj[-1][2] + motionCommands[1] * step / Car.wheelBase * math.tan(motionCommands[0])))
    for i in range((simulationLength/step)-1):
        traj.append(traj[i][0] + motionCommands[1] * step * math.cos(traj[i][2]),
                    traj[i][1] + motionCommands[1] * step * math.sin(traj[i][2]),
                    rsCurve.pi_2_pi(traj[i][2] + motionCommands[1] * step / Car.wheelBase * math.tan(motionCommands[0])))

    # Find grid index
    gridIndex[0] = round(traj[-1][0]/mapParamaters.xyResolution)
    gridIndex[1] = round(traj[-1][1]/mapParamaters.xyResolution)
    gridIndex[2] = round(traj[-1][2]/mapParamaters.yawResolution)

    # Calculate Cost of the node
    cost = simulatedPathCost(currentNode, motionCommands, simulationLength)

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
    # Previos Node Cost
    cost = currentNode.cost

    # Distance cost
    if direction == 1:
        cost += simulationLength 
    else:
        cost += simulationLength * Cost.reverse

    # Direction change cost
    if currentNode.direction != motionCommands[1]
        cost += Cost.directionChange

    # Steering Angle Cost
    cost += motionCommands[0] * Cost.steerAngle

    # Steering Angle change cost
    cost += abs(motionCommands[0] - currentNode.steerAngle) * Cost.steerAngleChange

    return cost

def map():
    # Build Map

    return obstacleX, obstacleY

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
        currentNode = costQueue.popitem()[0]

        # Get Reed-Shepp Node if available
        reedSheppNode = analyticExpansion(currentNode, goalNode)

        if not reedSheppNode:


        # Get all simulated Nodes from current node
        for i in range(len(motionCommands)):
            simulatedNode = kinematicSimulationNode(currentNode, motionCommands[i])

            # Check if path is within map bounds and is collision free
            if not isValid(simulatedNode)
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
    obstacleX, obstacleY = map()

    # Calculate map Paramaters
    mapParamaters = calculateMapParameters(obstacleX, obstacleY, 2, 0.3)

    # Run Hybrid A*
    path, plot = run(s, g, mapParamaters, plot)

    # Draw Car Footprint
    drawFootprint(path, plot)

if __name__ == '__main__':
    main()
