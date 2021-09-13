import math
import numpy as np

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
    def __int__(self, trajX, trajY, cost):
    self.trajX = trajX
    self.trajY = trajY 
    self.cost = cost

def motionCommands():
    direction = 1
    motionCommands = []
    for i in np.arange(Car.maxSteerAngle, -(Car.maxSteerAngle + Car.maxSteerAngle/Car.steerPresion), -Car.maxSteerAngle/Car.steerPresion):
        motionCommands.append([i, direction])
        motionCommands.append([i, -direction])
    return motionCommands

def kinematicSimulationNode():
    return Node([], [], 0)

def reedsSheppNode():
    return Node([], [], 0)

def drawCar():
    return 0

def run():
    motionCommands = motionCommands()
    while(true):
        for i in range(len(motionCommands)):
            kinematicSimulationNode(motionCommands[i][0], motionCommands[i][1])

def main():
    # Get Start, Goal
    run()
    drawCar()

if __name__ == '__main__':
    main()
