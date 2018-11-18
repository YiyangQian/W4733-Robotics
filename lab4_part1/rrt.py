import random
import numpy as np
import matplotlib.pyplot as plt

class RRT(object):
    def __init__(self, startNode, targetNode, step, vertices):
        self.nodes = [startNode]
        self.targetNode = targetNode
        self.edges = {}
        self.step = step
        self.vertices = vertices
        self.obstacleEdges = []
        self.done = False

    def getClosestNode(self, goNode):
        curClosestDistance = 1000
        curCloset = self.nodes[0]
        index = -1
        for i in range(len(self.nodes)):
            node = self.nodes[i]
            distance = self.getDistance(node, goNode)
            if distance < curClosestDistance:
                curCloset = node
                curClosestDistance = distance
                index = i
        return curCloset, index

    def generateNextPotentialNode(self, closestNode, goNode):
        distance = self.getDistance(closestNode, goNode)
        unit_oritentation_vector = ((goNode[0] - closestNode[0]) / distance, (goNode[1] - closestNode[1]) / distance)
        return (closestNode[0] + self.step * unit_oritentation_vector[0], closestNode[1] + self.step * unit_oritentation_vector[1])

    def getDistance(self, p1, p2):
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5
    
    def isLineExisting(self, nextPotentailNode, closestNode):
        p1 = nextPotentailNode
        p2 = closestNode
        for edge in self.obstacleEdges:
            q1, q2 = edge
            if self.isIntersected(p1, p2, q1, q2):
                return False
        return True

    def isIntersected(self, p1, p2, p3, p4):
        res = False
        if (max(p1[0], p2[0]) > min(p3[0], p4[0])
        and max(p3[0], p4[0]) > min(p1[0], p2[0])
        and max(p1[1], p2[1]) > min(p3[1], p4[1])
        and max(p3[1], p4[1]) > min(p1[1], p2[1])):
            if (self.cross(p1, p2, p3) * self.cross(p1, p2, p4) < 0 and self.cross(p3, p4, p1) * self.cross(p3, p4, p2) < 0):
                res = True
            else:
                res = False
        else:
            res = False
        return res

    def cross(self, p1, p2, p3):
        x1 = p2[0] - p1[0]
        y1 = p2[1] - p1[1]
        x2 = p3[0] - p1[0]
        y2 = p3[1] - p1[1]
        return x1*y2-x2*y1
    
    def generateObstacleEdges(self):
        storePoint = (0, 0)
        for i in range(len(self.vertices) - 1):
            curPoint = self.vertices[i]
            nextPoint = self.vertices[i + 1]
            if curPoint[0] == 0 and curPoint[1] == 0:
                storePoint = nextPoint
            elif nextPoint[0] == 0 and nextPoint[1] == 0:
                p1 = (storePoint[0], storePoint[1])
                p2 = (curPoint[0], curPoint[1])
                self.buildObstaleEdge(p1, p2)
            else: 
                p1 = (curPoint[0], curPoint[1])
                p2 = (nextPoint[0], nextPoint[1])
                self.buildObstaleEdge(p1, p2)

    def buildObstaleEdge(self, p1, p2):
        newPoint = (p1, p2)
        self.obstacleEdges.append(newPoint)

    def buildTree(self):
        self.generateObstacleEdges()
        while self.done is False:
            self.buildEdge()
        self.drawPath()

    def buildEdge(self):
        goNode = (-1, -1)
        if random.random() < 0.05:
            goNode = self.targetNode
        else: 
            goNode = (random.randint(0, 600), random.randint(0, 600))
        closestNode, parentIndex = self.getClosestNode(goNode)
        nextPotentailNode = self.generateNextPotentialNode(closestNode, goNode)
        isValid = self.isLineExisting(nextPotentailNode, closestNode)
        notTooCloseToExistingNodes = self.isNotTooCloseToExistingNodes(nextPotentailNode)
        if isValid and notTooCloseToExistingNodes: 
            print(nextPotentailNode)
            childIndex = len(self.nodes)
            self.nodes.append(nextPotentailNode)
            self.edges[childIndex] = parentIndex
            self.drawEdge(nextPotentailNode, closestNode, "blue")
            if self.getDistance(nextPotentailNode, self.targetNode) < self.step and self.isLineExisting(self.targetNode, nextPotentailNode):
                self.drawEdge(self.targetNode, nextPotentailNode, "red")
                self.done = True

    def isNotTooCloseToExistingNodes(self, nextPotentailNode):
        for node in self.nodes:
            distance = self.getDistance(nextPotentailNode, node)
            if distance < self.step:
                return False
        return True

    def drawEdge(self, p1, p2, myColor):
        x = [p1[0], p2[0]]
        y = [p1[1], p2[1]]
        plt.plot(x, y, marker = 'o', color = myColor)

    def drawPath(self):
        index = len(self.nodes) - 1
        while index in self.edges:
            parent = self.edges[index]
            p1 = self.nodes[index]
            p2 = self.nodes[parent]
            self.drawEdge(p1, p2, 'red')
            index = parent
            