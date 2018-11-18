from rrt import *

class BiDirectionRRT(object):
    def __init__(self, startNode, targetNode, step1, step2, vertices):
        self.step = step1
        self.rrt0 = RRT(startNode, targetNode, step1, vertices)
        self.rrt1 = RRT(targetNode, startNode, step2, vertices)
        self.done = False

    def buildTree(self):
        indicator = 0
        color_arr = ['blue', 'green']
        while self.done is False:
            toUpdatedRRT, toCompareRRT = self.getRRT(indicator)
            toUpdatedRRT.buildEdge(color_arr[indicator])
            if self.isIntersected(indicator) is True:
                self.done = True
            indicator = 1 - indicator

    def isIntersected(self, rrtIndicator):
        updatedRRT, toCompareRRT = self.getRRT(rrtIndicator)
        latestPoint = updatedRRT.nodes[len(updatedRRT.nodes) - 1]
        for i in range(len(toCompareRRT.nodes)):
            node = toCompareRRT.nodes[i]
            if self.getDistance(latestPoint, node) < self.step and toCompareRRT.isLineExisting(node, latestPoint):
                updatedRRT.drawPath(len(updatedRRT.nodes) - 1)
                toCompareRRT.drawPath(i)
                return True
        return False

    
    def getRRT(self, indicator):
        if indicator == 0:
            return self.rrt0, self.rrt1
        elif indicator == 1:
            return self.rrt1, self.rrt0
    
    def getDistance(self, p1, p2):
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5