from collections import deque
#our class for the adjacency matrix graph implementation
class Graph(object):

    #constructor
    def __init__(self):

        #declare instance variables
        self.size = 256
        self.boardWidth = 16
        self.currentNode = 0

        #creating the initial 2D array set to all 0s, meaning no connections between the edges
        self.adjacencyArray = [[0 for x in range(self.size)] for y in range(self.size)]

		#create all initial nodes
        self.nodeTable = {}
        for i in range(self.size):
            self.nodeTable[i] = Node(i)


	#method to set currentNode
    def setCurrentNode(self,nodeX,nodeY):
        node = self.findNode(nodeX,nodeY)
        self.currentNode = node.getIntVal()
		

    def getCurrentNode(self):
        nodeY = self.currentNode/self.boardWidth
        nodeX = self.currentNode - self.boardWidth*nodeY
        node = self.findNode(nodeX, nodeY)
        return node

	#method to update the amount of time it has been visited
    def updateNodeVisit(self):
        self.getCurrentNode().visit()

	#method to add an edge
    def addEdge(self,node1x, node1y, node2x, node2y):

		#convert the xy pos to actual positions 1 - 255 for the adjacency matrix
        node1 = self.findNode(node1x, node1y)
        node2 = self.findNode(node2x, node2y)

		#tests to see if nodes are even on the board
        if node1.getIntVal() < 0 or node1.getIntVal() > self.size-1 or node2.getIntVal() < 0 or node2.getIntVal() > self.size-1:
            print "Invalid node input, not on board"
            return

		#the differences between the inputted x and y valuse
        xdif = node1x - node2x
        ydif = node1y - node2y

		#checking if the differences make sense, absval of xdif has to be 1 and ydif has to be 0, or xdif = 0 and absval of ydif has to be 1
        if (abs(xdif) == 1 and ydif == 0) or (abs(ydif) == 1 and xdif == 0):
            print '>>>NODE TEST'
            self.adjacencyArray[node1.getIntVal()][node2.getIntVal()] = 1
            self.adjacencyArray[node2.getIntVal()][node1.getIntVal()] = 1

        else:
            print "Invalid node inputs, nodes should not be adjacent"

	#method to find adjacent vertices to index
    def findAdjacent(self,node):

        adjacent = []
        index = node.getIntVal()

        for i in range(self.boardWidth):
            if self.adjacencyArray[index][i] == 1:
                nodeY = index/self.boardWidth
                nodeX = index - self.boardWidth*nodeY
                adjacent.append(self.findNode(nodeX, nodeY))

        return adjacent

	#method to convert xy pairs to node
    def findNode(self,nodeX,nodeY):
        key = nodeX + self.boardWidth*nodeY
        return self.nodeTable[key]

	#method to convert nodes to the xy pairs
    def findXY(self, node):

        nodeY = node.getIntVal()/self.boardWidth
        nodeX = node.getIntVal() - self.boardWidth*nodeY

        return(nodeX,nodeY)

	#method to find if two nodes are connected
    def isConnected(self, node1x, node1y, node2x, node2y):
        node1 = self.findNode(node1x,node1y)
        node2 = self.findNode(node2x, node2y)
        if self.adjacencyArray[node1.getIntVal()][node2.getIntVal()] == 1 or self.adjacencyArray[node2.getIntVal()][node1.getIntVal()] == 1:
            return True
        else:
            return False

	#returns a list of the shortest path between two points
    def shortestPath(self, node1x, node1y, node2x, node2y):

        node1 = self.findNode(node1x, node1y)
        node2 = self.findNode(node2x, node2y)

        queue = deque()
        queue.append(node1)


        while not len(queue) == 0:

			#pop the first thing in queue and gets neigbors
            currentNode = queue.popleft()
            neighbors = self.findAdjacent(currentNode)

			#gets us out of the loop if it finds desired nodes
            if currentNode == node2:
                break

			#for every neigbor, if it hasn;t been marked yet, mark it with a prevNode and append it to the queue
            for i in range(neigbors):
                if neigbors[i].getPrev == None:
                    neigbors[i].setPrev(currentNode)
                    queue.append(neigbors[i])

		#if it traversed all paths and didn't find it
        if not currentNode == node2:
            return None

		#shortest path begins with endpoint
        shortestPath = deque()
        shortestPath.appendLeft(currentNode)

		#then go back through the prev pointers adding the
        while not currentNode.getPrev() == None:
            currentNode = currentNode.getPrev()
            shortestPath.appendLeft(currentNode)

        shortestPathBack = list(shortestPath)
        shortestPathBack.reverse()
        return (shortestPath,shortestPathBack)



class Node(object):
	#constructor
    def __init__(self,intVal):
        self.intVal = intVal
        self.prevNode = None
        self.visited = 0
        self.boardWidth = 16

	#mutator for the previous node used in bfs algorithm
    def setPrev(self, Node):
        self.prevNode = Node

	#accessor for the previous node
    def getPrev(self):
        return self.prevNode

	#accessor for the intval
    def getIntVal(self):
        return self.intVal

	#increment the amount its been visited
    def visit(self):
        self.visited += 1

	#accessor for visited
    def getVisited(self):
        return self.visited

    def getXY(self):
        nodeY = self.intVal/self.boardWidth
        nodeX = self.intVal - self.boardWidth*nodeY

        return(nodeX,nodeY)

