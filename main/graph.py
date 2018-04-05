
#our class for the adjacency matrix graph implementation
class Graph(object):

	#constructor
	def __init__(self):

		#declare instance variables
		self.size = 256
		self.boardWidth = 16

		#creating the initial 2D array set to all 0s, meaning no connections between the edges
		adjacencyArray = [[0 for x in range(size)] for y in range(size)]


	#method to add an edge
	def addEdge(self,node1x, node1y, node2x, node2y):

		#convert the xy pos to actual positions 1 - 255 for the adjacency matrix
		node1 = findNode(node1x, node1y)
		node2 = findNode(node2x, node2y)

		#tests to see if nodes are even on the board
		if node1 < 0 or node1 > size-1 or node2 < 0 or node 2 > size-1:
			print "Invalid node input, not on board"

		#the differences between the inputted x and y valuse
		xdif = node1x - node2x
		ydif = node1y - node2y

		#checking if the differences make sense, absval of xdif has to be 1 and ydif has to be 0, or xdif = 0 and absval of ydif has to be 1
		elif (abs(xdif) = 1 and ydif = 0) or (abs(ydif) = 1 and xdif = 0):
			adjacencyArray[node1][node2] = 1
			adjacencyArray[node2][node1] = 1

		else:
			print "Invalid node inputs, nodes should not be adjacent"

	#method to find adjacent vertices to index
	def findAdjacent(self,nodeX, nodeY):

		#converting xy pos to actual position
		node = nodeX + boardWidth*nodeY

		adjacent = []
		for i in range(size):
			if adjacencyArray[index][i] = 1:
				adjacent.append(i)

		return adjacent

	#method to convert xy pairs to node
	def findNode(self,nodeX,nodeY):
		node = nodeX + boardWidth*nodeY

	#method to convert nodes to the xy pairs
	def findXY(self,node):

		nodeY = node/boardWidth
		nodeX = node - boardWidth*nodeY

		return(nodeX,nodeY)

	#method to find if two nodes are connected
	def isConnected(self, node1x, node1y, node2x, node2y):
		node1 = findNode(node1x,node1y)
		node2 = findNode(node2x, node2y)
		if adjacencyArray[node1][node2] = 1 or adjacencyArray[node2][node1] = 1:
			return True
		else:
			return False

