from tkinter import *
from queue import *

#Part of the code/logic was found at https://github.com/vandersonmr/A_Star_Algorithm/blob/master/libs/python/AStar.py


class AStarAlgorithm():

    def __init__(self):
        pass

    #CHANGE BOARD AND ALGORITHM
    #standard algo = aStar
    dijkstra = False
    bfs = False
    file = open('board-2-4.txt', 'r')
    #allow diagonal movement
    #currently alpha
    allow_diagonal = False

    grid = file.read()
    #print(grid)
    #constructing list of grid
    grid_list = grid.split("\n")
    #constructing array of grid
    grid_array = []
    for n in range(len(grid_list) - 1):
        grid_array.append(list(grid_list[n]))
    #checking if board is weighted
    weight_dict = {'w': 100, 'm': 50, 'f': 10, 'g': 5, 'r': 1}
    weighted_indicators = ["w", "m", "f", "g", "r"]
    weighted_ignore = ["A", "B"]
    weighted = False
    for i in range(len(grid_array) - 1):
        for j in range(len(grid_array[i]) - 1):
            if grid_array[i][j] in weighted_indicators:
                weighted = True
                break
    path = []
    closedSet = {}
    openSet = {}
    openNodes = {}

    #GET WEIGHT OF CURRENT NODE
    def getWeight(self, current):
        if self.weighted:
            if self.grid_array[current[1]][current[0]] not in self.weighted_ignore:
                return self.weight_dict[self.grid_array[current[1]][current[0]]]
        return 0

    #SETS START- AND GOAL COORDINATES
    def getInitCo(self):
        for i in range(len(self.grid_list)):
            if "A" in self.grid_list[i]:
                start_y = i
                start_x = self.grid_list[i].index('A')
            if "B" in self.grid_list[i]:
                goal_y = i
                goal_x = self.grid_list[i].index('B')
        self.start = (start_x, start_y)
        print("Start: ", self.start)
        self.goal = (goal_x, goal_y)
        print("Goal: ", self.goal)

    #RETURNS DISTANCE BETWEEN current NODE AND neighbor
    def distBetween(self, current, neighbor):
        #returning 1 as distance between current node and neighbor
        #bc. not allowing diagonal moving, returning 1
        return 1

    #RETURNING DISTANCE FROM start TO goal
    def heuristicEstimate(self, start, goal):
        distance = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
        return distance

    #RETURNING NODE WITH LOWEST fScore
    def getLowest(self, openSet, fScore, wScore):
        lowest = float("inf")
        lowestNode = None
        for node in self.openSet:
            if fScore[node] < lowest:
                lowest = fScore[node]
                lowestNode = node
        return lowestNode

    #RECONSTRUCTING PATH FROM cameFrom TO goal
    def reconstructPath(self, cameFrom, goal):
        self.path = []
        node = goal
        self.path.insert(0, node)
        while node in cameFrom:
            node = cameFrom[node]
            self.path.insert(0, node)
        print("path:", self.path, "\n")
        return self.path

    #FINDING NEIGHBORING NODES
    def neighborNodes(self, current):
        #find coordinates of current
        curr_x = current[0]
        curr_y = current[1]
        #find possible neighbors
        possible_n = []
        possible_n.append((curr_x + 1, curr_y))
        possible_n.append((curr_x - 1, curr_y))
        possible_n.append((curr_x, curr_y + 1))
        possible_n.append((curr_x, curr_y - 1))
        if self.allow_diagonal:
            possible_n.append((curr_x + 1, curr_y + 1))
            possible_n.append((curr_x + 1, curr_y - 1))
            possible_n.append((curr_x - 1, curr_y - 1))
            possible_n.append((curr_x - 1, curr_y + 1))
        #checking if neighbors are out of map or obstacle
        neighbors = []
        for node in possible_n:
            #checking if out out map
            if len(self.grid_array) > node[1] and len(self.grid_array[0]) > node[0] and node[1] >= 0 and node[0] >= 0:
                #checking if obstacle
                if self.grid_array[node[1]][node[0]] != '#':
                    neighbors.append(node)
        return neighbors

    #A* ALGORITHM
    def aStar(self):
        cameFrom = {}
        self.openSet = set([self.start])
        self.closedSet = set()
        self.openNodes = set([self.start])
        #distance between node and starting point
        gScore = {}
        #total score
        fScore = {}
        #cost
        wScore = {}
        gScore[self.start] = 0
        wScore[self.start] = 0
        fScore[self.start] = gScore[self.start] + self.heuristicEstimate(self.start, self.goal)
        #if BFS algorithn
        if self.bfs:
            self.openSet = list()
            self.openSet.append(self.start)
            self.closedSet = list()
            while len(self.openSet) != 0:
                current = self.openSet[0]
                #checking for end point
                if current == self.goal:
                    self.path = self.reconstructPath(cameFrom, self.goal)
                    return
                self.openSet.pop(0)
                self.closedSet.append(current)
                #checking all neighbors
                for neighbor in self.neighborNodes(current):
                    #checking for end point
                    if neighbor == self.goal:
                        cameFrom[neighbor] = current
                        self.path = self.reconstructPath(cameFrom, self.goal)
                        return
                    if neighbor not in self.openSet and neighbor not in self.closedSet:
                        self.openSet.append(neighbor)
                        self.openNodes.add(neighbor)
                        cameFrom[neighbor] = current
        while len(self.openSet) != 0:
            #finds node with lowest fScore and wScore
            current = self.getLowest(self.openSet, fScore, wScore)
            #checking if end point
            if current == self.goal:
                self.path = self.reconstructPath(cameFrom, self.goal)
                break
            self.openSet.remove(current)
            self.closedSet.add(current)
            #checking all neighbors
            for neighbor in self.neighborNodes(current):
                self.openNodes.add(neighbor)
                #setting tentative gScore
                tentative_gScore = gScore[current] + self.getWeight(current) + self.distBetween(current, neighbor)
                if neighbor in self.closedSet and tentative_gScore >= gScore[neighbor]:
                    continue
                if neighbor not in self.closedSet or tentative_gScore < gScore[neighbor]:
                    cameFrom[neighbor] = current
                    gScore[neighbor] = tentative_gScore
                    wScore[neighbor] = self.getWeight(neighbor)
                    #Dijkstra does not use distance to end point
                    if self.dijkstra:
                        fScore[neighbor] = gScore[neighbor] + wScore[neighbor] * 100
                    #BFS only uses distance to end point
                    elif self.bfs:
                        fScore[neighbor] = self.heuristicEstimate(neighbor, self.goal)
                    #A* uses all three
                    else:
                        fScore[neighbor] = gScore[neighbor] + wScore[neighbor] * 2 + self.heuristicEstimate(neighbor, self.goal)
                    if neighbor not in self.openSet:
                        self.openSet.add(neighbor)
        print("Done.")

    #DRAWING IMAGE OF BOARD W/ PATH
    def drawPath(self):
        master = Tk()
        w = Canvas(master, width=len(self.grid_array[0]) * 30, height=len(self.grid_array) * 30)
        w.pack()

        for i in range(0, len(self.grid_array)):
            for j in range(0, len(self.grid_array[i])):
                pos = self.grid_array[i][j]
                #scaling
                r = 30
                #diameter circle
                offset = 10
                if pos == "#":
                    w.create_rectangle(j * r, i * r, (j + 1) * r, (i + 1) * r, fill="grey")
                elif pos == ".":
                    w.create_rectangle(j * r, i * r, (j + 1) * r, (i + 1) * r, fill="white")
                elif pos == "A":
                    w.create_rectangle(j * r, i * r, (j + 1) * r, (i + 1) * r, fill="red")
                    w.create_oval(j * r + offset, i * r + offset, (j + 1) * r - offset, (i + 1) * r - offset, fill="black")
                elif pos == "B":
                    w.create_rectangle(j * r, i * r, (j + 1) * r, (i + 1) * r, fill="green")
                    w.create_oval(j * r + offset, i * r + offset, (j + 1) * r - offset, (i + 1) * r - offset, fill="black")
                elif pos == "w":
                    w.create_rectangle(j * r, i * r, (j + 1) * r, (i + 1) * r, fill="blue")
                elif pos == "f":
                    w.create_rectangle(j * r, i * r, (j + 1) * r, (i + 1) * r, fill="dark green")
                elif pos == "r":
                    w.create_rectangle(j * r, i * r, (j + 1) * r, (i + 1) * r, fill="brown")
                elif pos == "m":
                    w.create_rectangle(j * r, i * r, (j + 1) * r, (i + 1) * r, fill="grey")
                elif pos == "g":
                    w.create_rectangle(j * r, i * r, (j + 1) * r, (i + 1) * r, fill="pale green")
                #draw line if closed node
                if (j, i) in self.closedSet:
                    w.create_text(j * r + offset, i * r + offset, text="x")
                #draw polygon if open node
                if (j, i) in self.openNodes and (j, i) not in self.closedSet:
                    w.create_text(j * r + offset, i * r + offset, text="*")
                #draw oval if path
                if (j, i) in self.path:
                    w.create_oval(j * r + offset, i * r + offset, (j + 1) * r - offset, (i + 1) * r - offset, fill="black")
        mainloop()


if __name__ == '__main__':
    a = AStarAlgorithm()
    a.getInitCo()
    a.aStar()
    a.drawPath()
