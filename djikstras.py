# Python program for Dijkstra's single  
# source shortest path algorithm. The program is  
# for adjacency matrix representation of the graph 
  
# Library for INT_MAX 
import sys 
from pandas import *   
class Graph(): 
    def __init__(self, vertices): 
        self.V = vertices 
        self.graph = [[0 for column in range(vertices)]  
                    for row in range(vertices)]
   
    def printSolution(self, dist): 
        print ("Vertex=Distance from Source") 
        min = 0
        for node in range(self.V): 
            if node == 4:
                print("Seattle=", dist[node]) 
            elif node == 7:
                print("New York=", dist[node]) 
    # A utility function to find the vertex with  
    # minimum distance value, from the set of vertices  
    # not yet included in shortest path tree 
    def minDistance(self, dist, sptSet): 
   
        # Initilaize minimum distance for next node 
        min = sys.maxsize 
        min_index = -1
        # Search not nearest vertex not in the  
        # shortest path tree 
        for v in range(self.V): 
            if dist[v] < min and v not in sptSet: 
                min = dist[v] 
                min_index = v 
   
        return min_index 
   
    # Funtion that implements Dijkstra's single source  
    # shortest path algorithm for a graph represented  
    # using adjacency matrix representation 
    def dijkstra(self, src): 
        self.dist = [sys.maxsize] * self.V 
        self.prev_vertex = [None] * self.V 
        self.dist[src] = 0
        sptSet = []
        for cout in range(self.V): 
            # Pick the minimum distance vertex from  
            # the set of vertices not yet processed.  
            # u is always equal to src in first iteration 
            u = self.minDistance(self.dist, sptSet) 
   
            # Put the minimum distance vertex in the  
            # shotest path tree 
            sptSet.append(u)
            # Update dist value of the adjacent vertices  
            # of the picked vertex only if the current  
            # distance is greater than new distance and 
            # the vertex in not in the shotest path tree 
            for v in range(self.V): 
                if self.graph[u][v] > 0 and v not in sptSet and self.dist[v] > self.dist[u] + self.graph[u][v]: 
                    self.dist[v] = self.dist[u] + self.graph[u][v] 
                    self.prev_vertex[v] = u
   
        self.printSolution(self.dist)
def getPath(node,prevNodeList,cities,capacities,graph):
    string = cities[node]
    path = [node]
    minC = capacities[prevNodeList[node]][node]
    while prevNodeList[node]!=None:
        if capacities[prevNodeList[node]][node] < minC:
            minC = capacities[prevNodeList[node]][node]
        string = cities[prevNodeList[node]] +">"+ string
        path.insert(0,prevNodeList[node])
        node = prevNodeList[node]
    
    for i in range(len(path)-1):
        capacities[path[i]][path[i+1]] = capacities[path[i]][path[i+1]] - minC
        if capacities[path[i]][path[i+1]] == 0:
            graph[path[i]][path[i+1]] = 0
    return "Optimal Path: "+string+" with lowest capacity possible " + str(minC)+"\n New Capacities:\n"+str(DataFrame(capacities))+"\n New Graph Weights:\n"+str(DataFrame(graph))
"""
Pseudocode:
1) Run Djikstras on graph where weights are cost per container
2) Use all containers in optimal path, deducting every paths capacity
3) If a paths capacity is 0, remove it from graph
4) If there are more needed containers, repeat 1)

Note: we will start with the optimal path from Seoul to New York because it requires more containers and is further away
"""

needed_containers = 38
needed_seattle_containers = int(needed_containers * 3/7)
needed_ny_containers = int(needed_containers * 4 / 7)
# 12 cities, start node = Seoul[0], end nodes = New York[7] and Seattle[4] such that Seattle:New York is 3:4
nodes = ['Seoul','Osaka','Shanghai','Taipei','Seattle','Shanghai','San Francisco','New York','Beijing','LA','KC','Baltimore']
g = Graph(12) 
edges = [('Seoul','Osaka',50,1100),('Seoul','Shanghai',70,900),('Seoul','Taipei',30,1000),('Osaka','Seattle',50,2400),('Shanghai','Seattle',30,2000),
            ('Shanghai','San Francisco',40,800),('Taipei','San Francisco',30,2000),('Seattle','New York',80,800),('San Francisco','New York',70,900),
            ('Beijing','Shanghai',20,1000),('Shanghai','LA',60,2000),('LA','KC',40,600),('LA','Seattle',30,600),('Seattle','Baltimore',40,1000),
            ('KC','Baltimore',20,600),('KC','New York',10000,900),('Baltimore','New York',10000,500)]
capacities = [[0 for column in range(12)]  
                    for row in range(12)] 
for start,end,cap,price in edges:
    i = nodes.index(start)
    j = nodes.index(end)
    g.graph[i][j] = price
    capacities[i][j] = cap
print(DataFrame(g.graph))
print(DataFrame(capacities))
g.dijkstra(0)
print(getPath(7,g.prev_vertex,nodes,capacities,g.graph))
g.dijkstra(0)
print(getPath(4,g.prev_vertex,nodes,capacities,g.graph))



