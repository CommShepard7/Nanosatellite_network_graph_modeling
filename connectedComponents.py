import networkx as nx
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import random
import csv
import time


def satClusters(transmissionRange,clusterMinSize,time):
    satPos = getSatCoordinates(0)
    connectedNodes = []
    for k in range(0,100):
       connectedNodes.append(getReachableNodes(k,satPos,transmissionRange))
    return resolveClusters(connectedNodes,clusterMinSize)
     

def resolveClusters(connectedNodes,clusterMinSize):
    clusters = []
    nodesList = []
    for k in range(0,100):
        cluster = [k]
        for i in connectedNodes[k]:
            for j in range(0,len(connectedNodes[k])):
                if(len(cluster) >= 1):
                    for s in cluster:
                        clusterNode = True
                        if(not i in connectedNodes[s]):
                            clusterNode = False
                    if(not i in cluster and clusterNode):
                        cluster.append(i)
                else:
                    cluster.append(i)
        if(len(cluster) != 0):
            clusterNumber = 0
            for c in clusters:
                if(set(c) == set(cluster)):
                    clusterNumber += 1
            if(clusterNumber == 0 and len(cluster) >= clusterMinSize):
                clusters.append(cluster)
        cluster = []
    return clusters

def writeClusters(transmissionRange,clusterMinSize,timeScale):
    clusterFile = open("clusters.txt","w")
    for k in range(0,9999,timeScale):
        clusters = satClusters(transmissionRange,clusterMinSize,k)
        for c in clusters:
            for i in range(0,len(c)-1):
                clusterFile.write(str(c[i])+",")
            clusterFile.write(str(c[len(c)-1])+";")
        clusterFile.write("\n")
    clusterFile.close()
    return

def getReachableNodes(nodeNumber,satPos,transmissionRange):
    satCoordinates = [satPos[nodeNumber][0],satPos[nodeNumber][1],satPos[nodeNumber][2]]
    reachableNodes = []
    for k in range(0,100):
        distantSatCoordinates = [satPos[k][0],satPos[k][1],satPos[k][2]]
        if(nodeReachable(satCoordinates,distantSatCoordinates,transmissionRange)):
            reachableNodes.append(k)
    if(len(reachableNodes) == 0 and transmissionRange <= 20):
        transmissionRange += 20
        return getReachableNodes(nodeNumber,satPos,transmissionRange)
    else:
        return reachableNodes

def nodeReachable(nodeCoordinates1,nodeCoordinates2,transmissionRange):
    satX = nodeCoordinates1[0]
    satY = nodeCoordinates1[1]
    satZ = nodeCoordinates1[2]
    distantSatX = nodeCoordinates2[0]
    distantSatY = nodeCoordinates2[1]
    distantSatZ = nodeCoordinates2[2]
    return np.sqrt(pow(satX-distantSatX,2)+pow(satY-distantSatY,2)+pow(satZ-distantSatZ,2)) <= transmissionRange

def getSatCoordinates(t):
    satPositionsFile = open("Traces.csv")
    satPosData = csv.reader(satPositionsFile)
    satCoordinates = []
    coordinates = []
    nextSat = 0
    for row in satPosData:
        coordinates.append(float(row[t])/1000.0)
        nextSat += 1
        if nextSat == 3:
            satCoordinates.append(coordinates)
            coordinates = []
            nextSat = 0
    satPositionsFile.close()
    return satCoordinates

def sat3DPlot(timeScale,globalView):
    space = plt.figure(figsize=(20, 20))
    ax = space.add_subplot(111,projection='3d')
    satX = [0]*99
    satY = [0]*99
    satZ = [0]*99
    for k in range(0,9999,timeScale) :
        satCoordinates = getSatCoordinates(k)
        for i in range(0,99):
            satX[i] = satCoordinates[i][0]
            satY[i] = satCoordinates[i][1]
            satZ[i] = satCoordinates[i][2]
        ax.scatter(satX,satY,satZ)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        if globalView:
            ax.set_xlim([-2000,2000])
            ax.set_ylim([-2000,2000])
            ax.set_zlim([-4000,5000])
        plt.title("Satellites' positions, t="+str(k))
        plt.draw()
        plt.pause(0.00001)
        ax.cla()
    plt.close()

def sat2DPlot(timeScale,globalView):
    satX = [0]*99
    satZ = [0]*99
    for k in range(0,9999,timeScale) :
        satCoordinates = getSatCoordinates(k)
        for i in range(0,99):
            satX[i] = satCoordinates[i][0]
            satZ[i] = satCoordinates[i][2]
            #plt.annotate(str(i),[satX[i],satZ[i]])
        plt.xlabel("x(t)")
        plt.ylabel("z(t)")
        if globalView:
            plt.xlim([-2500,2500])
            plt.ylim([-4000,5000])
        plt.text(0, 0, "Earth", size=20, ha="right", va="top", bbox=dict(boxstyle="circle",))
        plt.scatter(satX,satZ)
        plt.title("Satellites' positions, t="+str(k))
        plt.draw()
        plt.pause(0.1)
        plt.cla()
    plt.close()

def cluster3DPlot(timeScale,clusterMinSize,transmissionRange,globalView):
    space = plt.figure(figsize=(20, 20))
    ax = space.add_subplot(111,projection='3d')
    satX = []
    satY = []
    satZ = []
    for k in range(0,9999,timeScale) :
        clusters = satClusters(transmissionRange,clusterMinSize,k)
        satCoordinates = getSatCoordinates(k)
        for cluster in clusters:
            for i in cluster:
                satX.append(satCoordinates[i][0])
                satY.append(satCoordinates[i][1])
                satZ.append(satCoordinates[i][2])
                #plt.annotate(str(i),[satX[i],satZ[i]])
            ax.scatter(satX,satY,satZ)
            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_zlabel("z")
            plt.title("Clusters, minimum size = " + str(clusterMinSize) + ", transmission range = " + str(transmissionRange) + "km, t = " + str(k)
            + ", number of clusters = " + str(len(clusters)))
            plt.draw()
            satX = []
            satY = []
            satZ = []
        plt.pause(0.1)
        ax.cla()
    plt.close()

def createPos(satPos):
    pos = dict()
    for k in range(0,100):
        satX = satPos[k][0]
        satZ = satPos[k][2]
        pos[k] = [satX,satZ]
    return pos

def initGraph(transmissionRange,time,adaptRange):
    G = nx.Graph()
    satCoordinates = getSatCoordinates(time)
    for k in range(0,100):
        nodeCoordinates = [satCoordinates[k][0],satCoordinates[k][1],satCoordinates[k][2]]
        for i in range(k+1,100):
            distantNodeCoordinates = [satCoordinates[i][0],satCoordinates[i][1],satCoordinates[i][2]]
            nodesConnected = nodeReachable(nodeCoordinates,distantNodeCoordinates,transmissionRange)
            if(adaptRange):
                while(not(nodesConnected) and transmissionRange <= 60):
                    transmissionRange += 20
                    if(transmissionRange <= 60):
                        nodesConnected = nodeReachable(nodeCoordinates,distantNodeCoordinates,transmissionRange)
                transmissionRange = 20
            if(nodesConnected):
                G.add_edge(k,i,weight = transmissionRange)
            else:
                G.add_node(k)
            transmissionRange = 20
    pos = createPos(satCoordinates)
    return G,pos

def satEvolvingNetworkPlot(transmissionRange,timeScale,adaptRange):
    options = {
    "font_size": 10,
    "node_size": 300,
    "node_color": "blue",
    "edgecolors": "black",
    "linewidths": 1,
    "width": 1,
    }
    for k in range(0,9999,timeScale):
        [G,pos] = initGraph(transmissionRange,k,adaptRange)
        nx.draw_networkx(G,pos, **options)
        ax = plt.gca()
        ax.margins(0.20)
        plt.title("Satellites' relative positions ((x,z) plane projection), t="+str(k))
        plt.axis("off")
        plt.pause(0.00001)
        plt.cla()
    plt.close()


#satClusters(60,40,0)
#cluster3DPlot(200,7,20,False)
#print(satClusters(20,0))
#print(getReachableNodes(1,getSatCoordinates(0),20))
#satEvolvingNetworkPlot(20,20,False)
#writeClusters(40,1,20)

[G,pos] = initGraph(20,1000,True)
options = {
    "font_size": 10,
    "node_size": 300,
    "node_color": "blue",
    "edgecolors": "black",
    "linewidths": 1,
    "width": 1,
    }

#nx.draw_networkx(G,pos, **options)
#ax = plt.gca()
#ax.margins(0.20)
#plt.axis("off")
#plt.show()

#G_min = nx.minimum_spanning_tree(G)
#nx.draw_networkx(G_min,pos, **options)
#ax = plt.gca()
#ax.margins(0.20)
#plt.axis("off")
#plt.show()


#shortestPath = nx.shortest_path(G,54,86,"weight")




