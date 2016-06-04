# -*- coding: utf-8 -*-
"""
Created on Mon May 9 14:26:45 2016

For CS225A Experimental Robotics; team Sketch (2016)
Please credit if you reuse. Results may require tuning or debugging.

@author: Sean, ststanko@stanford.edu
"""

import cv2
import numpy as np

'''
Return list of neighbor edge nodes by coordinate
'''

def areVertical(low, mid, high):
    return low[0] == mid[0] == high[0]

def areHorizontal(left, mid, right):
    return left[1] == mid[1] == right[1]
    
def areDiagonal(left, mid, right):
    b1 =  (left[0] != mid[0] and mid[0] == right[0]) or (left[0] == mid[0] and mid[0] != right[0])
    b2 = (left[1] != mid[1] and mid[1] == right[1]) or (left[1] == mid[1] and mid[1] != right[1])
    return b1 or b2


def findNeighbors(x,y,img):
    n = []
    w,h = img.shape     
    for i in [-1, 0, 1]:
        for j in [-1, 0, 1]:
            if (i == j == 0) or x+i<0 or x+i>=h or y+j<0 or y+j>=w:
                continue
            else:
                if (img[y+j,x+i] > 0):
                    n.append((y+j,x+i))
    return n

def isUpperDiagonal(G, node):
    rNeighbor = False
    ruNeighbor = False
    lNeighbor = False
    luNeighbor= False
    for n in G[node]:
        if (node[0] - n[0] == 1) and (node[1] - n[1] == 0):
            lNeighbor= True
        if (node[0] - n[0] == 1) and (node[1] - n[1] == -1):
            luNeighbor= True
        if (node[0] - n[0] == -1) and (node[1] - n[1] == 0):
            rNeighbor= True
        if (node[0] - n[0] == -1) and (node[1] - n[1] == -1):
            ruNeighbor= True
    return (rNeighbor and ruNeighbor) or (lNeighbor and luNeighbor)
    
def isLowerDiagonal(G, node):
    rNeighbor = False
    rdNeighbor = False
    lNeighbor = False
    ldNeighbor= False
    for n in G[node]:
        if (node[0] - n[0] == 1) and (node[1] - n[1] == 0):
            lNeighbor= True
        if (node[0] - n[0] == 1) and (node[1] - n[1] == 1):
            ldNeighbor= True
        if (node[0] - n[0] == -1) and (node[1] - n[1] == 0):
            rNeighbor= True
        if (node[0] - n[0] == -1) and (node[1] - n[1] == 1):
            rdNeighbor= True
    return (rNeighbor and rdNeighbor) or (lNeighbor and ldNeighbor)
   
def jaggedDiagonal(left, key, right) :
    isJagged = False
    if(left[0] == key[0]) and (right[1] -  1 == key[1]):
        isJagged = True
    if(left[1] == key[1]) and (right[0] -  1 == key[0]):
        isJagged = True
    if(right[0] == key[0]) and (left[1] -  1 == key[1]):
        isJagged = True
    if(right[1] == key[1]) and (left[0] -  1 == key[0]):
        isJagged = True
    return isJagged

def collapseNodes(G, center, left, right):
    G[left].remove(center)
    if right not in G[left]:
        G[left].append(right)
    G[right].remove(center)
    if left not in G[right]:
        G[right].append(left)
    G.pop(center, None)
    
    
def collapseLargeNode(G, node):
    for neighbor in G[node]:
        G[neighbor].remove(node)
        for neighbor2 in G[node]:
            if not(neighbor2 == neighbor) and (neighbor2 not in G[neighbor]):
                if areVertical(neighbor,node,neighbor2) or areHorizontal(neighbor,node,neighbor2):# or areDiagonal(neighbor,node, neighbor2):
                    G[neighbor].append(neighbor2)
    G.pop(node, None)
   

'''
Pruning methods:
+Remove orphan nodes from graph
'''    
def prune(G, img): 
    pruneBrightness = 64
    '''PASS ZERO'''
    #Remove Orphans  
    for key in graph.keys():
        if len(graph[key]) == 0:
            graph.pop(key, None)
    '''PASS ONE'''
    #Collapse vertical lines and thick diagonals
    for key in graph.keys():
        if len(graph[key]) == 2:
            left = graph[key][0]
            right = graph[key][1]
            #Prune horizontal and vertical
            if (left[0] == right[0]) or (left[1] == right[1]):
                img[key[0],key[1]] = pruneBrightness
                collapseNodes(G, key, left, right)
                continue
        #Prune Diagonal            
        if (isUpperDiagonal(G, key)):
            img[key[0],key[1]] = pruneBrightness
            collapseLargeNode(G, key)
            continue
    for key in graph.keys():
        if len(graph[key]) == 2:
            left = graph[key][0]
            right = graph[key][1]
            if (left[0] == key[0] and right[1] == key[1]) or (left[1] == key[1] and right[0] == key[0]):
                collapseNodes(G, key, left, right)
    '''PASS TWO'''
    #Clean up diagonals further
    for key in graph.keys():
        if len(graph[key]) == 2:
            left = graph[key][0]
            right = graph[key][1]
            #Prune horizontal and vertical
            if (left[0] == right[0]) or (left[1] == right[1]):
                img[key[0],key[1]] = pruneBrightness
                collapseNodes(G, key, left, right)
                continue
    for key in graph.keys():
        if len(graph[key]) == 2:
            left = graph[key][0]
            right = graph[key][1]
            #Prune horizontal and vertical
            if(jaggedDiagonal(left,key,right)):
                img[key[0],key[1]] = pruneBrightness
                collapseNodes(G, key, left, right)
                continue

 
def directGraph(G, sort = None):
    if sort == None:    
        keys = sorted(G.keys())
    else:
        keys = sort  
    for parent in keys:
        for child in G[parent]:
            if parent in G[child]:
                G[child].remove(parent)
    
             
'''
<numNodes>
<nodeID> <x> <y> <numEdges> <edge1> <edge2> ...
<nodeID> <x> <y> <numEdges> <edge1> <edge2> ...
...
'''
def formatGraph(G, sort, w, h):
    formatted = {}
    node_id = {}
    nextID = 878
    #generate node IDs
    if sort == None:    
        keys = sorted(G.keys())
    else:
        keys = sort  
    for key in keys:
        #print nextID
        node_id[key] = nextID
        nextID += 1
        
    #convert each line
    for key in keys:
        fkey = node_id[key]
        x = key[0] / float(h)
        x = float("%.5f" % x)
        y = key[1] / float(w)
        y = float("%.5f" % y)
        numNeighbors = len(G[key])
        neighbors = []
        for node in G[key]:
            neighbors.append(node_id[node])
        formatted[fkey] = [fkey, x, y, numNeighbors] + neighbors
    return formatted

def writeOutput(out):
    with open('waypoints.txt', 'w') as f:
        f.write(str(len(out))+ '\n')
        for i in sorted(out.keys()):
            f.write(str(out[i]).strip('[]').replace(',','')+'\n')
    
  
def sortGraph(G):
    sortedKeys = []
    for key in sorted(G.keys()):
        if key not in sortedKeys:
            if len(G[key]) > 0 and G[key][0] in sortedKeys:
                
                idx = sortedKeys.index(G[key][0])
                print "Inserting parent " + str(key) + " at " + str(idx)
                sortedKeys = sortedKeys[0:idx] + [key] + sortedKeys[idx:]
            else:
                for key2 in sorted(G.keys()):
                    if len(G[key2]) > 0 and G[key2][0] == key and key2 in sortedKeys:
                        
                        idx = sortedKeys.index(key2)
                        print "Inserting child " + str(key) + " at " + str(idx+1)
                        sortedKeys = sortedKeys[0:idx+1] + [key] + sortedKeys[idx+1:]
                        break
                if key not in sortedKeys:
                    print "Inserting new " + str(key) + " at " + str(len(sortedKeys))
                    sortedKeys.append(key)
                     
    return sortedKeys
  

'''
Preprocess image to reduce noise
Some images may need more/less processing; apply filters as needed 
'''       
def filterImage(img):
    '''Apply Gaussian Blur; noise filtering'''
    img2 = cv2.GaussianBlur(img, (5,5),7)
    img2 = cv2.GaussianBlur(img2, (5,5),7)
    #img2 = cv2.GaussianBlur(img2, (5,5),11)
    #img2 = cv2.GaussianBlur(img2, (5,5),11) 
    '''Apply Bilateral filter; edge-sensitive noise filtering'''
    #img2 = cv2.bilateralFilter(img2, 21, 100, 64)
    img2 = cv2.bilateralFilter(img2, 21, 75, 32)
    '''Apply MeanShift Filter; pseudo color clustering'''
    img2 = cv2.pyrMeanShiftFiltering(img2, 20,45)
    '''Downsample Image'''
    h1, w1, d1 = img.shape
    img2= cv2.resize(img2, (200,int(round(h1*(200.0/w1)))))
    return img2


#Still Life
#fn = './GraphInputs/fruit.jpg'

#Musik
#fn = './GraphInputs/trumpet.jpg'

#Eagle
#fn = './GraphInputs/eagle.jpg'

#Smile
fn = './GraphInputs/smile.jpg'

#rocketeer
#fn = './GraphInputs/rocket.jpg'

#scuba
#fn = './GraphInputs/scuba_big.jpg'

#logo,  no tree
#fn = './GraphInputs/block_s.jpg'

#logo,  with tree
#fn = './GraphInputs/logo_tree.png'

#hoover
#fn = './GraphInputs/hoover.jpg'

img = cv2.imread(fn)
img2 = img
img2 = filterImage(img)
img3 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
img3 = cv2.Canny(img3, 10,30)

graph = {}

h,w = img3.shape
img4 = img3.copy()
for i in range(0,h):
    for j in range(0,w):
        if img3[i,j] > 0:
            n = findNeighbors(j,i,img3)
            graph[(i,j)] = n
            if len(n) == 0:
                img4[i,j] = 0

print len(graph)
prune(graph, img4)
print len(graph)

h4, w4 = img4.shape
scale = 3
img5 = np.zeros((h4*scale, w4*scale, 1), np.uint8)
directGraph(graph)
for key in graph.keys():
    for node in graph[key]:
        strtPt = (key[1]*scale, key[0]*scale)
        endPt = (node[1]*scale, node[0]*scale)
        img5 = cv2.line(img5, strtPt, endPt, 255)

keySort = sortGraph(graph)
out = formatGraph(graph, keySort, w4, h4)

writeOutput(out)

'''
#cv2.imshow('Original', img)
cv2.imshow('Filtered', img2)
cv2.imshow('Edges', img3)
h1, w1, d1 = img.shape
#img4= cv2.resize(img4, (200,int(round(h1*(200.0/w1)))))
cv2.imshow('graph', img4)
cv2.imshow('robot', img5)
'''
cv2.imwrite('1_filtered.jpg', img2)
cv2.imwrite('2_edges.jpg', img3)
cv2.imwrite('3_graph.jpg', img4)
cv2.imwrite('4_robot.jpg', img5)

#cv2.waitKey(0) 

     