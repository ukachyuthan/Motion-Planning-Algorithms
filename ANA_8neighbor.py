# -*- coding: utf-8 -*-
"""
Created on Tue Feb 19 22:16:34 2019

@author: Achyuthan
"""

# import libraries
from sys import version_info
if version_info.major == 2:
    # We are using Python 2.x
    from Tkinter import *
    import ttk
elif version_info.major == 3:
    # We are using Python 3.x
    from tkinter import *
    from tkinter import ttk
import pdb
import time as t
import numpy as np

openlist=[] #Will contain the nodes that need to be opened
G=1e15 #Cost to goal
E=1e15 #The value that weights the heuristics
sp=[] #records the shortest path
mazelist=[] #takes all the information of the nodes of maze

'''
Define the color scheme for visualization. You may change it but I recommend using the same colors
'''
# white (0) is an unvisited node, black(1) is a wall, blue(2) is a visited node
# yellow(3) is for start node, green(4) is for exit node, red (5) is a node on the completed path
colors = {5: "red", 4: "green", 3: "yellow", 2: "blue", 1: "black", 0: "white"}


'''
Opens the maze file and creates tkinter GUI object
'''
# load maze
with open("easy.txt") as text:#By default the easy maze is loaded. To explore the hard maze uncomment line 37 while commenting line 36
#with open("hard.txt") as text:
    maze = [list(line.strip()) for line in text]
[col, row] = np.shape(maze)

# create map
root = Tk()
size = 800 / row
canvas = Canvas(root, width=(size*row), height=(size*col))
root.title("ANA* Algorithm")


def draw_canvas(canvas, maze):
    '''
Function to draw the grid and plot the solution
    '''
    for i in range(0, col):
        for j in range(0, row):
            canvas.create_rectangle(j*size, i*size, (j+1)*size, (i+1)*size, fill=colors[int(maze[i][j])])
    canvas.pack()


class node:
        def __init__(self, val, x, y):
            self.color = val #Saves the color of the node in maze
            self.x = x #
            self.y = y
            self.e = None
            self.f = None
            self.g = 1e15  # a very high value
            self.h = None  # use Euclidean distance as heuristic
            self.parent = None
            
def heuristics(curr,goal):
    '''
    This function is to calculate the heuristics of the algorithm. In this case Euclidian distance was used
    '''
    h=((curr.x-goal.x)**2+(curr.y-goal.y)**2)**0.5
    return h

def Emax(openlist,G):
    '''
    This function returns the index of the maximum e(s) value to identify the node to be expanded in the ANA* methods
    '''
    eslist=[]
    for i in range (0,len(openlist)):
        eslist.append(openlist[i].e) #records all the e values in the form of a list
    s=np.argmax(eslist) #argmax returns the corresponding index value of the maximum e value
    return s

def Eval(G,g,h):
    '''
    This function is to find the e value of all nodes given G, g and h values
    '''
    e=(G-g)/(h + 1e-15) #the 1e-15 value is to avoid the 0 heuristic value associated with goal node
    return e

def successor(mazelist,s):
    '''
    This fucntion is to identify the children that will be expanded from the selected node from openlist
    '''
    children=[]
    for i in range (0,len(mazelist)):
        if i!=s:#This condition is to avoid considering the parent node as a child. The condition below is to satisfy the 4 neighbor for a parent node condition
            if (mazelist[i].x==(mazelist[s].x+1) or mazelist[i].x==(mazelist[s].x-1) or mazelist[i].x==mazelist[s].x):
                if mazelist[i].y==(mazelist[s].y+1) or mazelist[i].y==(mazelist[s].y-1) or mazelist[i].y==mazelist[s].y:
                    if mazelist[i].color!="1":#Checking for obstacle at node
                        children.append(mazelist[i])
    return children

def locfinder(mazelist,x,y):
    '''
    Given the maze information and x and y values the corresponding index of the coordinates location will be returned by this function
    '''
    for i in range(0,len(mazelist)):
        if mazelist[i].x==x and mazelist[i].y==y:
            loc=i
    return loc

def improvesolution(goal):
    '''
    Given the goal this function will identify the path to goal.
    Repeated iterations of the function will send the algorithm towards optimal solution
    '''
    global G,openlist,E,mazelist
    while len(openlist)!=0:
        for i in range (0,len(openlist)):
            openlist[i].h=heuristics(openlist[i],goal)#define the huristics of all the nodes in the openlist
            openlist[i].e=Eval(G,openlist[i].g,openlist[i].h)#define the e value of all the nodes in the openlist
        s=Emax(openlist,G)#The maximum E value is in the location s and Snode is the corresponding node
        Snode=openlist[s]
        #print("gg",openlist[s].x," ",openlist[s].y)
        del openlist[s]#the node with max e value is deleted as it is being expanded
        if Snode.e<E:
            E=Snode.e#Replacing the E value with the optimal e value
        if Snode.color=="4":#Checking for the goal node by virtue of its color
            G=Snode.g#G value will be updated to the cost of reaching Snode from start as this route is now the solution
            current=Snode.parent#
            while(current!=None):#This loop is to backtrack from the goal to identify the shortest path found
                sp.append(current)
                current=current.parent
            break
        children=[]
        lol=locfinder(mazelist,Snode.x,Snode.y)#The location of Snode is identified here
        children=successor(mazelist,lol)#The children of Snode is evaluated here
        for i in range (0,len(children)):
            children[i].h=heuristics(children[i],goal)#The heuristics of the children are calculated
            children[i].e=Eval(G,children[i].g,children[i].h)#The e value of the children are calculated
        for i in range (0,len(children)):
            cc=1#The cost of travelling to a node
            if Snode.g+cc<children[i].g:#This block of code is to determine which childrens are to be expanded based on their g and h values
                children[i].g=Snode.g+cc
                children[i].parent=Snode
                locp=locfinder(mazelist,children[i].x,children[i].y)
                mazelist[locp].parent=children[i].parent
                if children[i].g+children[i].h<G:
                    openlist.append(children[i])

def ana_star(maze, start_node, exit_node):
    global mazelist,G,E,openlist


    # This visualizes the grid.
    maze[start_node.x][start_node.y] = "3"
    maze[exit_node.x][exit_node.y] = "4"
    draw_canvas(canvas, maze)
    root.update()

    #-------------------------------------------YOUR CODE HERE--------------------------------------------------

    start=locfinder(mazelist,start_node.x,start_node.y)#get the location of the start node in mazelist
    mazelist[start].g=0#initializing the first start g value as 0
    mazelist[start].h=heuristics(mazelist[start],exit_node)#calculating the heuristics of the start node
    mazelist[start].e=Eval(G,mazelist[start].g,mazelist[start].h)#calculating the e value of the start node
    openlist.append(mazelist[start])#adding the start node to open list to be expanded
    while len(openlist)!=0:#This block of code executes the ANA* ideologies
        i=0
        improvesolution(exit_node)
        #print("new len",len(openlist))
        for i in range (0,len(openlist)):#every member of the openlist is considered as nodes visited and are colored blue
            s=locfinder(mazelist,openlist[i].x,openlist[i].y)
            mazelist[s].color="2"

        
        for i in range (0,len(openlist)):#This loop is to evaluate the e value of all the members of the openlist
            openlist[i].e=Eval(G,openlist[i].g,openlist[i].h)
        i=0
        for index,node in enumerate(openlist):#This loop is to pop the members of the openlist which are guarenteed to not have an optimal alternative to the current solution, aka, pruning
           if node.g+node.h>=G:
               j=openlist.pop(index)   

            
            

    #-----------------------------------------------------------------------------------------------------------

    return


def main():
    t0=t.time()
    global mazelist,G,E,openlist
    '''
    Define start and goal node. You may change how to define the nodes.
    '''
    entrance_node = node("3",row-1, 1)#The entrance node or start node
    exit_node =node("4",0, col-2)#The exit node or goal


    # run the ana_star algorithm
    for i in range(0,row):#These two nested loops are to add all the information of the maze into the mazelist list of node objects
        for j in range (0,col):
            mazelist.append(node(maze[i][j],i,j))
    endit=locfinder(mazelist,exit_node.x,exit_node.y)#finding the location of goal nodes in mazelist
    mazelist[endit].color="4"#setting the color greeen for the goal node
    ana_star(maze, entrance_node, exit_node)#Execute the algorithm
    i=0
    for j in range(0,row):#The two nested loops supply the color infromation from mazelist to maze post all the operations
        for k in range (0,col):
            maze[j][k]=mazelist[i].color
            i=i+1
    for k in range (0,len(sp)):#Members of the sp path are considered to be visited nodes and are painted blue
        for i in range(0,row):
            for j in range (0,col):
                if i==sp[k].x and j==sp[k].y:
                    maze[i][j]="2"
    finch=0
    totalc=0
    i=len(sp)-1
    while finch==0:#Here members of the last iteration or the final optimal solution are identified and the corresponding maze locations are painted red
        if sp[i].x==sp[len(sp)-1].x and sp[i].y==sp[len(sp)-1].y and i!=len(sp)-1:
            #print("conditions",sp[len(sp)-1].x," ",sp[len(sp)-1].y)
            finch=1
        else:
            for j in range(0,row):
                for k in range (0,col):
                    if j==sp[i].x and k==sp[i].y:
                        totalc=totalc+1
                        maze[j][k]="5"
        i=i-1
    pathindic=1
   # endpath=0
    pathcounter=1
    pathlength=0
    pc=0
    sp.reverse()
    condition=0
    final_path=[]
    a=[]
    a.append(0)
    print("Optimal Path")
    for i in range (0,len(sp)):
        if (sp[i].x==exit_node.x+1 and sp[i].y==exit_node.y) or (sp[i].x==exit_node.x-1 and sp[i].y==exit_node.y) or (sp[i].x==exit_node.x and sp[i].y==exit_node.y+1) or (sp[i].x==exit_node.x+1 and sp[i].y==exit_node.y-1):
            print("X=",sp[i].x,"Y=",sp[i].y)
            print("X=",exit_node.x,"Y=",exit_node.y)
            print("Path length including start and end nodes:",pathlength+2)
            pathlength=0
            if i!=len(sp)-1:
                print("Alternate Path ",pathcounter)
                pathcounter=pathcounter+1
        else:
            print("X=",sp[i].x,"Y=",sp[i].y)
            pathlength=pathlength+1
    t1=t.time()
    total=t1-t0
    print("Time for execution:",total)   
    draw_canvas(canvas,maze)
            
    root.mainloop()
    
if __name__ == '__main__':
    main()
