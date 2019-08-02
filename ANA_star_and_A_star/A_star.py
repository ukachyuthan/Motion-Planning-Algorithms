## A* Algorithm

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

import time as t
import numpy as np
import sys

openlist=[]
closedlist=[]
mazelist=[]
sp=[]
childs=[]

'''
Define the color scheme for visualization.
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

def heuristics(curr,goal):
    '''
    This function is to calculate the heuristics of the algorithm. In this case Euclidian distance was used
    '''
    h=((curr.x-goal.x)**2+(curr.y-goal.y)**2)**0.5
    return h

def Fmin(openlist):
    '''
    This function finds the minimum f value in the list of nodes
    '''
    fslist=[]
    for i in range (0,len(openlist)):
        fslist.append(openlist[i].f)
    s=np.argmin(fslist)
    return s

def nextinline(node,exit_node):
    '''
    Finds all the successors to a given node
    '''
    global mazelist
    children=[]
    for i in range(0,len(mazelist)):
        '''
        Uncomment line 71 and comment lines 72 and 73 for for using 4 neighbor expansion. By default 8 neighbor method is active
        '''
        #if (node.x==mazelist[i].x+1 and node.y==mazelist[i].y) or (node.x==mazelist[i].x-1 and node.y==mazelist[i].y) or (node.x==mazelist[i].x and node.y==mazelist[i].y+1) or (node.x==mazelist[i].x and node.y==mazelist[i].y-1):
        if (node.x==mazelist[i].x+1 or node.x==mazelist[i].x or node.x==mazelist[i].x-1):
            if (node.y==mazelist[i].y+1 or node.y==mazelist[i].y or node.y==mazelist[i].y-1):
                if mazelist[i].color!='1':
                    children.append(mazelist[i])
    return children

def checklist(child,openlist):
    '''
    Checks if a node is in a given list or not
    '''
    ok=0
    for i in range (0,len(openlist)):
        if child.x==openlist[i].x and child.y==openlist[i].y:
            ok = 1
    return ok

def locfinder(mazelist,x,y):
    '''
    Given the maze information and x and y values the corresponding index of the coordinates location will be returned by this function
    '''
    for i in range(0,len(mazelist)):
        if mazelist[i].x==x and mazelist[i].y==y:
            loc=i
    return loc

def draw_canvas(canvas, maze):
    '''
    draws the canvas
    '''
    for i in range(0, col):
        for j in range(0, row):
            canvas.create_rectangle(j*size, i*size, (j+1)*size, (i+1)*size, fill=colors[int(maze[i][j])])
    canvas.pack()
    
class node:
    def __init__(self, val, x, y):
            self.color = val #saves the color of the node
            self.x = x
            self.y = y
            self.e = None
            self.f = None
            self.g = 99999999  # a very high value
            self.h = None  # use Euclidean distance as heuristic
            self.parent = None

    
def a_star(maze, start_node, exit_node):

    global mazelist,openlist,closedlist,childs

    # This visualizes the grid. You may remove this and use the functions as you wish.
    maze[start_node.x][start_node.y] = start_node.color #assigning the colors to start and exit nodes
    maze[exit_node.x][exit_node.y] = exit_node.color
    draw_canvas(canvas, maze)
    root.update()

    #-------------------------------------------YOUR CODE HERE--------------------------------------------------
    '''Initialize the g, h and f values for the start node. Append the same to opennlist
    '''
    start_node.g=0
    start_node.f=heuristics(start_node,exit_node)
    openlist.append(start_node)
    while len(openlist)!=0:
        '''
        Lowest value of f is identified. Deleted from openlist after being selected for expansion
        '''
        loc=Fmin(openlist)
        curr=openlist[loc] 
        del openlist[loc]
        closedlist.append(curr) #adding the opened node to closed list
        if curr.x==exit_node.x and curr.y==exit_node.y:
            '''
            If the selected node is goal the loop is broken and path is printed
            '''
            current=curr.parent
            while(current!=None):
                sp.append(current)
                current=current.parent
            break
        '''
        Exploring the successors of the current node
        '''
        childs=nextinline(curr,exit_node)
        for i in range (0,len(childs)):
            '''
            Sets the f value for node
            '''
            childs[i].h=heuristics(childs[i],exit_node)
            childs[i].g=curr.g+1
            childs[i].f=childs[i].g+childs[i].h
        for i in range (0,len(childs)):
            if checklist(childs[i],openlist): #Avoid checking for the same node if already better option in openlist 
                loc=locfinder(openlist,childs[i].x,childs[i].y)
                if childs[i].g>=openlist[loc].g:
                    continue
            if checklist(childs[i],closedlist):#Avoid checking for the same node if already better option in closedlist 
                loc=locfinder(closedlist,childs[i].x,childs[i].y)
                if childs[i].g>=closedlist[loc].g:
                    continue
            '''
            Select the appropriate child. Set current node to parent and add the child to openlist for expansion
            '''
            childs[i].parent=curr
            openlist.append(childs[i])
    #-----------------------------------------------------------------------------------------------------------

    return

def main():
    t1=t.time()
    global mazelist

    '''
    Define start and goal node. You may change how to define the nodes.
    '''
    entrance_node = node(3,row-1, 1)
    exit_node = node(4,0, col-2)   
    for i in range (0,row):#These two nested loops are to add all the information of the maze into the mazelist list of node objects
        for j in range (0,col):
            mazelist.append(node(maze[i][j],i,j))   
    # run the a_star algorithm
    a_star(maze, entrance_node, exit_node)
    for i in range (0,len(closedlist)):#Visited or explored nodes are painted blue
        for j in range (0,row):
            for k in range (0,col):
                if closedlist[i].x==j and closedlist[i].y==k:
                    maze[j][k]='2'
                    
    for k in range (0,len(sp)):#Members of the shortest path are painted red
        for i in range(0,row):
            for j in range (0,col):
                if i==sp[k].x and j==sp[k].y:
                    maze[i][j]="5"
    print("Optimal Path:")
    for i in range (len(sp)-1,-1,-1):
        print("X=",sp[i].x," ","Y=",sp[i].y)
    t2=t.time()
    tot=t2-t1
    print("Time for execution:",tot)
    print("Length of path:",len(sp))
    maze[exit_node.x][exit_node.y]='4'
    draw_canvas(canvas, maze)
    root.mainloop()

if __name__ == '__main__':
    main()