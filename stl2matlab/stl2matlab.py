#!/usr/bin/env python
import scipy.io as sio

filename = 'solidblock100.stl'
f = open(filename,'r')
line = 'trash'

points = {'x':[],'y':[],'z':[]}

while(line != 'endsolid'):
    line = f.readline()
    splits = line.split(' ')

    for i in xrange(0,len(splits)):
        if(splits[i]=='vertex'):
            points['x'].append(eval(splits[i+1]))
            points['y'].append(eval(splits[i+2]))
            points['z'].append(eval(splits[i+3]))            

sio.savemat('points.mat',points)
