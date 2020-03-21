'''
Prose Answers:

I choose to use a Median shift algorithm to cluster the points. I chose
this because using a median is generally more robust to outliers than
choosing a mean. I considered K-means clustering, but since we did not
know beforehand how many clusters to expect, I looked for other algorithms.
Median-shift starts with many equally spaced points and then as they converge
to clusters they are combined. Below is a very simple pseudocode of my median-shift
algorithm.

Median-Shift Clustering algorithm:
- initialize n uniformly spaced median points within data range
- choose a window size (r)
- every median point finds all data points within a distance r of itself
- every median point moves itself to the median of the data points within its window
- when two median points overlap, the one which contains more points is conserved
- convergence is acheived when median points stop moving (or max iterations are reached)

Because we are working with polar coordinates it is necessary to take into account angle
wrapping. I did not want a data point at 5 degrees to be considered 'far' from one at 
355 degrees. To get around this problem I took the polar coordinates (I was only given theta)
and converted them into cartesian coordinates. I then calculated distance based on cartesian
distance. I think this approach would work well for the case where r is given.

The solutions given by my code are well within 20 degrees of the given solutions. They are
given below, as well as the python code I used to run my algorithm. Given more time, I'm sure
the python code could be organized better and be made to run faster. If I really wanted it to run
fast though, I would probably implement this algorithm in C++.

My solution for data set 0:

[ 77.2  179.75 344.2 ]

My solution for data set 1:

[ 49.3 221.1 293.3]

My solution for data set 2:

[ 61.1 189.4 285.4   5. ]

My solution for data set 3:

[ 95.5 165.9 230.7   7.9]

My solution for data set 4:

[ 23.7  118.3  190.55 252.8  323.4 ]


'''
import numpy as np
import matplotlib.pyplot as plt

def distance(pt1,pt2):
    x1 = np.cos(pt1*np.pi/180)
    y1 = np.sin(pt1*np.pi/180)
    x2 = np.cos(pt2*np.pi/180)
    y2 = np.sin(pt2*np.pi/180)
    dist = np.array([x2-x1,y2-y1])
    return np.linalg.norm(dist)
        
class MedianShiftClusterer():

    def __init__(self,data,r,num_medians=10):
        
        self.data = data
        self.r = r        

        self.medians = np.linspace(np.min(data),np.max(data),num_medians)
        self.points_per_median = np.zeros(len(self.medians))

    def cluster(self):

        i = 0
        while i < len(self.medians):            
            
            median_pt = self.medians[i]
            points_within_r = []
            
            for data_pt in self.data:
                if distance(median_pt, data_pt) < self.r:
                    points_within_r.append(data_pt)

            new_median = np.median(points_within_r)
            self.points_per_median[i] = len(points_within_r)
            
            if not np.isnan(new_median):
                self.medians[i] = new_median
                i = i+1
            else:
                self.medians = np.delete(self.medians,i,0)
                self.points_per_median = np.delete(self.points_per_median,i,0)                

    def combine_medians(self):
        i = 1
        while i < len(self.medians):
            if distance(self.medians[i], self.medians[i-1]) < self.r:
                which_to_delete = np.argmin(self.points_per_median[i-1:i+1])
                self.medians = np.delete(self.medians,i-1+which_to_delete,0)
                self.points_per_median = np.delete(self.points_per_median,i-1+which_to_delete,0)
            else:
                i = i+1

        if distance(self.medians[0], self.medians[-1]) < self.r:
            which_to_delete = np.argmin([self.points_per_median[0],self.points_per_median[-1]])
            if which_to_delete==0:
                self.medians = np.delete(self.medians,0,0)
                self.points_per_median = np.delete(self.points_per_median,0,0)
            else:
                self.medians = np.delete(self.medians,len(medians),0)
                self.points_per_median = np.delete(self.points_per_median,len(medians),0)
                            

    def do_iteration(self):
        self.cluster()
        self.combine_medians()
        



if __name__=='__main__':

    data = np.genfromtxt('input4.txt',delimiter=',')

    clusterer = MedianShiftClusterer(data,.77)

    for i in xrange(0,10):
        clusterer.do_iteration()
        print clusterer.medians

