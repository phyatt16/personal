import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import pickle

class Image2CoverageMap():
    def __init__(self,image_filepath):
        self.filename = image_filepath
        self.color_image = cv2.imread(image_filepath)

        max_dim = min(max(self.color_image.shape),1000)
        self.color_image = self.scale_image_to_have_max_dimension(self.color_image,max_dim)
        self.color_image = cv2.bilateralFilter(self.color_image,10,200,200)
        
        self.edges = cv2.Canny(self.color_image,50,100)

        cv2.imshow('Edges',self.edges)
        cv2.waitKey(100)

    def scale_image_to_have_max_dimension(self,img,maxdim):
        new_xsize = img.shape[0]*maxdim/float(np.max(img.shape))
        new_ysize = img.shape[1]*maxdim/float(np.max(img.shape))
        return cv2.resize(img,(int(new_ysize),int(new_xsize)))
    
class ToolPathPlanner():
    def __init__(self,image_path,tool_radius):

        self.x = 0
        self.y = 0
        self.z = 0
        
        self.img_converter = Image2CoverageMap(image_path)
        self.image = self.img_converter.edges
        self.xmin = 0
        self.ymin = 0
        self.xmax = self.image.shape[0]-1
        self.ymax = self.image.shape[1]-1
        
        self.coverage_map = np.zeros(self.image.shape)
        self.to_be_covered_map = self.image-self.coverage_map

    def visualize_coverage(self,coverage_map,fignum=1):
        cv2.imshow(str(fignum),coverage_map)
        cv2.waitKey(100)        

    def find_nearest_pocket(self,x,y):
        
        found = False
        radius = 1
        
        while found == False:
            radius += 1
            # print radius
            
            xlower = np.max([x-radius,self.xmin])
            ylower = np.max([y-radius,self.ymin])
            xupper = np.min([x+radius,self.xmax+1])
            yupper = np.min([y+radius,self.ymax+1])            

            search_region = self.to_be_covered_map[xlower:xupper,ylower:yupper]
            region_max = np.max(self.to_be_covered_map[xlower:xupper,ylower:yupper])

            if region_max > 0:
                found = True
                
                numCols = np.shape(search_region)[1]
                idx = np.argmax(search_region)
                r = (idx-idx%numCols)/numCols
                c = np.argmax(search_region) - r*numCols

        return xlower + r, ylower + c
        

    def make_xy_in_bounds(self,x,y,theta,xmin,xmax,ymin,ymax):
        if(x > xmax or x < xmin):
            x = x - np.sin(theta)
        if(y > ymax or y < ymin):
            y = y - np.cos(theta)

        return int(round(x)),int(round(y))

    def wall_follow_cw(self,x,y,theta,cmap):
        thetas = [3*np.pi/4, np.pi/2,np.pi/4, 0,-np.pi/4, -np.pi/2, -3*np.pi/4]
        for t in thetas:
            xnext = x + np.sin(theta+t)
            ynext = y + np.cos(theta+t)
            
            xnext,ynext = self.make_xy_in_bounds(xnext,ynext,theta+t,self.xmin,self.xmax,self.ymin,self.ymax)
            if((xnext!=x or ynext!=y) and cmap[xnext,ynext]>0):
                return xnext, ynext, theta+t

        return x,y,theta+t
            
    def fill_pocket(self,pocket_x,pocket_y,waypoints):
        x = pocket_x
        y = pocket_y

        xlast = waypoints[-1][0]
        ylast = waypoints[-1][1]
        
        theta = np.round(np.arctan2(x-xlast,y-ylast)/np.pi*4.0)*np.pi/4.0
        pocket_waypoints = []
        
        while self.coverage_map[x,y] != self.image[x,y]:
            self.coverage_map[x,y] = 255
            self.to_be_covered_map = self.image-self.coverage_map
            
            x,y,theta = self.wall_follow_cw(x,y,theta,self.to_be_covered_map)
            pocket_waypoints.append((x,y,0))


        if(len(pocket_waypoints)>10):
            waypoints.append((pocket_x,pocket_y,1))
            waypoints.append((pocket_x,pocket_y,0))
            
            for pt in pocket_waypoints:
                waypoints.append(pt)

            return waypoints,x,y
        else:
            return waypoints,pocket_x,pocket_y

    def write_waypoints_to_file(self,filename):
        with open(filename,'wb') as f:
            pickle.dump(self.waypoints,f)

    def read_waypoints_from_file(self,filename):
        with open(filename,'rb') as f:
            self.waypoints = pickle.load(f)
            
    
    def generate_g_code(self,waypoints):
        pass

    def preview_image_from_waypoints(self):
        img = np.zeros(self.coverage_map.shape)
        for pt in self.waypoints:
            if(pt[2]==0):
                img[pt[0],pt[1]] = 255

        self.visualize_coverage(img,fignum=10)


    def plan_2d(self):
        
        x = self.x
        y = self.y
        z = self.z
        zup = 1
        zdown = 0
        self.waypoints = [[x,y,z]]
        numPockets = 0
        
        while np.linalg.norm(self.coverage_map-self.image)>0:
            
            pocket_x,pocket_y = self.find_nearest_pocket(x,y)

            len_before_filling = len(self.waypoints)
            self.waypoints,x,y = self.fill_pocket(pocket_x,pocket_y,self.waypoints)
            if(len(self.waypoints)>len_before_filling):
                numPockets += 1

            # self.visualize_coverage(self.coverage_map,fignum=1)
            self.preview_image_from_waypoints()
            
        self.preview_image_from_waypoints()
        
        print "done"
        print numPockets," pockets"

        
if __name__=='__main__':
    
    # a = Image2CoverageMap('testpic.jpg')
    # b = Image2CoverageMap('8.png')
    

    filenames = ['random_shapes2.png','testpic.jpg','hyatt_sibs.jpg','ruthie.jpg','8.png','words.png','wagon_wheel.jpg']
    # filenames = ['wagon_wheel.jpg']
   
    tool_diameter = 1

    for filename in filenames:
        planner = ToolPathPlanner(filename,tool_diameter)    
        planner.plan_2d()
        planner.write_waypoints_to_file(filename[0:-4])
        cv2.waitKey(-1)    


    for filename in filenames:
        planner = ToolPathPlanner(filename,tool_diameter)            
        planner.read_waypoints_from_file(filename[0:-4])
        planner.preview_image_from_waypoints()
        cv2.waitKey(-1)
    
    # g_code = planner.generate_g_code()
