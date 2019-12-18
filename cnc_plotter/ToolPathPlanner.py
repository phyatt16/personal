import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

class Image2CoverageMap():
    def __init__(self,image_filepath):
        self.color_image = cv2.imread(image_filepath)
        self.color_image = self.scale_image_to_have_max_dimension(self.color_image,300)
        self.bw_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)
        self.edges = cv2.Canny(self.color_image,150,300)
        # self.thresh_image = cv2.bitwise_not(cv2.adaptiveThreshold(self.bw_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,33,2))
        ret, self.thresh_image = cv2.threshold(self.bw_image,127,255,cv2.THRESH_BINARY_INV)
        self.bw_image = cv2.bitwise_not(cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY))
        
        cv2.imshow('Color image',self.color_image)
        # cv2.imshow('Greyscale image',self.bw_image)
        # cv2.imshow('Edges',self.edges)
        # cv2.imshow('Thresholded',self.thresh_image)        

        cv2.waitKey(10)

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
        
        # self.image = self.img_converter.thresh_image
        # self.image = self.img_converter.bw_image        
        self.image = self.img_converter.edges
        self.xmin = 0
        self.ymin = 0
        self.xmax = self.image.shape[0]-1
        self.ymax = self.image.shape[1]-1
        
        self.coverage_map = np.zeros(self.image.shape)
        self.to_be_covered_map = self.image-self.coverage_map

    def visualize_coverage(self,coverage_map,fignum=1):
        plt.ion()
        plt.matshow(coverage_map,fignum=fignum)
        plt.show()
        plt.pause(.0001)
        # raw_input('')

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

            # print xlower
            # print xupper
            # print ylower
            # print yupper
            # print "search region shape: ",search_region.shape
            # print "image shape: ",self.image.shape            

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
        # thetas = -np.array(thetas)
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

        xlast = waypoints[-3][0]
        ylast = waypoints[-3][1]
        
        theta = np.round(np.arctan2(x-xlast,y-ylast)/np.pi*4.0)*np.pi/4.0
        
        while self.coverage_map[x,y] != self.image[x,y]:
            self.coverage_map[x,y] = self.image[x,y]
            self.coverage_map[x,y] = 255
            self.to_be_covered_map = self.image-self.coverage_map
            
            x,y,theta = self.wall_follow_cw(x,y,theta,self.to_be_covered_map)
            
            waypoints.append((x,y,0))
            
            # plt.figure(2)
            # plt.clf()
            # self.visualize_coverage(self.coverage_map,2)
            # time.sleep(.01)

        return waypoints,x,y
    
    def generate_g_code(self,waypoints):
        pass

    def plan_2d(self):
        
        x = self.x
        y = self.y
        z = self.z
        zup = 1
        zdown = 0
        waypoints = [[x,y,z]]
        numPockets = 0
        
        while np.linalg.norm(self.coverage_map-self.image)>0:
            
            # print "finding new pocket"
            pocket_x,pocket_y = self.find_nearest_pocket(x,y)

            waypoints.append((pocket_x,pocket_y,zup))
            waypoints.append((pocket_x,pocket_y,zdown))

            # print "filling pocket"
            waypoints,x,y = self.fill_pocket(pocket_x,pocket_y,waypoints)
            numPockets += 1

            plt.figure(1)
            plt.clf()
            self.visualize_coverage(self.coverage_map,fignum=1)

            # plt.figure(2)
            # plt.clf()
            # self.to_be_covered_map = self.image-self.coverage_map
            # self.visualize_coverage(self.to_be_covered_map,fignum=2)
            

        plt.figure(1)
        plt.clf()
        self.visualize_coverage(self.coverage_map,fignum=1)
        print "done"
        print numPockets," pockets"
    
        


        
if __name__=='__main__':
    
    # a = Image2CoverageMap('testpic.jpg')
    # b = Image2CoverageMap('8.png')
    

    # filename = 'random_shapes2.png'
    filename = 'testpic.jpg'
    # filename = '8.png'
    # filename = 'words.png'
    # filename = 'fleur.png'
    tool_diameter = 1

    planner = ToolPathPlanner(filename,tool_diameter)

    # Makes a list of waypoints (x,y,z)
    planner.plan_2d()

    time.sleep(100)
    
    # g_code = planner.generate_g_code()
