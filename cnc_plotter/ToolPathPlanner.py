import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

class Image2CoverageMap():
    def __init__(self,image_filepath):
        self.color_image = cv2.imread(image_filepath)
        self.color_image = self.scale_image_to_have_max_dimension(self.color_image,400)
        self.bw_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)
        self.edges = cv2.Canny(self.color_image,100,150)
        self.thresh_image = cv2.bitwise_not(cv2.adaptiveThreshold(self.bw_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2))
        
        cv2.imshow('Color image',self.color_image)
        cv2.imshow('Greyscale image',self.bw_image)
        cv2.imshow('Edges',self.edges)
        cv2.imshow('Thresholded',self.thresh_image)        

        cv2.waitKey(1)

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
        
        self.image = self.img_converter.thresh_image
        # self.image = self.img_converter.edges
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

    def make_xy_in_bounds(self,x,y,theta,xmin,xmax,ymin,ymax):
        if(x > xmax or x < xmin):
            x = x - np.sin(theta)
        if(y > ymax or y < ymin):
            y = y - np.cos(theta)

        return int(round(x)),int(round(y))

    # TODO - Streamline this...maybe a for loop over thetas with a break in case it finds something
    # Make the thetas increment by 45 degrees to include diagonal travel
    def wall_follow_cw(self,x,y,theta,cmap):
        thetas = [3*np.pi/4, np.pi/2,np.pi/4, 0,-np.pi/4, -np.pi/2, -3*np.pi/4]
        # thetas = [np.pi/2, np.pi, -np.pi/2, 0]        
        for t in thetas:
            xnext = x + np.sin(theta+t)
            ynext = y + np.cos(theta+t)
            
            xnext,ynext = self.make_xy_in_bounds(xnext,ynext,theta + t,self.xmin,self.xmax,self.ymin,self.ymax)
            if((xnext!=x or ynext!=y) and cmap[xnext,ynext]>0):
                return xnext, ynext, theta+t

        return x,y,theta
            
    
    # def wall_follow_cw(self,x,y,theta,cmap):
    #     # Check to the left
    #     xnext = x + np.cos(theta + np.pi/2.0)
    #     ynext = y + np.sin(theta + np.pi/2.0)
    #     xnext,ynext = self.make_xy_in_bounds(xnext,ynext,theta + np.pi/2.0,self.xmin,self.xmax,self.ymin,self.ymax)

    #     if((xnext!=x or ynext!=y) and cmap[xnext,ynext]>0):
    #         return xnext, ynext, theta+np.pi/2.0

    #     else:
    #         # Check ahead
    #         xnext = x + np.cos(theta)
    #         ynext = y + np.sin(theta)
    #         xnext,ynext = self.make_xy_in_bounds(xnext,ynext,theta,self.xmin,self.xmax,self.ymin,self.ymax)
            
    #         if((xnext!=x or ynext!=y) and cmap[xnext,ynext]>0):
    #             return xnext, ynext, theta

    #         else:
    #             # Check to the right
    #             xnext = x + np.cos(theta - np.pi/2.0)
    #             ynext = y + np.sin(theta - np.pi/2.0)
    #             xnext,ynext = self.make_xy_in_bounds(xnext,ynext,theta - np.pi/2.0,self.xmin,self.xmax,self.ymin,self.ymax)
                
    #             if((xnext!=x or ynext!=y) and cmap[xnext,ynext]>0):
    #                 return xnext, ynext, theta-np.pi/2.0

    #             else:
    #                 # return zeros
    #                 return int(round(x)), int(round(y)), theta


    # TODO - Make it so this actually finds the nearest pocket...
    def find_nearest_pocket(self,x,y):
        search_map = np.ones(self.image.shape)
        # search_map = np.zeros(self.image.shape)        
        search_map[x,y] = 0
        
        found = False
        theta = 0
        
        while found == False:
            # Swirl outward
            # plt.figure(3)
            # plt.clf()
            # self.visualize_coverage(search_map,3)
            xnext,ynext,thetanext = self.wall_follow_cw(x,y,theta,search_map)
            if(xnext!=x or ynext!=y or thetanext!=theta):
                x = xnext
                y = ynext
                theta = thetanext
                search_map[x,y] = 0
            else:
                # print "Resetting search map!"
                # print "x ",x
                # print "y ",y
                # print "theta ",theta                
                search_map = np.ones(self.image.shape)
                # search_map[x,y] = 0
                # x,y,theta = self.wall_follow_cw(x,y,theta,np.ones(search_map.shape))


            # print "x: ",x
            # print "y: ",y            
            # print "theta: ",theta

            if self.coverage_map[x,y] != self.image[x,y]:
                found = True

        # print "Found something"
        return x,y,theta


    # TODO - make it recognize diagonal pixels as part of the same pocket
    def fill_pocket(self,pocket_x,pocket_y,pocket_theta,waypoints):
        x = pocket_x
        y = pocket_y
        theta = pocket_theta

        # thetas = [np.pi/4,np.pi/2, 3*np.pi/4, np.pi, -3*np.pi/4, -np.pi/2, -np.pi/4, 0 ]
        thetas = [3*np.pi/4, np.pi/2,np.pi/4, 0,-np.pi/4, -np.pi/2, -3*np.pi/4]
        # thetas = [np.pi/2,np.pi, 3*np.pi/2, 0, np.pi/4, 3*np.pi/4, 5*np.pi/4, 7*np.pi/4]
        # for test_theta in thetas:
            
        #     xnext = x + np.sin(test_theta)
        #     ynext = y + np.cos(test_theta)
            
        #     xnext,ynext = self.make_xy_in_bounds(xnext,ynext,test_theta,self.xmin,self.xmax,self.ymin,self.ymax)
            
        #     if(self.coverage_map[xnext,ynext] != self.image[xnext,ynext]):
        #         theta = test_theta
        #         break
            
        while self.coverage_map[x,y] != self.image[x,y]:
            self.coverage_map[x,y] = self.image[x,y]
            self.to_be_covered_map = self.image-self.coverage_map
            x,y,theta = self.wall_follow_cw(x,y,theta,self.to_be_covered_map)
            
            waypoints.append((x,y,0))
            # plt.figure(2)
            # plt.clf()
            # self.visualize_coverage(self.coverage_map,2)

        return waypoints,x,y
    
    def generate_g_code(self,waypoints):
        pass

    def plan_2d(self):
        
        x = self.x
        y = self.y
        z = self.z
        zup = 1
        zdown = 0
        waypoints = []
        
        while np.linalg.norm(self.coverage_map-self.image)>0:
            
            # print "finding new pocket"
            pocket_x,pocket_y,pocket_theta = self.find_nearest_pocket(x,y)

            waypoints.append((pocket_x,pocket_y,zup))
            waypoints.append((pocket_x,pocket_y,zdown))

            # print "filling pocket"
            waypoints,x,y = self.fill_pocket(pocket_x,pocket_y,pocket_theta,waypoints)

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
    
        


        
if __name__=='__main__':
    
    # a = Image2CoverageMap('testpic.jpg')
    # b = Image2CoverageMap('8.png')
    

    filename = 'random_shapes2.png'
    # filename = 'testpic.jpg'
    # filename = '8.png'
    tool_diameter = 1

    planner = ToolPathPlanner(filename,tool_diameter)

    # Makes a list of waypoints (x,y,z)
    planner.plan_2d()

    time.sleep(100)
    
    # g_code = planner.generate_g_code()
