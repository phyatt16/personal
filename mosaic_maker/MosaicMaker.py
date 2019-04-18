import cv2
import os
import numpy as np
from copy import deepcopy
import pickle

class MosaicMaker():

    def __init__(self,big_picture,small_pic_folder='./'):

        self.bigPic = cv2.imread(big_picture)
        self.bigHeight = self.bigPic.shape[0]
        self.bigWidth = self.bigPic.shape[1]
        self.aspectRatio = float(self.bigHeight)/self.bigWidth
        
        self.smallPics = []
        self.smallPicAvgs = []
        self.bigPicAvgs = []        
        pictureStrings = os.listdir(small_pic_folder)
        self.numPics = len(pictureStrings)

        self.numCellsPerSide = np.int(np.sqrt(self.numPics))
        # self.numCellsPerSide = 25
        self.pixelsPerCellHeight = self.bigHeight/self.numCellsPerSide
        self.pixelsPerCellWidth = self.bigWidth/self.numCellsPerSide

        print "cell width in pixels: ",self.pixelsPerCellWidth
        print "cell height in pixels: ",self.pixelsPerCellHeight

        for i in xrange(0,self.numPics):
            pic = cv2.imread(small_pic_folder+'/'+pictureStrings[i])
            smallPic = cv2.resize(pic,(self.pixelsPerCellWidth,self.pixelsPerCellHeight))
            avg = np.mean(np.mean(pic,axis=0),axis=0)
            self.smallPics.append(smallPic)
            self.smallPicAvgs.append(avg)
            
            print "loaded image ",i+1," of ",self.numPics
            
        self.newImage = deepcopy(self.bigPic*0)

        
    def find_color_swatches_for_big_picture(self,alpha=.5):
        irange = range(0,self.numCellsPerSide)
        jrange = range(0,self.numCellsPerSide)
        np.random.shuffle(irange)
        np.random.shuffle(jrange)
        for i in irange:
            for j in jrange:
                bigPictureCell = self.bigPic[self.pixelsPerCellHeight*i:self.pixelsPerCellHeight*(i+1),self.pixelsPerCellWidth*j:self.pixelsPerCellWidth*(j+1)]

                avgs = np.mean(np.mean(bigPictureCell,axis=0),axis=0)
                self.bigPicAvgs.append(avgs)

                nearestPic = self.find_nearest_small_image(avgs)

                # self.newImage[self.pixelsPerCellHeight*i:self.pixelsPerCellHeight*(i+1),self.pixelsPerCellWidth*j:self.pixelsPerCellWidth*(j+1)] = avgs

                self.newImage[self.pixelsPerCellHeight*i:self.pixelsPerCellHeight*(i+1),self.pixelsPerCellWidth*j:self.pixelsPerCellWidth*(j+1)] = (1-alpha)*nearestPic + (alpha)*avgs*np.ones(nearestPic.shape)
        

    def find_nearest_small_image(self,avg,reuse=False):
        idx = np.argmin(np.linalg.norm(avg-self.smallPicAvgs,axis=1))
        nearest = self.smallPics[idx]
        if reuse==False:        
            del self.smallPics[idx]
            del self.smallPicAvgs[idx]
        return nearest

    def apply_big_picture_to_new_image(self,alpha = .5,savename='mosaic.png'):
        for i in xrange(0,self.bigWidth):
            for j in xrange(0,self.bigHeight):
                self.newImage[j,i] = (1.0-alpha)*self.newImage[j,i] + alpha*self.bigPic[j,i]
        cv2.imwrite(savename,self.newImage)
        cv2.waitKey(0)
        

if __name__=='__main__':
    # mm = MosaicMaker('./big_picture.jpg','./wedding_and_honeymoon2')
    # mm = MosaicMaker('./big2.jpg','./trip_photos')
    # mm = MosaicMaker('./big5.jpg','./trip_photos')    
    # mm.find_color_swatches_for_big_picture(alpha=0)
    # mm.apply_big_picture_to_new_image(alpha=.6,savename='mosaic5.png')

    mm = MosaicMaker('./big_picture.jpg','./trip_photos')    
    mm.find_color_swatches_for_big_picture(alpha=0)
    for i in xrange(0,50):
        mm.apply_big_picture_to_new_image(alpha=.05,savename='mosaic2_alpha_'+str(i)+'.png')
    
        
            
