#!/usr/bin/env python
import os
import numpy as np
import cv2


class SpotItMaker():

    def __init__(self,source_dir,out_dir):
        self.source_dir = source_dir
        self.out_dir = out_dir
        self.files = os.listdir(source_dir)
        num_pics_in_folder = len(self.files)
        print "Found ",num_pics_in_folder," pictures in folder"
        prime_numbers = [1,2,3,5,7,11,13,17,19,23,29,31]
        for i in range(0,len(prime_numbers)):
            number_of_pics = len(self.get_image_numbers_for_cards(prime_numbers[i]))
            if number_of_pics > num_pics_in_folder:
                i -= 1
                break
        self.img_number_array = self.get_image_numbers_for_cards(prime_numbers[i])
        print "Making ",len(self.img_number_array)," cards with ",prime_numbers[i]+1," images each"

    def scale_image_to_have_max_dimension(self,img,maxdim):
        new_xsize = img.shape[0]*maxdim/float(np.max(img.shape))
        new_ysize = img.shape[1]*maxdim/float(np.max(img.shape))
        return cv2.resize(img,(int(new_ysize),int(new_xsize)))

    def get_random_placed_image_corners(self,img,card):
        radius = card.shape[0]/2
        center = card.shape[0]/2        

        x_point = 1000
        y_point = 1000
        while (np.sqrt((x_point-center)**2 + (y_point-center)**2)>=radius):
            x_point = center + np.random.randint(-(card.shape[0]),(card.shape[0]-img.shape[0]-center))
            y_point = center + np.random.randint(-(card.shape[1]),(card.shape[1]-img.shape[1]-center))

        return x_point,x_point+img.shape[0],y_point,y_point+img.shape[1]

    def randomly_rotate_image(self,img):
        return cv2.rotate(img,np.random.randint(0,3))

    def draw_circle_on_card(self,img):
        diameter = img.shape[0]
        radius = diameter/2
        color = (np.random.randint(0,255),np.random.randint(0,255),np.random.randint(0,255))
        img = cv2.circle(img,(img.shape[0]/2,img.shape[1]/2),radius,color,5)
        return img

    def create_one_card(self,images):
        card_size = 350
        img_size = 80
        card = -1*np.ones((card_size,card_size,3),np.uint8)
        card = self.draw_circle_on_card(card)
        i = 0
        while i < len(images):
            img = cv2.imread(self.source_dir+self.files[images[i]])
            img = self.randomly_rotate_image(img)            
            img = self.scale_image_to_have_max_dimension(img,img_size+np.random.randint(-img_size*.5,img_size*.5))
            a,b,c,d = self.get_random_placed_image_corners(img,card)

            count = 0
            fits = True
            while np.max(card[a:b,c:d])>-1:
                img = cv2.imread(self.source_dir+self.files[images[i]])
                img = self.randomly_rotate_image(img)                
                img = self.scale_image_to_have_max_dimension(img,img_size+np.random.randint(-img_size*.25,img_size*.25))                
                a,b,c,d = self.get_random_placed_image_corners(img,card)
                count += 1

                if(count==1000):
                    i=-1
                    fits = False
                    card = -1*np.ones((card_size,card_size,3),np.uint8)
                    card = self.draw_circle_on_card(card)                    
                    break

            i += 1                
            if fits:
                card[a:b,c:d] = img
                
        # Make card a type uint8 array
        card[card==-1]=255
        card = np.array(card,np.uint8)
        return card

        
    def get_image_numbers_for_cards(self,p):
        cards = []
        for i in range(p):
            card = []
            for j in range(p):
                card.append(i * p + j)
            cards.append(card+[p*p])
            # cards.append(([i * p + j for j in range(p)] + [p * p]))
        for i in range(p):
            for j in range(p):
                card = []
                for k in range(p):
                    card.append(k * p + (j + i * k) % p)
                cards.append(card+[p*p+1+i])
                # cards.append(([k * p + (j + i * k) % p for k in range(p)] + [p * p + 1 + i]))
        card = []
        for i in range(p+1):
            card.append(p * p + i )
        cards.append(card)
        # cards.append(([p * p + i for i in range(p + 1)]))
        return cards
    
    def create_cards(self):

        self.cards = []
        for i in xrange(0,len(self.img_number_array)):
            self.cards.append(self.create_one_card(self.img_number_array[i]))
            print "Made card: ",i+1

    def preview_cards(self):
        for i in xrange(0,len(self.cards)):
            cv2.imshow('card'+str(i),self.cards[i])
            cv2.waitKey(1)
        cv2.waitKey(0)
        
    def save_cards(self):
        for i in xrange(0,len(self.cards)):
            cv2.imwrite(self.out_dir+'card_'+str(i)+'.png',self.cards[i])


if __name__=='__main__':
    # source_folder = './alphabet_pics/'
    # destination_folder = './alphabet_spot_it/'        
    source_folder = './spot_it_pics/'    
    destination_folder = './spot_it_cards/'


    maker = SpotItMaker(source_folder,destination_folder)

    maker.create_cards()
    # maker.preview_cards()
    maker.save_cards()    

    
