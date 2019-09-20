import os
import numpy as np
import cv2

class SpotItMaker():

    def __init__(self,source_dir,out_dir):
        self.source_dir = source_dir
        self.out_dir = out_dir
        self.files = os.listdir(source_dir)

    def scale_image_to_have_max_dimension(self,img,maxdim):
        new_xsize = img.shape[0]*maxdim/float(np.max(img.shape))
        new_ysize = img.shape[1]*maxdim/float(np.max(img.shape))
        return cv2.resize(img,(int(new_ysize),int(new_xsize)))

    def get_random_placed_image_corners(self,img,card):
        x_point = np.random.randint(0,card.shape[0]-img.shape[0])
        y_point = np.random.randint(0,card.shape[1]-img.shape[1])

        return x_point,x_point+img.shape[0],y_point,y_point+img.shape[1]

    def randomly_rotate_image(self,img):
        return cv2.rotate(img,np.random.randint(0,3))

    def create_one_card(self,images):
        card_size = 300
        img_size = 100
        card = -1*np.ones((card_size,card_size,3),np.uint8)
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
                img = self.scale_image_to_have_max_dimension(img,img_size+np.random.randint(-img_size*.5,img_size*.5))                
                a,b,c,d = self.get_random_placed_image_corners(img,card)
                count += 1

                if(count==1000):
                    i=-1
                    fits = False
                    card = -1*np.ones((card_size,card_size,3),np.uint8)
                    break

            i += 1                
            if fits:
                card[a:b,c:d] = img
                
        # Make card a type uint8 array
        card[card==-1]=255
        card = np.array(card,np.uint8)
        return card

        
    def get_image_numbers_for_cards(self,p):
        for min_factor in range(2, 1 + int(p ** 0.5)):
            if p % min_factor == 0:
                break
        else:
            min_factor = p
        cards = []
        for i in range(p):
            cards.append(([i * p + j for j in range(p)] + [p * p]))
        for i in range(min_factor):
            for j in range(p):
                cards.append(([k * p + (j + i * k) % p
                    for k in range(p)] + [p * p + 1 + i]))

        cards.append(([p * p + i for i in range(min_factor + 1)]))
        return cards
    
    def create_cards(self):
        img_number_array = self.get_image_numbers_for_cards(7)

        self.cards = []
        for i in xrange(0,len(img_number_array)):
            self.cards.append(self.create_one_card(img_number_array[i]))
            print "Made card: ",i

    def preview_cards(self):
        for i in xrange(0,len(self.cards)):
            cv2.imshow('card'+str(i),self.cards[i])
            cv2.waitKey(1)
        cv2.waitKey(0)


if __name__=='__main__':
    source_folder = './spot_it_pics/'
    destination_folder = './spot_it_cards/'

    maker = SpotItMaker(source_folder,destination_folder)

    maker.create_cards()
    maker.preview_cards()

    
