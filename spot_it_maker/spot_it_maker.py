import os
import numpy as np
import cv2

class SpotItMaker():

    def __init__(self,source_dir,out_dir):
        self.source_dir = source_dir
        self.out_dir = out_dir
        self.files = os.listdir(source_dir)
        self.num_pics = len(self.files)
        if(self.num_pics>57):
            self.num_pics = 57

        self.card_height = 800
        self.card_width = 800
        self.blank_card = 255*np.ones((self.card_height,self.card_width,3), np.uint8)

    def bgr_to_bgra(self,img):
        b_channel, g_channel, r_channel = cv2.split(img)
        alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype)*0
        img_BGRA = cv2.merge((b_channel, g_channel, r_channel, alpha_channel))
        return img_BGRA

    def scale_image_to_have_max_dimension(self,img,maxdim):
        new_xsize = img.shape[0]*maxdim/float(np.max(img.shape))
        new_ysize = img.shape[1]*maxdim/float(np.max(img.shape))
        return cv2.resize(img,(int(new_xsize),int(new_ysize)))

    def randomly_place_image(self,img,card):

        x_point = np.random.randint(0,card.shape[0]-img.shape[0])
        y_point = np.random.randint(0,card.shape[1]-img.shape[1])

        ## TODO - CHECK FOR IMAGE COLLISION

        card[x_point:x_point+img.shape[0],y_point:y_point+img.shape[1]] = img

        return card

    def create_one_card(self,images):

        card = 0*np.ones((1000,1000,3),np.uint8)
        for i in xrange(0,len(images)):
            img = cv2.imread(self.source_dir+self.files[images[i]])

            #Do something to get random variation in scaling
            img = self.scale_image_to_have_max_dimension(img,250+np.random.randint(-150,150))
            # img = self.randomly_rotate_image(img)
            card = self.randomly_place_image(img,card)
            #img is now the same size as card, but has -1 everywhere except the image

            # while(self.has_collision(img,card)):
            #     img = self.scale_image_to_have_max_dimension(img,maxdim)
            #     img = self.randomly_rotate_image(img)
            #     img = self.randomly_place_image(img,card)

            # card = alpha*card + beta*img

        # Make card a type uint8 array
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

if __name__=='__main__':
    source_folder = './spot_it_pics/'
    destination_folder = './spot_it_cards/'

    maker = SpotItMaker(source_folder,destination_folder)

    maker.create_cards()
    
