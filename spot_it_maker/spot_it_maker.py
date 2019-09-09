import os
import numpy as np

class SpotItMaker():

    def __init__(self,source_dir,out_dir):
        self.source_dir = source_dir
        self.out_dir = out_dir
        self.files = os.listdir(source_dir)
        self.num_pics = len(self.files)
        if(num_pics>57):
            self.num_pics = 57

        self.card_height = 800
        self.card_width = 800
        self.blank_card = 255*np.ones((self.card_height,self.card_width,3), np.uint8)

    def create_one_card(self,card_number):

        imgs = self.get_images_for_card(card_number)

        card = -1*np.ones((1000,1000,3))
        for i in xrange(0,len(images)):
            img = imgs[i]

            #Do something to get random variation in scaling
            img = self.scale_image_to_have_max_dimension(img,maxdim)
            img = self.randomly_rotate_image(img)
            img = self.randomly_place_image(img)
            #img is now the same size as card, but has -1 everywhere except the image

            while(self.has_collision(img,card)):
                img = self.scale_image_to_have_max_dimension(img,maxdim)
                img = self.randomly_rotate_image(img)
                img = self.randomly_place_image(img)

            card = alpha*card + beta*img

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
            cards.append(set([i * p + j for j in range(p)] + [p * p]))
        for i in range(min_factor):
            for j in range(p):
                cards.append(set([k * p + (j + i * k) % p
                    for k in range(p)] + [p * p + 1 + i]))

        cards.append(set([p * p + i for i in range(min_factor + 1)]))
        return cards, p * p + p + 1

if __name__=='__main__':
    source_folder = './spot_it_pics/'
    destination_folder = './spot_it_cards/'

    maker = SpotItMaker(source_folder,destination_folder)

    maker.create_cards()
    
