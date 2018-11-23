import numpy as np
import cv2

def astar() :
    #file_addr = '/home/kimtaehun/mapLounge3.jpg'
    file_addr = '/home/kimtaehun/mapLounge5.jpg'
    totalmap = cv2.imread(file_addr, cv2.IMREAD_GRAYSCALE)

    w = totalmap.shape[0]
    l = totalmap.shape[1]

    for i in range(0,w):
        for j in range(0,l):
            if totalmap[i,j] > 125 :
                totalmap[i,j] = 0

            else :
		totalmap[i,j] = 1

    return totalmap
