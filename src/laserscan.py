import numpy as np
import cv2

def score(guessPos, guessRot, worldmap, wmscale, wmorigin, scan):
    matrx = rangeMatrix(np.array(scan.ranges), scan.angle_min, scan.angle_max)
    phi, r = np.hsplit(matrx, 2)
    x = (np.cos(phi+guessRot) * r) + guessPos[0]
    y = (np.sin(phi+guessRot) * r) + guessPos[1]
    h = np.rint(-y * wmscale  + wmorigin[0])
    w = np.rint(x * wmscale + wmorigin[1])

    score = 0
    for i in range(r.shape[0]):
        if not np.isnan(r[i,0]):
            if h[i,0]>=0 and w[i,0] >=0 and h[i,0]<worldmap.shape[0] and w[i,0]<worldmap.shape[1]:
                score += worldmap[int(h[i,0]), int(w[i,0])]
    score /= r.shape[0]
    score = 1/(1+np.exp(-score)) # logistic function: output value between 0 and 1
    return score

def rangeMatrix(ranges, angle_min, angle_max):
    return np.c_[np.linspace(angle_min, angle_max, ranges.shape[0]), ranges]


################

def ranges2cart(ranges, range_min, range_max, angle_min, angle_increment):
    imgsize = 100
    rangestep = range_max/imgsize
    ranges = np.array(ranges)
    r, phi = np.mgrid[0:range_max:rangestep,-np.pi:np.pi:angle_increment]
    phimin_index = int((angle_min+np.pi)/angle_increment)
    ranges2d = np.zeros(r.shape)
    ranges2d[:,phimin_index:phimin_index+ranges.shape[0]]=ranges
    ranges2d[np.isnan(ranges2d)] = range_max*2
    v = np.zeros(r.shape)
    v[r>ranges2d] = 0
    v[r<ranges2d] = -1
    v[r<range_min] = 0
    v[(ranges2d!=0) & (ranges2d!=range_max) & (np.abs(r-ranges2d)<0.1)] =1
    v *= np.cos(r/range_max)
    # v *= np.cos(2*phi)
    dst = cv2.linearPolar(v.T, (imgsize,imgsize),imgsize,cv2.WARP_FILL_OUTLIERS | cv2.WARP_INVERSE_MAP)
    dst2 = np.zeros((imgsize*2,imgsize*2))
    dst2[:,imgsize:] = dst[:imgsize*2,:][:,::-1] 
    return dst2

def ranges2mgrid(ranges, range_min, range_max, angle_min, angle_increment):
    imgsize = 100
    rangestep = range_max/imgsize
    ranges = np.array(ranges)
    r, phi = np.mgrid[0:range_max:rangestep,-np.pi:np.pi:angle_increment]
    return r,phi