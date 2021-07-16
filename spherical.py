import numpy as np
import math

class SphericalIntrinsic():
    # define a sphere class contain its width and height
    def __init__(self, w, h):
        super(SphericalIntrinsic).__init__()
        self.w = w
        self.h = h
        self.size = max(w, h)
    # 
    def cam2ima(self, p):
        p[:, 0] = p[:, 0] * self.size + self.w / 2.0
        p[:, 1] = p[:, 1] * self.size + self.h / 2.0
        p[:, 0] = np.clip(p[:, 0], 0, self.w - 1)
        p[:, 1] = np.clip(p[:, 1], 0, self.h - 1)
        return p

    def img2cam(self, p):
        p[:, 0] = (p[:, 0] - self.w / 2.0) / self.size
        p[:, 1] = (p[:, 1] - self.h / 2.0) / self.size
        return p

    def reproject(self, p):
        '''
        reproject a 2D image point to 3D space
        '''
        p = self.ima2cam(p)
        lon = p[:, 0] * 2 * math.pi
        lat = -p[:, 1] * 2 * math.pi
        temp = np.zeros(lon.shape[0], 3)
        temp[:, 0] = np.cos(lat) * np.sin(lon)
        temp[:, 1] = -np.sin(lat)
        temp[:, 2] = np.cos(lat) * np.cos(lon)
        return temp

    def project(self, p):
        '''
        project a 3D point to 2D image
        '''

        lon = np.arctan2(p[:, 0], p[:, 2])
        lat = np.arctan2(-p[:, 1], np.hypot(p[:, 0], p[:, 2]))
        temp = np.zeros((p.shape[0], 2),dtype=np.float32)
        temp[:, 0] = lon / (2*math.pi)
        temp[:, 1] = -lat / (2*math.pi)

        return self.cam2ima(temp)
