import numpy as np


class PinholeIntrinsic():
    # pinhole camera intrinsic matrix
    def __init__(self, w, h, focal_length_pix, ppx, ppy):
        super(PinholeIntrinsic).__init__()
        self.K_ = np.array([[focal_length_pix, 0, ppx],
                   [0, focal_length_pix, ppy],
                   [0, 0, 1]])
        self.Kinv_ = np.linalg.inv(self.K_)

    def K(self):
        return self.K_

    def Kinv(self):
        return self.Kinv_

    def focal(self):
        return self.K_[0, 0]

    def principal_point(self):
        return (self.K_[0, 2], self.K_[1, 2])

    def reproject(self, p):
        '''
        reproject a 2D image point to 3D space
        '''
        # homogeneous_point = np.zeros((p.shape[0], 3), dtype=np.float)
        # homogeneous_point[:, 0] = p[:, 0]
        # homogeneous_point[:, 1] = p[:, 1]
        # homogeneous_point[:, 2] = 1.0
        # homogeneous_point= np.transpose(homogeneous_point,(1,0))

        # return np.matmul(self.Kinv_,homogeneous_point)
        p = self.ima2cam(p)
        homogeneous_point = np.zeros((p.shape[0], 3), dtype=np.float)
        print(homogeneous_point)
        homogeneous_point[:, 0] = p[:, 0]
        print(homogeneous_point)
        homogeneous_point[:, 1] = p[:, 1]
        print(homogeneous_point)
        homogeneous_point[:, 2] = 1.0
        print(homogeneous_point)
        homogeneous_point= np.transpose(homogeneous_point,(1,0))
        print(homogeneous_point)
        return homogeneous_point

    def cam2ima(self, p):
        return self.focal() * p + self.principal_point()

    def ima2cam(self, p):
        p[:, 0] = p[:, 0] - self.principal_point()[0]
        p[:, 1] = p[:, 1] - self.principal_point()[1]
        p = p / self.focal()
        return p
