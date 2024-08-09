import numpy as np
from scipy.spatial.transform import Rotation as R

class DH_ARM:
    def __init__(self) -> None:
        # delta, theta, r, alpha
        self._DH_table = np.mat([[-0.064, 0, 0, np.pi/2],
                                 [0, -np.pi/2, 0.279, 0],
                                 [0, 0, 0.279, 0],
                                 [-0.076, -np.pi/2, 0, -np.pi/2],
                                 [0.068, np.pi/2, 0, -np.pi/2],
                                 [0.0275, 0, 0, 0]])

    def _get_T(self, single_dh):
        d = single_dh[0]
        th = single_dh[1]
        r = single_dh[2]
        a = single_dh[3]
        T = np.mat([[np.cos(th), -np.sin(th)*np.cos(a), np.sin(th)*np.sin(a), r*np.cos(th)],
                    [np.sin(th), np.cos(th)*np.cos(a), -np.cos(th)*np.sin(a), r*np.sin(th)],
                    [0, np.sin(a), np.cos(a), d],
                    [0, 0, 0, 1]])
        return T

    def get_idx_link_T(self, idx):
        t = self._DH_table[idx, :].tolist()[0]
        T = self._get_T(t)

        rot = R.from_matrix(T[:3, :3]).as_euler("xyz", degrees=False)
        trans = T[:3, 3].T.tolist()[0]
        
        # print(T)
        print(rot, trans)
        return T

arm = DH_ARM()
for i in range(6):
    arm.get_idx_link_T(i)