#
# Python UKF based Motion Tracker module of the piCAMTracker package
# Copyright (c) 2019-2020 Axel Barnitzke <barney@xkontor.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holder nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
import numpy as np
from math import sin,cos,pi
from scipy.linalg import block_diag

class ukfFilter(UnscentedKalmanFilter):
    def __init__(self, dt=1.0, w=1.0):
        self.dim_x = 6
        self.dim_z = 2
        self._w = w
        sigma_points = MerweScaledSigmaPoints(n=self.dim_x, alpha=.3, beta=2., kappa=-3)
        super(ukfFilter,self).__init__(dim_x=self.dim_x, dim_z=self.dim_z, fx=self.f_ct, hx=self.h_ca, dt=dt, points=sigma_points)
    
    def f_ca(self, x, dt):
        """ state transition function for a constant accelarating aircraft"""
        dt2 = dt*dt*0.5
        Fa = np.array([[1., dt,  dt2, 0.,  0.,   0.],
                       [0.,  1.,  dt, 0.,  0.,   0.],
                       [0.,  0.,  1., 0.,  0.,   0.],
                       [0.,  0.,  0., 1.,  dt,  dt2],
                       [0.,  0.,  0., 0.,  1.,  dt],
                       [0.,  0.,  0., 0.,  0.,  1.]])
        return np.dot(Fa, x)
    
    def h_ca(self, x):
        """ measurement function """
        #print("x: ", x[[0,3]])
        return x[[0, 3]]
    
    def start_ca(self, std_r=1.0, v=0.1, vstart=None, dt=None):
        if vstart is None:
            self.x = np.array([0.,0.,0.,0.,0.,0.])
        else:
            self.x =np.array(vstart)

        if dt is None:
            dt = self._dt
        else:
            self._dt = dt

        self.Q[0:3, 0:3] = Q_discrete_white_noise(3, dt=dt, var=v)
        self.Q[3:6, 3:6] = Q_discrete_white_noise(3, dt=dt, var=v)
        self.P *= 5000. #np.eye(6) * 5000.
        self.R = np.diag([std_r**2, std_r**2])
        
    '''
    Constant turn rate:
    '''
    def f_ct(self, x, dt, w=None):
        if w is None:
            w = self._w
        wt = w * dt
        w2 = w * w
        sin_wt = sin(wt)
        cos_wt = cos(wt)
        if (abs(w) < 1.e-99):
                          # x       v        a     x        v        a
            Ft = np.array([[1.,     1.,      0.5,  0.,      0.,      0.   ],
                           [0.,  cos_wt,     1.,   0.,      0.,      0.   ],
                           [0.,     0.,    cos_wt, 0.,      0.,      0.   ],
                           [0.,     0.,      0.,   1.,      1.,      0.5  ],
                           [0.,     0.,      0.,   0.,   cos_wt,     1.   ],
                           [0.,     0.,      0.,   0.,      0.,    cos_wt ]])
        else:
                          # x      v             a          x      v            a
            Ft = np.array([[1.,  sin_wt/w, (1.-cos_wt)/w2,  0.,    0.,          0.          ],
                           [0.,  cos_wt,      sin_wt/w,     0.,    0.,          0.          ],
                           [0., -w*sin_wt,     cos_wt,      0.,    0.,          0.          ],
                           [0.,     0.,          0.,        1.,  sin_wt/w, (1.-cos_wt)/w2 ],
                           [0.,     0.,          0.,        0.,  cos_wt,     sin_wt/w     ],
                           [0.,     0.,          0.,        0., -w*sin_wt,     cos_wt     ]])

        new_x = np.dot(Ft,x)
        #print("x: ", x)
        return new_x

    '''
    Constant turn rate: (tangential model)
    '''
    def f_ctt(self, x, dt, w=None):
        if w is None:
            w = self._w
        wt = w * dt
        sin_wt = sin(wt)
        cos_wt = cos(wt)
        if (abs(w) < 1.e-99):
                          # x        vx       ax    y        vy         ay
            Fq = np.array([[1.,      1.,      0.,   0.,      0.,        0. ],
                           [0.,   cos_wt,     0.,   0.,     -sin_wt,    0. ],
                           [0.,      0.,      1.,   0.,      0.,        0. ],
                           [0.,      0.,      0.,   1.,      1.,        0. ],
                           [0.,   sin_wt,     0.,   0.,     cos_wt,     0. ],
                           [0.,      0.,      0.,   0.,      0.,        1. ]])

        else:
                          # x        vx       ax    y        vy         ay
            Fq = np.array([[1.,  sin_wt/w,    0.,   0.,  (1.-cos_wt)/w, 0. ],
                           [0.,   cos_wt,     0.,   0.,     -sin_wt,    0. ],
                           [0.,      0.,      1.,   0.,      0.,        0. ],
                           [0.,(1.-cos_wt)/w, 0.,   1.,     sin_wt/w,   0. ],
                           [0.,   sin_wt,     0.,   0.,     cos_wt,     0. ],
                           [0.,      0.,      0.,   0.,      0.,        1. ]])

        return np.dot(Fq,x)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from time import time
    def f_ca(x, dt):
        """ state transition function for a 
        constant accelarating aircraft"""
        dt2 = (dt*dt)/2.
        Fa = np.array([[1., dt,  dt2, 0.,  0.,   0.],
                    [0.,  1.,  dt, 0.,  0.,   0.],
                    [0.,  0.,  1., 0.,  0.,   0.],
                    [0.,  0.,  0., 1.,  dt,  dt2],
                    [0.,  0.,  0., 0.,  1.,  dt],
                    [0.,  0.,  0., 0.,  0.,  1.]])   
        return np.dot(Fa, x)
    def h_ca(x):
        #print("x:", x)
        return x[[0, 3]]
    N = 20
    r = 3.9
    dphi = pi/N
    zs = []
    track = []
    dt = 2.0/N
    w = -pi*dt*r

    np.random.seed(int(time())&0xff)
    for n in range(0,N):
        x = 100.0 * -cos(n*dphi) + 101.
        y = 100.0 * sin(n*dphi) + 1.
        #print("x: %5.2f y: %5.2f" % (x,y))
        track.append([x,y])
        px = x + np.random.randn()*r
        py = y + np.random.randn()*r
        if n == 0:
            zs.append([x,y])
        else:
            zs.append([px,py])

    points_ca = MerweScaledSigmaPoints(n=6, alpha=.3, beta=2., kappa=-3.0)
    ukf_ca = UnscentedKalmanFilter(dim_x=6, dim_z=2, fx=f_ca, hx=h_ca, dt=dt, points=points_ca)

    ukf_ca.x = np.array([1., 0.0, 0., 1., 1., 0.])
    ukf_ca.R = np.diag([r*r, r*r]) 
    ukf_ca.Q[0:3, 0:3] = Q_discrete_white_noise(3, dt=dt, var=0.02)
    ukf_ca.Q[3:6, 3:6] = Q_discrete_white_noise(3, dt=dt, var=0.01)
    ukf_ca.P *= 5000**2.
    
    ukf = ukfFilter(dt=dt, w=w)
    ukf.start_ca(std_r=r,vstart=[1., 0.0, 0., 1., 1., 0.])
    xs = []
    xs_ca = []
    
    for z in zs:
        ukf.predict(w=w)
        ukf.update(z)
        xs.append(ukf.x)
        ukf_ca.predict()
        ukf_ca.update(z)
        xs_ca.append(ukf_ca.x)
        print("px:%6.2f > xx:%6.2f  py:%6.2f > xy:%6.2f" % (z[0],ukf.x[0],z[1],ukf.x[3]))

    xs = np.array(xs)
    xs_ca = np.array(xs_ca)
    zs = np.array(zs)
    track = np.array(track)
    
    plt.subplot(131)
    plt.title('ukf_ct')
    plt.plot(track[:, 0], track[:, 1], '--r')
    plt.scatter(zs[:, 0], zs[:, 1], marker='+')
    plt.plot(xs[:, 0], xs[:, 3], 'k')

    plt.subplot(132)
    plt.title('ukf_ca')
    plt.plot(track[:, 0], track[:, 1], '--r')
    plt.scatter(zs[:, 0], zs[:, 1], marker='+')
    plt.plot(xs_ca[:, 0], xs_ca[:, 3], 'k')

    plt.subplot(133)
    
    dx_ca = (xs_ca[:,0].T - track[:,0]) / track[:,0]
    dy_ca = (xs_ca[:,3].T - track[:,1]) / track[:,1]
    plt.plot(dx_ca.T, label='dx_ca')
    plt.plot(dy_ca.T, label='dy_ca')
    dx = (xs[:,0].T - track[:,0]) / track[:,0]
    dy = (xs[:,3].T - track[:,1]) / track[:,1]
    plt.plot(dx.T, label='dx_ct')
    plt.plot(dy.T, label='dy_ct')
    plt.title('relative error')
    plt.legend()
    plt.axhline(y=0, color='k')

    plt.show()
    
