#
# Python UKF based Motion Tracker module of the piCAMTracker package
# Copyright (c) 2021-2022 Axel Barnitzke <barney@xkontor.org>
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
from math import sqrt,sin,cos,pi
from scipy.linalg import block_diag

class ukfFilterct2(UnscentedKalmanFilter):
    def __init__(self, dt=1.0, w=1.0):
        self.dim_x = 4
        self.dim_z = 2
        self._w = w
        self._dt = dt
        self.Ft = np.eye(self.dim_x)
        self._T = 0.
        sigma_points = MerweScaledSigmaPoints(n=self.dim_x, alpha=.1, beta=2., kappa=-1.)
        super(ukfFilterct2,self).__init__(dim_x=self.dim_x, dim_z=self.dim_z, fx=self.f_ct, hx=self.h_ca, dt=dt, points=sigma_points)
        
    def h_ca(self, x):
        """ measurement function """
        #print("x: ", x[[0,2]])
        return x[[0, 2]]
    
    def start_ct(self, std_r=1.0, v=0.1, vstart=None, dt=None, w=None):
        if vstart is None:
            #                  x  vx y  vy
            self.x = np.array([0.,0.,0.,0.])
        else:
            self.x =np.array(vstart)

        if dt is None:
            dt = self._dt
        else:
            self._dt = dt
        
        if w is None:
            w = self._w
        else:
            self._w = w
        
        self.setup_function()
        
        # process noise
        self.Q = Q_discrete_white_noise(dim=2, dt=dt, var=v, block_size=2)
        # covariance estimation matrix
        self.P = np.eye(4) * 2.
        # measurement noise
        self.R = np.diag([std_r**2, std_r**2])
    
    def f_ct(self, x, dt, w=None):
        #print("<dt: %6.2f, w:%6.2f [x:%6.2f vx:%6.2f y:%6.2f vy:%6.2f]" % (self._dt,w,x[0],x[1],x[2],x[3]))
        new_x = np.dot(self.Ft,x) 
        return new_x
    
    def update_time(self, dt, w=None):
        self._T += dt
        if w:
            self._w = w
        self.setup_function()
            
    def update_time_absolute(self, T, w=None):
        self._T = T
        if w:
            self._w = w
        self.setup_function()
            
    def setup_function(self):
        w = self._w
        wt = w * self._dt
        sin_wt = sin(wt)
        cos_wt = cos(wt)
        if (abs(w) < 1.e-99):
                      # x       vx        y          vy     
            self.Ft = [[1.,     1.,       0.,        0.      ],
                       [0.,   cos_wt,     0.,     -sin_wt    ],
                       [0.,     0.,       1.,        1.      ],
                       [0.,   sin_wt,     0.,      cos_wt    ]]
        else:
                      # x       vx         y         vy
            self.Ft = [[1.,   sin_wt/w,    0., (1.-cos_wt)/w ],
                       [0.,    cos_wt,     0.,    -sin_wt    ],
                       [0., (1.-cos_wt)/w, 1.,    sin_wt/w   ],
                       [0.,    sin_wt,     0.,     cos_wt    ]]
            
        return

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
            #                  x  vx ax y  vy ay
            self.x = np.array([0.,0.,0.,0.,0.,0.])
        else:
            self.x =np.array(vstart)

        if dt is None:
            dt = self._dt
        else:
            self._dt = dt

        # process noise
        self.Q[0:3, 0:3] = Q_discrete_white_noise(3, dt=dt, var=v)
        self.Q[3:6, 3:6] = Q_discrete_white_noise(3, dt=dt, var=v)
        # covariance estimation matrix
        self.P = np.eye(6) * 2.
        # measurement noise
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
                          # x       vx       ax    y        vy       ay
            Ft = np.array([[1.,     1.,      0.5,  0.,      0.,      0.   ],
                           [0.,  cos_wt,     1.,   0.,      0.,      0.   ],
                           [0.,     0.,    cos_wt, 0.,      0.,      0.   ],
                           [0.,     0.,      0.,   1.,      1.,      0.5  ],
                           [0.,     0.,      0.,   0.,   cos_wt,     1.   ],
                           [0.,     0.,      0.,   0.,      0.,    cos_wt ]])
        else:
                          # x      vx            ax         y      vy           ay
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
    import sys
    def f_ca(x, dt):
        """ state transition function for a 
        constant accelarating aircraft"""
        dt2 = .5*dt*dt
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
    N = 30
    r = 3.0
    T = pi
    dphi = pi/N
    zs = []
    track = []
    dt = T/(N+1)
    w = -1. #pi/T
    r1 = r

    setv = True
    vx0 = -10.
    vy0 = -100.
    x0 = 100
    y0 = 0
    np.random.seed(int(time())&0xff)
    for n in range(0,N):
        x = 101.0 - (100.0 * cos(n*dphi))
        y = 1.0 + 100.0 * sin(n*dphi)
        #print("x: %5.2f y: %5.2f" % (x,y))
        track.append([x,y])
        px = x + np.random.randn()*r
        py = y + np.random.randn()*r
        if n < 1:
            x0 = x
            y0 = y
            zs.append([x,y])
        else:
            zs.append([px,py])
            if setv:
                setv = False
                vx0 = (x-x0)/dt
                vy0 = (y-y0)/dt
                print("x0: %6.2f y0: %6.2f vx0: %5.2f vy0: %5.2f" % (x0,y0,vx0,vy0))
            


    # build a ca filter for comparation
    points_ca = MerweScaledSigmaPoints(n=6, alpha=.1, beta=2., kappa=0.1)
    ukf_ca = UnscentedKalmanFilter(dim_x=6, dim_z=2, fx=f_ca, hx=h_ca, dt=dt*1.1, points=points_ca)

    
    #            x  vx  ax y  vy   ay
    vstart_ca = [x0,vx0,0.,y0,vy0,-60.]
    ukf_ca.x = np.array(vstart_ca)
    ukf_ca.R = np.diag([r*r/2., r*r/2.]) 
    ukf_ca.Q[0:3, 0:3] = Q_discrete_white_noise(3, dt=dt, var=0.001)
    ukf_ca.Q[3:6, 3:6] = Q_discrete_white_noise(3, dt=dt, var=0.001)
    ukf_ca.P *= 2.
    
    # create the filter from class
    ukf = ukfFilterct2(dt=dt, w=w)
    vstart = [x0,vx0,y0,vy0*(1.0+2.0/N)]
    #vstart = [x0,-sin(w*dt),y0,cos(w*dt)]
    ukf.start_ct(std_r=r1,v=0.2,vstart=vstart,w=w)

    #xs = [[vstart]]
    #xs_ca = [vstart_ca]
    xs = []
    xs_ca = []
    #xs.append(vstart)
    
    ### >>>
    #T = pi
    #dt = T/(N+1)
    #w = -1. #pi/T
    #ukf._dt = dt
    #ukf._w  = w
    #ukf.setup_function()
    #vx = -sin(w*dt)
    #vy = cos(w*dt)
    #xx = [[-1.,vx,0.,vy]]
    
    #for n in range(0,N):
    #    xn = ukf.f_ct(xx[n],dt,w)
    #    print(">dt: %6.2f, w:%6.2f [x:%6.2f vx:%6.2f y:%6.2f vy:%6.2f]" % (dt,w,xn[0],xn[1],xn[2],xn[3]))
    #    xx.append(xn)
    #
    #xx = np.array(xx)
    #plt.title('ukf_f_ct')
    #plt.plot(xx[:, 0], xx[:, 2], 'k')
    #plt.scatter(xx[:, 0], xx[:, 2], marker='+')
    #plt.show()    
    #sys.exit(0)
    ### <<<
    
    err = []
    err_ca = []
    for n,z in enumerate(zs):
        xs_ca.append(ukf_ca.x)
        xs.append(ukf.x)
        t0 = time()
        ukf.predict(w=w)
        ukf.update(z)
        t1 = time()        
        ukf_ca.predict()
        ukf_ca.update(z)
        t2 = time()
        dct = (t1-t0) * 1000.0 
        dca = (t2-t1) * 1000.0 

        #print("[000] ukf: dx:%6.2f dy:%6.2f ca: dx:%6.2f dy:%6.2f" % (z[0]-ukf.x[0],z[1]-ukf.x[3],z[0]-ukf_ca.x[0],z[1]-ukf_ca.x[3]))

        dx = ukf.x[0] - track[n][0]
        dy = ukf.x[2] - track[n][1]
        distance = sqrt(dx*dx+dy*dy)
        err.append(distance)
        dx_ca = ukf_ca.x[0] - track[n][0]
        dy_ca = ukf_ca.x[3] - track[n][1]
        distance_ca = sqrt(dx_ca*dx_ca+dy_ca*dy_ca)
        err_ca.append(distance_ca)
        print("[%03d] ukf: dx:%6.2f dy:%6.2f ca: dx:%6.2f dy:%6.2f dist: %6.2f tct: %4.2fms tca: %4.2fms" % (n, z[0]-ukf.x[0],z[1]-ukf.x[2],z[0]-ukf_ca.x[0],z[1]-ukf_ca.x[3],distance,dct,dca))

    #del xs[-1]
    #del xs_ca[-1]
    
    xs = np.array(xs)
    xs_ca = np.array(xs_ca)
    zs = np.array(zs)
    track = np.array(track)
    err = np.array(err)
    err_ca = np.array(err_ca)
    
    #print("len zs:   %d" %len(zs))
    #print("len xs:   %d" %len(xs))
    #print("len trck: %d" %len(track))
    
    plt.subplot(131)
    plt.title('ukf_ct')
    plt.plot(track[:, 0], track[:, 1], '--r')
    plt.scatter(zs[:, 0], zs[:, 1], marker='+')
    plt.plot(xs[:, 0], xs[:, 2], color='k', marker='x')

    plt.subplot(132)
    plt.title('ukf_ca')
    plt.plot(track[:, 0], track[:, 1], '--r')
    plt.scatter(zs[:, 0], zs[:, 1], marker='+')
    plt.plot(xs_ca[:, 0], xs_ca[:, 3], color='k', marker='x')

    plt.subplot(133)
    
    #dx_ca = (xs_ca[:,0].T - track[:,0]) / track[:,0]
    #dy_ca = (xs_ca[:,3].T - track[:,1]) / track[:,1]
    #plt.plot(dx_ca.T, label='dx_ca')
    #plt.plot(dy_ca.T, label='dy_ca')
    #dx = (xs[:,0].T - track[:,0]) / track[:,0]
    #dy = (xs[:,2].T - track[:,1]) / track[:,1]
    #plt.plot(dx.T, label='dx_ct')
    #plt.plot(dy.T, label='dy_ct')

    plt.plot(err, 'x-', label='dist_ct')
    plt.plot(err_ca, 'x-', label='dist_ca')
    
    
    #plt.axhline(y=0, 'k')
    plt.axhline(y=r, color='grey', linestyle='dashed', label='noise-level')
    
    plt.title('absolute error')
    plt.legend()
    plt.show()
