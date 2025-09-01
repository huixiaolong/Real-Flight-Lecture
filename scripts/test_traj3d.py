from scipy.linalg import lstsq
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Traj:
    def __init__(self):
        self.px,self.py,self.pz = 0,0,0
    def parmToPolynomial(self,T,start_pos=[0,0,1],end_pos = [1,1,1],
                       start_vel=[-0.4,0.5,0.0],end_vel = [-2,1,0],
                       start_acc=[0,-0.5,0],end_acc = [0,0.5,0]):
        bx = np.array([start_pos[0],start_vel[0],start_acc[0],end_pos[0],end_vel[0],end_acc[0]])
        by = np.array([start_pos[1],start_vel[1],start_acc[1],end_pos[1],end_vel[1],end_acc[1]])
        bz = np.array([start_pos[2],start_vel[2],start_acc[2],end_pos[2],end_vel[2],end_acc[2]])
        A = np.zeros((6,6),float)
        A[0,5] = 1
        A[1,4] = 1
        A[2,3] = 2
        A[3,0],A[3,1],A[3,2],A[3,3],A[3,4],A[3,5] = T**5,T**4,T**3,T**2,T,1
        A[4,0],A[4,1],A[4,2],A[4,3],A[4,4] = 5*(T**4),4*(T**3),3*(T**2),2*T,1
        A[5,0],A[5,1],A[5,2],A[5,3] = 20*(T**3),12*(T**2),6*T,2
        self.px = lstsq(A,bx)[0]
        self.py = lstsq(A,by)[0]
        self.pz = lstsq(A,bz)[0]
        return self.px,self.py,self.pz
    
    def calcPos(self,ts = 0):
        x = self.px[0]*ts**5 + self.px[1]*ts**4 + self.px[2]*ts**3 + self.px[3]*ts**2 + self.px[4]*ts + self.px[5]
        y = self.py[0]*ts**5 + self.py[1]*ts**4 + self.py[2]*ts**3 + self.py[3]*ts**2 + self.py[4]*ts + self.py[5]
        z = self.pz[0]*ts**5 + self.pz[1]*ts**4 + self.pz[2]*ts**3 + self.pz[3]*ts**2 + self.pz[4]*ts + self.pz[5]
        return [x,y,z]
    
    def calcVel(self,ts = 0):
        vx = 5*self.px[0]*ts**4 + 4*self.px[1]*ts**3 + 3*self.px[2]*ts**2 + 2*self.px[3]*ts + self.px[4]
        vy = 5*self.py[0]*ts**4 + 4*self.py[1]*ts**3 + 3*self.py[2]*ts**2 + 2*self.py[3]*ts + self.py[4]
        vz = 5*self.pz[0]*ts**4 + 4*self.pz[1]*ts**3 + 3*self.pz[2]*ts**2 + 2*self.pz[3]*ts + self.pz[4]
        return [vx,vy,vz]
    
    def calcAcc(self,ts = 0):
        ax = 20*self.px[0]*ts**3 + 12*self.px[1]*ts**2 + 6*self.px[2]*ts + 2*self.px[3]
        ay = 20*self.py[0]*ts**3 + 12*self.py[1]*ts**2 + 6*self.py[2]*ts + 2*self.py[3]
        az = 20*self.pz[0]*ts**3 + 12*self.pz[1]*ts**2 + 6*self.pz[2]*ts + 2*self.pz[3]
        return [ax,ay,az]
        

    # def p_traj(self,T,P1=np.array([0,-0.4,0,0,0.5,0,1,0,0],float),P2=np.array([1,-2,0,1,1,0,1,0,0])):
    #     # 起点P_1([x-axis],[y-axis],[z-axis])
    #     # 起点P_1(x,vx,ax,y,vy,ay,z,vz,az) 终点P_2(x,vx,ax,y,vy,ay,z,vz,az)
    #     #正弦效果
    #     #P1=np.array([0,-0.4,0,0,0.5,0,1,0,0],float),P2=np.array([1,-2,0,1,1,0,1,0,0])  
    #     #P1=np.array([0,0.5,0,0,0.5,0,1,0,0],float),P2=np.array([1,1,0,1,1,0,1,0,0])
    #     bx = np.hstack((P1[0:3],P2[0:3]))
    #     by = np.hstack((P1[3:6],P2[3:6]))
    #     bz = np.hstack((P1[6:9],P2[6:9]))
    #     A = np.zeros((6,6))
    #     A[0,5] = 1
    #     A[1,4] = 1
    #     A[2,3] = 2
    #     A[3,0],A[3,1],A[3,2],A[3,3],A[3,4],A[3,5] = T**5,T**4,T**3,T**2,T,1
    #     A[4,0],A[4,1],A[4,2],A[4,3],A[4,4] = 5*(T**4),4*(T**3),3*(T**2),2*T,2
    #     A[5,0],A[5,1],A[5,2],A[5,3] = 20*(T**3),12*(T**2),6*T,2
    #     self.px = lstsq(A,bx)[0]
    #     self.py = lstsq(A,by)[0]
    #     self.pz = lstsq(A,bz)[0]
    #     return 
    # def v_traj(self,ts = 0):





if __name__=='__main__':
    traj = Traj()
    # 起点P_1(x,vx,ax,y,vy,ay,z,vz,az) 终点P_2(x,vx,ax,y,vy,ay,z,vz,az)
    Time = 2
    traj.parmToPolynomial(T=Time)
    tslist = np.linspace(0,Time,100)
    pos = []
    vel = []
    acc = []
    # x(t) = px_0*t^5 + px_1*t^4 + px_2*t^3 + px_3*t^4 + px_4*t^5 + px_5
    # y(t) = py_0*t^5 + py_1*t^4 + py_2*t^3 + py_3*t^4 + py_4*t^5 + py_5
    # z(t) = pz_0*t^5 + pz_1*t^4 + pz_2*t^3 + pz_3*t^4 + pz_4*t^5 + pz_5
    #(x(t),y(t),z(t)) = (px_0,px_1,px_2,px_3,px_4,px_5).()
    #                   (py_0,py_1,py_2,py_3,py_4,py_5).()
    #                   (pz_0,pz_1,pz_2,pz_3,pz_4,pz_5).()
    for ts in tslist:
        pos.append(traj.calcPos(ts))
        vel.append(traj.calcVel(ts))
        acc.append(traj.calcAcc(ts))
    t = tslist.tolist()
    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # ax.plot(x, y, z, label='parametric curve')
    # fig2 = plt.figure()
    # plt.plot(tslist,yd)
    # fig3 = plt.figure()
    
    plt.subplot(2,2,1)
    plt.plot(tslist,[i[0] for i in vel])
    plt.subplot(2,2,2)
    plt.plot(tslist,[i[1] for i in vel])
    plt.subplot(2,2,3)
    plt.plot(tslist,[i[2] for i in vel])
    plt.subplot(2,2,4)
    plt.plot(tslist,[i[1] for i in acc])
    plt.show()