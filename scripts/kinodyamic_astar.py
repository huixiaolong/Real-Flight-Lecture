#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
# import matplotlib.pyplot as plt
import numpy as np
import heapq
from math import *
# 在.3py基础上修改，改变KinoAstar中map的初始化方式,增加规划结果的返回值  修改Map中addObstcal 去掉节点初始化环节，改由Planning节点调用

class Node:
    def __init__(self) -> None:
        self.cost = 0.0
        self.pos = np.array([0.0,0.0],float)
        self.vel = np.array([0.0,0.0],float)
        self.parent = None
        self.acc = np.array([0.0,0.0],float)
        self.dur = 0.0

class KinoAstar:
    def __init__(self):
        self.tb =dict()
        self.pq = []
        self.rho = 10.0
        self.max_vel = 3.0
        self.coef = None
        self.t_shot = 0.0
        self.t_search = 0.0
        self.cur_end_id = None
        self.Tm = np.array([[0,0,0,0],[1,0,0,0],[0,2,0,0],[0,0,3,0]])
        # rospy.init_node('kino')
        self.traj_marker = Marker()
        self.traj_pub = rospy.Publisher('traj_show',Marker,queue_size=10)
        self.traj_pts = []

        # rospy.sleep(1.0)
        return 
    # 修改下面函数，替代原函数
    def pos2id(self,pos,origin,voxelsize):
        x = pos[0]
        y = pos[1]
        x0 = origin[0]
        y0 = origin[1]
        idx = floor((x-x0)/voxelsize)
        idy = floor((y-y0)/voxelsize)
        # x轴长度20，分辨率0.1，每行200个格子，左下角索引是0，从左到有，从下到上索引增加，看运动规划教材图6.6
        return idx,idy

    def id2pos(self,id,origin,voxelsize):
        return (id+0.5)*voxelsize+origin
    
    def draw(self,id,type,color,scale,pts):
        self.traj_marker.header.frame_id = 'map'
        self.traj_marker.type =type
        self.traj_marker.action = Marker.ADD
        self.traj_marker.id = id
        self.traj_marker.color.r = color[0]
        self.traj_marker.color.g = color[1]
        self.traj_marker.color.b = color[2]
        self.traj_marker.color.a = 1.0
        self.traj_marker.scale.x = scale
        self.traj_marker.scale.y = scale
        self.traj_marker.scale.z = scale
        self.traj_marker.points = pts
        self.traj_pub.publish(self.traj_marker)
    
    def nextState(self,cur_pos,cur_vel,acc,dur):
        next_pos = cur_pos + cur_vel*dur+0.5*acc*dur*dur
        next_vel = cur_vel + acc*dur
        return next_pos,next_vel

    def computeCost(self,acc,dur):
        return np.dot(acc,acc)*dur+self.rho*dur 

    def computeHeu(self,cur_pos,end_pos):
        return np.linalg.norm(cur_pos-end_pos)
    
    def retrievePath(self,end_id):
        cur_id = end_id
        fuel_cost = self.tb[cur_id].cost
        time_cost = 0.0
        self.traj_pts.clear()
        while self.tb[cur_id].parent:
            pre_id = self.tb[cur_id].parent
            pre_pos = self.tb[pre_id].pos
            pre_vel = self.tb[pre_id].vel
            acc = self.tb[cur_id].acc
            dur = self.tb[cur_id].dur
            self.drawPrimitive(pre_pos,pre_vel,acc,dur,cl='b',lw=3.0)
            cur_id = pre_id
            time_cost = time_cost + dur 
        # print('fuel_cost = ',fuel_cost)
        # print('time_cost = ',time_cost)
        self.draw(0,Marker.LINE_STRIP,[0,0,1],0.05,self.traj_pts)
        self.traj_pts.clear()
        self.drawShotTraj(cl='g',lw=3.0)
        self.draw(1,Marker.LINE_STRIP,[0,1,0],0.05,self.traj_pts)
        self.cur_end_id = end_id
        self.t_search = time_cost      
        return
    
    def drawPrimitive(self,pre_pos,pre_vel,acc,dur,cl='r',lw=1.0):
        num=10
        ts_list = np.linspace(0,dur,num,endpoint=False) 
        # 注意：整体是从后往前，但是每一段点事从前往后 
        ts_list = np.flip(ts_list) 
        x_list= [] 
        y_list = []
        for ts in ts_list:
            pos,vel = self.nextState(pre_pos,pre_vel,acc,ts)
            self.traj_pts.append(Point(pos[0],pos[1],0.0))
            # x_list.append(pos[0])
            # y_list.append(pos[1])
        # plt.plot(x_list,y_list,linewidth=lw,color=cl)
        return
    
    def drawShotTraj(self,cl='g',lw=1.0):
        point_num = 10
        ts = np.linspace(0,self.t_shot,num=point_num,endpoint=True)
        pos = np.zeros((2,point_num),float)
        for i,t in enumerate(ts):
            pos[:,i] = np.dot(self.coef,np.array([1.0,t,t**2,t**3]))
            self.traj_pts.append(Point(pos[0][i],pos[1][i],0.0))
        # plt.plot(pos[0,:],pos[1,:],linewidth = lw,color = cl)
        return
    
    def check(self,pre_pos,pre_vel,acc,dur,occ_map:OccupancyGrid):
        # 如果采样时间是1s，中间规划的轨迹可能因为速度快而更长，
        # 而如果强很薄，可能20个采样点无法采到障碍物所在的voxel
        # 解决这个问题要么加厚障碍物，要么提高采样数量
        # 100速度会比较慢，之前设置20
        sx,sy = occ_map.info.origin.position.x,occ_map.info.origin.position.y
        res = occ_map.info.resolution
        width = occ_map.info.width
        num=100
        ts_list = np.linspace(0,dur,num,endpoint=False)   
        for ts in ts_list:
            pos,vel = self.nextState(pre_pos,pre_vel,acc,ts)
            idx,idy = self.pos2id(pos,(sx,sy),res)
            # 改到这了，周一继续
            if occ_map.data[width*idy+idx]==100:
                return False
        return True
    
    def estimateHeuristic(self,x1,x2):
        dp = x2[0:2] - x1[0:2]
        v0 = x1[2:4]
        v1 = x2[2:4]
        ts = self.quartic(self.rho,0,-4*(np.dot(v0,v0)+np.dot(v0,v1)+np.dot(v1,v1)),24*np.dot(v0+v1,dp),-36*np.dot(dp,dp))
        t_min = np.linalg.norm(dp,ord = np.inf)/self.max_vel #轴向最远距离，最大轴速度，求时间下限
        cost = 100000000 
        td = t_min
        for t in ts:
            if t <= t_min:
                continue
            #找最优代价及其对应的时间
            c = 12*np.dot(dp,dp)/(t*t*t)-12*np.dot(v0+v1,dp)/(t*t)+4*(np.dot(v0,v0)+np.dot(v0,v1)+np.dot(v1,v1))/t + self.rho*t
            if c< cost:
                cost = c
                td = t
        return 2*cost,td #返回最优代价和时间

    def quartic(self,a,b,c,d,e):#返回实数根
        # ax^4 + bx^3 + cx^2 + dx + e =0 => x^4 + a3x^3 + a2x^2 +a1x + a0 = 0
        a3 = b/a
        a2 = c/a
        a1 = d/a
        a0 = e/a
        dts = []
        y1 = self.cubic(1,-a2, a1*a3- 4*a0, 4*a2*a0-a1*a1-a3*a3*a0)
        r = a3*a3/4 - a2 + y1
        if r<0:
            return dts
        R = np.sqrt(r)
        if R != 0:
            D = 0.75*a3*a3 - R*R - 2*a2 +0.25*(4*a3*a2-8*a1-a3*a3*a3)/R
            E = 0.75*a3*a3 - R*R - 2*a2 -0.25*(4*a3*a2-8*a1-a3*a3*a3)/R
        else:
            D = 0.75*a3*a3 - 2*a2 + 2*np.sqrt(y1*y1-4*a0)
            E = 0.75*a3*a3 - 2*a2 - 2*np.sqrt(y1*y1-4*a0)
        if D >= 0:
            dts.append(-a3/4 + R/2 + np.sqrt(D)/2)
            dts.append(-a3/4 + R/2 - np.sqrt(D)/2)
        if E >= 0:
            dts.append(-a3/4 - R/2 + np.sqrt(E)/2)
            dts.append(-a3/4 - R/2 - np.sqrt(E)/2)
        return dts 

    def cubic(self,a,b,c,d):#返回一个最大实数根
        #ax^3 + bx^2 + cx + d = 0 => x^3 + a2x^2 + a1x + a0 = 0
        a2 = b/a 
        a1 = c/a
        a0 = d/a
        Q = a1/3 - a2*a2/9
        R = (a1*a2 - 3*a0)/6-a2**3/27
        D = R**2 + Q**3 
        if D>0:
            U = np.cbrt(R + np.sqrt(D))
            V = np.cbrt(R - np.sqrt(D))
            return U+V-a2/3
        elif Q ==0: 
            return -(a2)/3
        else:
            theta = acos(R/((-Q)**(3/2)))
            z1=2*(-Q)**0.5*cos(theta/3)-a2/3
            # theta = math.acos(R/((-Q)**(3/2)))
            # z1=2*(-Q)**0.5*math.cos(theta/3)-a2/3
            return z1 
        
    def computeShotTraj(self,state1,state2,time_to_goal):
        self.end_vel_ = state2[2:4]
        p0 = state1[0:2]
        dp = state2[0:2]-p0
        v0 = state1[2:4]
        dv = state2[2:4]-v0
        td = time_to_goal
        a = 1.0/6.0*(-12.0*(dp-v0*td)/(td*td*td)+6*dv/(td*td))
        b = 0.5*(6.0*(dp-v0*td)/(td*td)-2.0*dv/td)
        c = v0
        d = p0
        # 1/6 * alpha *t^3 + 1/2 * beta *t^2 + v0*t + p0
        # a*t^3 + b*t^2 + c*t + d  
        # d + c*t + b*t^2 + a*t^3  c + 2b*t + 3a*t^2 + 0t^3   2b + 6a*t + 0t^2 + 0t^3
        #[d,c,b,a] dot [1,t,t^2,t^3] = p(t)
        coef = np.zeros((2,4),float)
        coef[:,0] = d
        coef[:,1] = c
        coef[:,2] = b
        coef[:,3] = a
        self.coef = coef
        self.t_shot = time_to_goal
        return 
    
    def getSamples(self,ts=0.05):
        t_sum = self.t_shot + self.t_search
        seg_num = np.floor(t_sum/ts)
        ts = t_sum/seg_num
        t = self.t_shot
        
        pos = []
        vel = []
        while t>0:
            pos.append(np.dot(self.coef,np.array([1.0,t,t**2,t**3])))
            vel.append(np.dot(self.coef,np.dot(self.Tm,np.array([1.0,t,t**2,t**3]))))
            t = t-ts
        cur_id = self.cur_end_id
        while self.tb[cur_id].parent:
            pre_id = self.tb[cur_id].parent
            pre_pos = self.tb[pre_id].pos
            pre_vel = self.tb[pre_id].vel
            acc = self.tb[cur_id].acc
            dur = self.tb[cur_id].dur
            t = dur + t
            while t>0:
                nextstate = self.nextState(pre_pos,pre_vel,acc,t)
                pos.append(nextstate[0])
                vel.append(nextstate[1])
                t=t-ts   
            cur_id = pre_id
        return pos,vel
    
    def search(self,start_pos=np.array([0.0,0.0]),start_vel=np.array([0.0,0.0]),end_pos=np.array([5.5,5.5]),end_vel=np.array([0.0,0.0]),occ_map = OccupancyGrid()):
        #连续调用search时，备忘录和优先队列需要清空
        self.tb = dict()
        self.pq = []
        sx,sy = occ_map.info.origin.position.x,occ_map.info.origin.position.y
        res = occ_map.info.resolution
        end_id = self.pos2id(end_pos,(sx,sy),res)
        end_state = np.concatenate((end_pos,end_vel))
        start_state = np.concatenate((start_pos,start_vel))
        start_node = Node()
        start_node.pos = start_pos
        start_node.vel = start_vel
        start_node.parent = None
        start_node.cost = 0.0

        start_id = self.pos2id(start_pos,(sx,sy),res)
        self.tb[start_id]=start_node
        index = 0
        start_priority = start_node.cost +10.0*self.estimateHeuristic(start_state,end_state)[0]

        heapq.heappush(self.pq,(start_priority,index,start_id))
        while self.pq:
            x,y,cur_id = heapq.heappop(self.pq)
            # 到达终点提前退出
            if np.linalg.norm(np.array(cur_id) - np.array(end_id))<20:
                # print(self.tb[cur_id].pos)
                # print(self.tb[cur_id].vel)
                cur_state = np.concatenate((self.tb[cur_id].pos,self.tb[cur_id].vel))
                cost,time_to_goal = self.estimateHeuristic(cur_state,end_state)
                self.computeShotTraj(cur_state,end_state,time_to_goal)
                self.retrievePath(cur_id)
                return 
            cur_pos,cur_vel = self.tb[cur_id].pos,self.tb[cur_id].vel
            cur_cost = self.tb[cur_id].cost
            for ax in [-1.5,-1.0,-0.5,0,0.5,1.0,1.5]:
                for ay in [-1.5,-1.0,-0.5,0,0.5,1.0,1.5]:
                    acc = np.array((ax,ay),float)
                    for dur in [1.0]:
                        next_pos,next_vel = self.nextState(cur_pos,cur_vel,acc,dur)
                        x , y = next_pos
                        vx,vy = next_vel
                        nextid = self.pos2id(next_pos,(sx,sy),res)
                        if nextid==cur_id:
                            continue
                        # 位置、速度超过边界值 
                        if x<-10 or y<-20 or x>=10 or y>=20 or abs(vx)>3 or abs(vy)>3:
                            continue
                        # 障碍物问题
                        if not self.check(cur_pos,cur_vel,acc,dur,occ_map):
                            continue
                        cost = cur_cost + self.computeCost(acc,dur)
                        # priority = cost + self.computeHeu(next_pos,end_pos)
     
                        if nextid not in self.tb or cost< self.tb[nextid].cost:
                            node = Node()
                            node.cost = cost
                            node.pos = next_pos
                            node.vel = next_vel
                            node.parent = cur_id
                            node.acc = acc
                            node.dur = dur
                            self.tb[nextid]=node
                            index = index+1
                            next_state = np.concatenate((next_pos,next_vel))
                            priority = cost + 10.0*self.estimateHeuristic(next_state,end_state)[0]
                            heapq.heappush(self.pq,(priority,index,nextid))
                            # self.drawPrimitive(cur_pos,cur_vel,acc,dur)
        return 
    

