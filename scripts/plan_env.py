#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point,Polygon,PoseStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from visualization_msgs.msg import Marker
from kinodyamic_astar import KinoAstar
import numpy as np
import random
from lecture3.msg import PosVelArray,PosVel
from math import *



class Planning:
    def __init__(self):
        rospy.init_node('Planning')
        rospy.Subscriber('goal',Point,self.goal_callback,queue_size=10)
        self.traj_pub = rospy.Publisher('traj',PosVelArray,queue_size=10)
        self.sub_odom = rospy.Subscriber('/mavros/local_position/odom',Odometry,self.odomCb)
        self.sub_grid = rospy.Subscriber('d435i/gridmap',OccupancyGrid,self.gridCb)
        #执行间隔缩短意味着速度提升，此时check周期必须缩短，但search规划时间如果过长，轨迹会直接穿过障碍物
        rospy.Timer(rospy.Duration(0.5),self.fsm_callback) 
        self.occ_map = OccupancyGrid()
        self.kino = KinoAstar()
        self.robot_pos = None
        self.robot_vel = None
        self.end_pos = None
        self.robot_pos_mark = False
        self.robot_map_mark = False
        self.traj_pts = None
        self.traj_vpts = None
        self.state = 0
        rospy.sleep(1.0)
        rospy.spin()
        return 
    # 外部触发
    def goal_callback(self,msg):
        # print('receive goal')
        if self.robot_pos_mark and self.robot_map_mark:
            self.end_pos = np.array((msg.x,msg.y),float)
            self.state = 1
        return
    
    def fsm_callback(self,msg):
        # 0 等待目标  1正在规划  2正在执行 3到达终点
        if self.state==0: 
            return 
        if self.state==1:
            self.kino.search(start_pos=self.robot_pos,start_vel=self.robot_vel,end_pos=self.end_pos,occ_map=self.occ_map)
            self.traj_pts,self.traj_vpts = self.kino.getSamples()
            self.traj_pts.reverse()
            self.traj_vpts.reverse()
            pts = PosVelArray()
            for i in range(len(self.traj_pts)):
                pos = self.traj_pts[i]
                vel = self.traj_vpts[i]
                pts.traj.append(PosVel(pos[0],pos[1],0.0,vel[0],vel[1],0.0))
            self.traj_pub.publish(pts)
            self.state=2
            return
        if self.state==2:
            res = self.check(self.traj_pts,self.robot_pos,self.occ_map)
            if res:
                self.state = 1
                print('replan')
            if np.linalg.norm(self.robot_pos-self.end_pos)<0.1:
                self.state = 3
            return
        if self.state==3:
            print('reach goal,next goal',self.end_pos)
            randy = random.uniform(1.0,4.0)
            if self.robot_pos[0]>6:
                self.end_pos = np.array((0.0,randy),float)
            else:
                self.end_pos = np.array((8.0,randy),float)
            self.state=1
            return
    
    def check(self,traj_pts,cur_pos,occ_map:OccupancyGrid):
        sx,sy = occ_map.info.origin.position.x,occ_map.info.origin.position.y
        res = occ_map.info.resolution
        width = occ_map.info.width
        cur_i = 0
        end_i = len(traj_pts)
        for i,pt in enumerate(traj_pts):
            if np.linalg.norm(pt-cur_pos)<1e-5:
                cur_i = i
                break
        for i in range(cur_i,end_i):
            pt = traj_pts[i]
            idx,idy = self.pos2id(pt,(sx,sy),res)
            if self.occ_map.data[idy*width+idx]==100:
                return True
            dis = np.linalg.norm(pt-cur_pos)
            if dis>3:
                break
        return False
    
    def odomCb(self,msg:Odometry):
        self.robot_pos_mark = True
        self.robot_pos = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y],float)
        self.robot_vel = np.array([msg.twist.twist.linear.x,msg.twist.twist.linear.y],float)

    def gridCb(self,msg:OccupancyGrid):
        self.robot_map_mark = True
        self.occ_map = msg

    def pos2id(self,pos,origin,voxelsize):
        x = pos[0]
        y = pos[1]
        x0 = origin[0]
        y0 = origin[1]
        idx = floor((x-x0)/voxelsize)
        idy = floor((y-y0)/voxelsize)
        # x轴长度20，分辨率0.1，每行200个格子，左下角索引是0，从左到有，从下到上索引增加，看运动规划教材图6.6
        return idx,idy




     
    
if __name__=='__main__':
    planning = Planning()