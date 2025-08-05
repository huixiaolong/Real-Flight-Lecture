import mavros
import pyrealsense2 as rs
import numpy as np
import rospy
from quaternion import *
import tf
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import Pose,PoseStamped
from math import *
import queue
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def logit(x):
    return log(x/(1-x))

p_hit,p_miss,p_min,p_max,p_occ=0.7,0.35,0.12,0.97,0.8
prob_hit_log = logit(p_hit)
prob_miss_log = logit(p_miss)
clamp_min_log = logit(p_min)
clamp_max_log = logit(p_max)
min_occupancy_log = logit(p_occ)

map_size=np.array((20,40),float)
resolution = 0.1#单位m
buffer_size = ceil(map_size[0]/resolution)*ceil(map_size[1]/resolution)#向上取整
map_origin = -map_size/2.0

cache_voxel = queue.Queue()
hit = np.zeros(buffer_size,int)
hitandmiss = np.zeros(buffer_size,int)
occupancy_buffer = np.ones(buffer_size,float)*clamp_min_log
flag_rayend = np.zeros(buffer_size,int)

fx,fy,cx,cy = 606.72357,606.79547,332.51956,240.02243
width,height = 640,480
uav_pose = PoseStamped()
uav_odom = Odometry()

def setcache(id,occ):
    if id>80000:
        print('chao le ')
    hitandmiss[id] = hitandmiss[id] + 1
    # 大量射线会经过同一网格，条件语句用于处理冗余
    # 如果之前hitandmiss是0，那么现在为1，加入cache_voxel等待处理
    if hitandmiss[id] == 1:
        cache_voxel.put(id)
    if occ==1:
        hit[id] = hit[id] +1
    return

def raycast(start,end,origin,voxelsize):
    # https://blog.csdn.net/u011426016/article/details/103229303/
    cur_id = np.floor((start-origin)/voxelsize)
    end_id = np.floor((end-origin)/voxelsize)
    # if np.linalg.norm(end_id[2]+2)<1e-5:
    #     print(end)
    vec = end - start
    # 将整体运动分解为沿三个坐标轴的分运动，
    # 因此每个分运动只有前、后、原地不动三种情况。
    # stepx表示x轴运动方向。
    stepx = 1 if vec[0]>=0 else -1
    stepy = 1 if vec[1]>=0 else -1
    stepz = 1 if vec[2]>=0 else -1
    # 根据运动方向判断下一个边界位置
    if stepx>0:
        next_voxel_boundary_x = (cur_id[0]+1 )*voxelsize + origin[0]
    else:
         next_voxel_boundary_x = cur_id[0]*voxelsize + origin[0]
    if stepy>0:
        next_voxel_boundary_y = (cur_id[1]+1 )*voxelsize + origin[1]
    else:
        next_voxel_boundary_y = cur_id[1]*voxelsize + origin[1]
    if stepz>0:
        next_voxel_boundary_z = (cur_id[2]+1 )*voxelsize + origin[2]
    else:
        next_voxel_boundary_z = cur_id[2]*voxelsize + origin[2]
    # 从起点到下一边界的时间(时间参数迭代的初值)
    # 对于速度为0的分量，设置一个较大的初始时间
    tx = (next_voxel_boundary_x-start[0])/vec[0] if vec[0]!=0 else 1e6
    ty = (next_voxel_boundary_y-start[1])/vec[1] if vec[1]!=0 else 1e6
    tz = (next_voxel_boundary_z-start[2])/vec[2] if vec[2]!=0 else 1e6
    
    dtx = voxelsize/abs(vec[0]) if vec[0] !=0 else 1e6
    dty = voxelsize/abs(vec[1]) if vec[1] !=0 else 1e6
    dtz = voxelsize/abs(vec[2]) if vec[2] !=0 else 1e6
    
    while np.linalg.norm(cur_id - end_id)>1e-5:
        voxel_id = cur_id[1]*200+cur_id[0]
        setcache(int(voxel_id),0)
        if tx<ty:
            if tx<tz:
                tx = tx + dtx
                cur_id[0] = cur_id[0] + stepx
            else:
                tz = tz + dtz
                cur_id[2] = cur_id[2] + stepz
        else:
            if ty<tz:
                ty = ty + dty
                cur_id [1]= cur_id[1] + stepy
            else:
                tz = tz + dtz
                cur_id[2] = cur_id[2] + stepz
    return

def pos2id(pos,origin,voxelsize):
    x = pos[0]
    y = pos[1]
    x0 = origin[0]
    y0 = origin[1]
    idx = floor((x-x0)/voxelsize)
    idy = floor((y-y0)/voxelsize)
    # x轴长度20，分辨率0.1，每行200个格子，左下角索引是0，从左到有，从下到上索引增加，看运动规划教材图6.6
    return idy*200+idx

def id2pos(id,origin,voxelsize):
    return (id+0.5)*voxelsize+origin
    
def initCamera():
    use435 = True
    # use265 = True
    # useIMU = True
    context = rs.context()    
    if use435:
        width = 640
        height = 480
        pipeline = rs.pipeline(context)
        config = rs.config()
        config.enable_device('317222070681')
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, 30)
        # config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
        profile = pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: ", depth_scale)  
        
    # 对齐彩色和深度   
    align_to = rs.stream.color
    align = rs.align(align_to)  
    # return pipeline,pipeline2,align,depth_scale
    return pipeline,align,depth_scale

def xyzrgb_array_to_pointcloud2(points, colors):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()
    assert(points.shape == colors.shape)
    buf = []
    msg.header.stamp = rospy.Time.now()
    
    msg.header.frame_id = 'map'

    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        N = len(points)
        xyzrgb = np.array(np.hstack([points, colors]), dtype=np.float32)
        msg.height = 1
        msg.width = N

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('r', 12, PointField.FLOAT32, 1),
        PointField('g', 16, PointField.FLOAT32, 1),
        PointField('b', 20, PointField.FLOAT32, 1)
    ]
    msg.is_bigendian = False
    msg.point_step = 24
    msg.row_step = msg.point_step * N
    msg.is_dense = True; 
    msg.data = xyzrgb.tostring()
    return msg

def drawRayAndGrid():
    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp = rospy.Time.now()
    m.color.g = 1.0
    m.id = 80001
    m.color.a = 1.0
    m.scale.x = 0.01
    m.scale.y = 0.01
    m.scale.z = 0.01
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    # 标记射线更新的网格
    p = Marker()
    p.header.frame_id = 'map'
    p.header.stamp = rospy.Time.now()
    p.color.b = 1.0
    p.color.a = 1.0
    p.scale.x = 0.01
    p.scale.y = 0.01
    p.scale.z = 0.01
    p.id = 80002
    p.type = Marker.POINTS
    p.action = Marker.ADD
    return m,p

def initMap():
    gridmap = OccupancyGrid()
    gridmap.header.frame_id = 'map'
    gridmap.header.stamp = rospy.Time.now()
    #width对应x轴，height对应y轴 （idx,idy）在一维数组中的索引是 idy*width+idx
    gridmap.info.width = 200
    gridmap.info.height = 400
    gridmap.info.resolution = 0.1
    gridmap.info.origin = Pose()
    gridmap.info.origin.position.x = -10.0
    gridmap.info.origin.position.y = -20.0
    gridmap.info.origin.position.z = 0.0
    gridmap.info.origin.orientation.w = 1.0
    gridmap.info.origin.orientation.x = 0.0
    gridmap.info.origin.orientation.y = 0.0
    gridmap.info.origin.orientation.z = 0.0
    gridmap.data = np.ones(gridmap.info.width *gridmap.info.height ,np.uint8)
    return gridmap
def poseCb(msg:PoseStamped):
    global uav_pose
    uav_pose = msg


if __name__=='__main__':
    rospy.init_node('cameraTest')
    sub_pose = rospy.Subscriber('/mavros/local_position/pose',PoseStamped,poseCb)
    pub_cloud = rospy.Publisher('d435i/points',PointCloud2,queue_size=1)
    pub_gridmap = rospy.Publisher('d435i/gridmap',OccupancyGrid,queue_size=1)
    pub_ray = rospy.Publisher("d435i/ray", Marker, queue_size=10)
    motion = tf.TransformBroadcaster()
    
    gridmap = initMap()
    d435,align,depth_scale = initCamera()
    
    raycast_num = 0
    while not rospy.is_shutdown():
        # 从D435i读对齐后的深度、彩色图像
        iframes = d435.wait_for_frames()
        aligned_frames = align.process(iframes)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        depth = np.asanyarray(depth_frame.get_data())
        color = np.asanyarray(color_frame.get_data())
        rospoints = [] #存点云
        roscolor =  [] #存点云颜色
        horizonpts = []#只处理（1，240）-- （640，240）的像素
        horizonvirpts = []#(1,240)--(640,240)没有深度的点
        # https://www.intelrealsense.com/zh-hans/depth-camera-d435i/  相机参数
        # 点云越稠密，占据结果越稳定，但处理时间越长。
        # 对于python实现，建议水平和垂直像素步进为4或8。
        for x in range(0,width,4):
            for y in range(0,height,4):
                xyz = np.zeros(3,float)
                xyz[0] = depth[y,x].astype(float)*depth_scale
                # 去掉没有深度的点或深度大于6的点，防止边界溢出，目前地图是（20,40）大
                if xyz[0]<0.3 or xyz[0]>6:
                    continue

                # 转换点云坐标从相机坐标系（右下前）到地图坐标系（前左上）
                xyz[1] = -(x-cx)/fx*xyz[0]
                xyz[2] = -(y-cy)/fy*xyz[0]
                bgr = np.zeros(3,float)
                bgr[:] = color[y,x]/255#rviz color [0,1]
                rospoints.append(xyz)
                roscolor.append(bgr)
                if y ==240:
                    horizonpts.append(xyz)
        
        # 转换点云从当前位置到全局
        tw = np.array([uav_pose.pose.position.x,uav_pose.pose.position.y,uav_pose.pose.position.z],float)
        rwb = quaternion(uav_pose.pose.orientation.w,uav_pose.pose.orientation.x,uav_pose.pose.orientation.y,uav_pose.pose.orientation.z)
        # 异常处理：水平线上可能没点
        # print(len(horizonpts))
        if not horizonpts:
            continue
        wp = rotate_vectors(rwb,rospoints)+tw
        whp = rotate_vectors(rwb,horizonpts)+tw
        # raycastpts = rotate_vectors(rwb,horizonpts)+tw
        msg = xyzrgb_array_to_pointcloud2(wp,np.array(roscolor))
        # msg = xyzrgb_array_to_pointcloud2(np.array(rospoints),np.array(roscolor))
        pub_cloud.publish(msg)
        m,p = drawRayAndGrid()
        for pt in whp:
            id = pos2id(pt,(-10,-20),0.1)
            # 有深度的点，才更新相机到3D点之间的网格，是不是有问题？射线尽头没有实体就不更新了？
            setcache(id,1)
            raycast(tw,pt,np.array([-10,-20,0]),0.1)
            point1 = Point(tw[0],tw[1],tw[2])
            point2 = Point(pt[0],pt[1],pt[2])
            m.points.append(point1)
            m.points.append(point2)
        pub_ray.publish(m)

        # 处理setcache的所有网格状态，记录、更新、显示
        idupdate = []
        while not cache_voxel.empty():
            id = cache_voxel.get()
            idupdate.append(id)
            # 判断hit与miss大小  prob_hit_log>0  prob_miss_log<0
            if hit[id]>=(hitandmiss[id]-hit[id]):
                log_odds_update = prob_hit_log  
            else:
                log_odds_update = prob_miss_log
            hit[id] = 0
            hitandmiss[id] = 0
            # 超过阈值不再更新
            if log_odds_update>=0 and occupancy_buffer[id]>=clamp_max_log:
                continue
            if log_odds_update<=0 and occupancy_buffer[id]<=clamp_min_log:
                continue
            occupancy_buffer[id] = min(max(occupancy_buffer[id] \
                + log_odds_update,clamp_min_log),clamp_max_log)
            
        for id in idupdate:        
            if  occupancy_buffer[id]>min_occupancy_log:
                gridmap.data[id] = 100 
            else:
                gridmap.data[id] = 0  
        pub_gridmap.publish(gridmap)   
            
