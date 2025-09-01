import mavros
import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import RCIn
from geometry_msgs.msg import PoseStamped


mode = 'stablized'
def cmdCb(event):
    # print('hello',event.last_real)
    # pub_cmd.publish(PositionTarget())
    return

def rcCb(msg:RCIn):
    global mode
    if msg.channels[4]==1065:
        mode = 'stablized'
    if msg.channels[4]==1499:
        mode = 'position'
    if msg.channels[4]==1933:
        mode = 'offboard'

if __name__=='__main__':
    rospy.init_node('action')
    rospy.Timer(rospy.Duration(0.01),cmdCb)
    pub_cmd = rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget)
    pub_cmd2 = rospy.Publisher('/mavros/local_position/pose',PoseStamped)
    rospy.Subscriber('/mavros/rc/in',RCIn,rcCb)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        print('cur mode',mode)
        cmd = PoseStamped()
        cmd.pose.position.x = 2.0
        cmd.pose.position.y = 1.0
        cmd.pose.position.z = 2.0
        cmd.pose.orientation.w = 1.0
        cmd.pose.orientation.x = 0.0
        cmd.pose.orientation.y = 0.0
        cmd.pose.orientation.z = 0.0
        pub_cmd2.publish(cmd)
        rate.sleep()
        
    
    # rospy.spin()
