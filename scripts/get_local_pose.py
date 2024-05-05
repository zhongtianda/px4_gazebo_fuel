import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
import sys
from gazebo_msgs.msg import ModelStates
from pyquaternion import Quaternion
import numpy as np

vehicle_type = sys.argv[1]
vehicle_num = int(sys.argv[2])
multi_pose_pub = [None]*vehicle_num
multi_speed_pub = [None]*vehicle_num
multi_local_pose = [PoseStamped() for i in range(vehicle_num)]
multi_speed = [Vector3Stamped() for i in range(vehicle_num)]

init_ = False
init_x = 0
init_y = 0
init_z = 0
q = Quaternion(1, 0, 0, 0)

def gazebo_model_state_callback(msg):
    for vehicle_id in range(vehicle_num):
        id = msg.name.index(vehicle_type+'_'+str(vehicle_id))
        multi_local_pose[vehicle_id].header.stamp = rospy.Time().now()
        multi_local_pose[vehicle_id].header.frame_id = 'world'
        multi_local_pose[vehicle_id].pose = msg.pose[id]
        global init_, init_x, init_y, init_z, q

        if not init_:
            init_x = multi_local_pose[vehicle_id].pose.position.x
            init_y = multi_local_pose[vehicle_id].pose.position.y
            init_z = multi_local_pose[vehicle_id].pose.position.z
            # 待写：这里增加记录初始变换
            init_qw = multi_local_pose[vehicle_id].pose.orientation.w
            init_qx = multi_local_pose[vehicle_id].pose.orientation.x
            init_qy = multi_local_pose[vehicle_id].pose.orientation.y
            init_qz = multi_local_pose[vehicle_id].pose.orientation.z
            q = Quaternion(init_qw, init_qx, init_qy, init_qz)
            init_ = True
        # multi_speed[vehicle_id].header.stamp = rospy.Time().now()
        # multi_speed[vehicle_id].header.frame_id = 'world'
        # multi_speed[vehicle_id].vector = msg.twist[id]

if __name__ == '__main__':
    rospy.init_node(vehicle_type+'_get_pose_groundtruth')
    gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback,queue_size=1)
    for i in range(vehicle_num):
        multi_pose_pub[i] = rospy.Publisher(vehicle_type+'_'+str(i)+'/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        # multi_speed_pub[i] = rospy.Publisher(vehicle_type+'_'+str(i)+'/mavros/vision_speed/speed', Vector3Stamped, queue_size=1)
        print("Get " + vehicle_type + "_" + str(i) + " groundtruth pose")
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        for i in range(vehicle_num):
            pose = PoseStamped()
            twb = np.array([init_x, init_y, init_z])
            twa = np.array([multi_local_pose[i].pose.position.x, multi_local_pose[i].pose.position.y, multi_local_pose[i].pose.position.z])
            tba = q.inverse.rotate(twa - twb)
            
            pose.pose.position.x = tba[0]
            pose.pose.position.y = tba[1]
            pose.pose.position.z = tba[2]

            q_ = Quaternion(multi_local_pose[i].pose.orientation.w, multi_local_pose[i].pose.orientation.x, multi_local_pose[i].pose.orientation.y, multi_local_pose[i].pose.orientation.z)
            q_ = q_*q.inverse
            pose.pose.orientation.w = q_.w
            pose.pose.orientation.x = q_.x
            pose.pose.orientation.y = q_.y
            pose.pose.orientation.z = q_.z
            pose.header.stamp = rospy.Time().now()
            pose.header.frame_id = 'world'
            
            multi_pose_pub[i].publish(pose)
            # multi_speed_pub[i].publish(multi_speed[i])
        try:
            rate.sleep()
        except:
            continue

