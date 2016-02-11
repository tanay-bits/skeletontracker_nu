#!/usr/bin/env python

import rospy
import tf
import baxter_interface
import struct
import numpy as np
from skeletonmsgs_nu.msg import Skeletons
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
# from std_msgs.msg import UInt8
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from sensor_msgs.msg import JointState
from baxter_interface import CHECK_VERSION

def ik_solve(limb,p,q):
    global right
    global left
    
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(position=p,orientation=q)),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(position=p,orientation=q))
    }
        
    right = baxter_interface.Limb('right')
   
    '''
    seeds={
         'right': JointState(
             header=hdr,
             name= right.joint_angles().keys(),
             position= right.joint_angles().values()
             )
             }
    
    #using random seeds  
    
    seeds2={
         'right': JointState(
             header=hdr,
             name= ['right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0', 'right_e1'],

             position= [-1.3483691110107423, 
             -0.11850001572875977, 1.18768462366333, -0.002300971179199219,0.4371845240478516, 
             1.8419274289489747, 0.4981602602966309]
             )
             }            
    '''
 

    ikreq.pose_stamp.append(poses[limb])
    
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
            
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    for i in range(100):
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                    }.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                  (seed_str,))
        # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print "\nIK Joint Solution:\n", limb_joints
            print "------------------"
            print "Response Message:\n", resp
            return limb_joints

        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
            
            noise= np.random.normal(0,0.25,7)
            
            '''
            print right.joint_angles().values()
            print right.joint_angles().keys()
            print right.joint_angles()
            print "\r\n"
            '''
            
            '''
            seeds_random={
                'right': JointState(
                 header=hdr,
                 name= right.joint_angles().keys(),
                 position= (right.joint_angles().values() + noise).tolist()
                 
                 )
             }
             '''
            
            js = JointState()
            js.header = hdr 
            ii = 0
            for key,val in right.joint_angles().iteritems():
                js.name.append(key)
                js.position.append(val+noise[ii])
                ii += 1
           
            
            ikreq.seed_angles = [js]
            
            resp = iksvc(ikreq)

        return right.joint_angles()



def callback(message):
    global left
    global right
    global tflistener   

    number_users = len(message.skeletons)
    user = message.skeletons[number_users-1].userid
    print str(user)
    trans_point = Point()
    rot_quat = Quaternion()

    try:
        (trans, rot) = tflistener.lookupTransform('/torso_' + str(user),
            '/right_hand_' + str(user), rospy.Time(0))
        trans_point.x = -trans[0]*1.5
        trans_point.y = trans[2]*1.5
        trans_point.z = -trans[1]*1.5
        rot_quat.x = rot[0]
        rot_quat.y = rot[1]
        rot_quat.z = rot[2]
        rot_quat.w = rot[3]
        right.move_to_joint_positions(ik_solve('right',trans_point,rot_quat))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass


        
        


def main():
    global left
    global right
    global tflistener

    rospy.init_node('baxnode')
    tflistener = tf.TransformListener()
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    right = baxter_interface.Limb('right')
    right.set_joint_position_speed(1.0)
    left = baxter_interface.Limb('left')
    left.set_joint_position_speed(1.0)

    rospy.Subscriber("/skeletons", Skeletons, callback)
    rospy.spin()


if __name__ == '__main__':
    main()