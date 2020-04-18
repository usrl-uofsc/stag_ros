from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped,PoseStamped,Quaternion
from tf.transformations import quaternion_multiply as qm
import rosbag
import sys
import math
tag_pose_topic="/vrpn_client_node/MarkerBase/pose"
tag_tf_frame = "tag"

cam_pose_topic="/vrpn_client_node/CameraBase/pose"
cam_tf_frame="logi_cam"

qC = [0,0,1,0]

with rosbag.Bag(sys.argv[1], "r") as ib:
    with rosbag.Bag(sys.argv[2],"w") as ob:
        for topic, msg, t in ib:
            ob.write(topic,msg,t)
            if(topic == tag_pose_topic):
                tfmsg = TFMessage()
                tf = TransformStamped()
                tf.header = msg.header
                tf.child_frame_id = tag_tf_frame
                tf.transform.translation.x = msg.pose.position.x
                tf.transform.translation.y = msg.pose.position.y
                tf.transform.translation.z = msg.pose.position.z
                tf.transform.rotation = msg.pose.orientation

                tfmsg.transforms.append(tf)
                ob.write("/tf",tfmsg,t)
            elif(topic == cam_pose_topic):
                print("cam")
                tfmsg = TFMessage()
                tf = TransformStamped()
                tf.header = msg.header
                tf.child_frame_id = cam_tf_frame
                tf.transform.translation.x = msg.pose.position.x
                tf.transform.translation.y = msg.pose.position.y
                tf.transform.translation.z = msg.pose.position.z
                
                q=[0,0,0,0]
                q[0] = msg.pose.orientation.x
                q[1] = msg.pose.orientation.y
                q[2] = msg.pose.orientation.z
                q[3] = msg.pose.orientation.w
                newq=qm(q,qC)
                tf.transform.rotation = Quaternion(newq[0],newq[1],newq[2],newq[3])
                


                tfmsg.transforms.append(tf)
                ob.write("/tf",tfmsg,t)