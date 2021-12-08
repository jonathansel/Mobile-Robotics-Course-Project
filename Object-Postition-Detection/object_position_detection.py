#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from tf2_ros.transform_broadcaster import TransformBroadcaster
from vision_msgs.msg import Detection2DArray as msg_Detect
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import geometry_msgs.msg
import sys
import os
import numpy as np
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

class ImageListener:
    def __init__(self, depth_image_topic, depth_info_topic, detect_topic):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback)
        confidence_topic = depth_image_topic.replace('depth', 'confidence')
        self.sub_conf = rospy.Subscriber(confidence_topic, msg_Image, self.confidenceCallback)
        self.vis_pub = rospy.Publisher("visualization_marker", Marker)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        
        self.sub_detect = rospy.Subscriber(detect_topic, msg_Detect, self.detectCallback)
        self.pixelx = None
        self.pixely = None
        self.id = None 

    def imageDepthCallback(self, data):
        try:
            if self.id == 18 or self.id == 64:
                cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
                # pick one pixel among all the pixels with the closest range:
                #indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
                pix = (int(self.pixelx), int(self.pixely))
                #pix = (240, 240)
                self.pix = pix
                line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
                if self.intrinsics:
                    depth = cv_image[pix[1], pix[0]]
                    result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                    line += '  Coordinate (m): %8.2f %8.2f %8.2f.' % ((result[0]/1000), (result[1]/1000), (result[2]/1000))
                if (not self.pix_grade is None):
                    line += ' Grade: %2d' % self.pix_grade
                line += '\r'
                sys.stdout.write(line)
                sys.stdout.flush()
                id = self.id
                tfBroadcaster((result[0]/1000), (result[1]/1000), (result[2]/1000), id)
                self.createMarker((result[0]/1000), (result[1]/1000), (result[2]/1000), id)
                self.id = 0

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def confidenceCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            grades = np.bitwise_and(cv_image >> 4, 0x0f)
            if (self.pix):
                self.pix_grade = grades[self.pix[1], self.pix[0]]
        except CvBridgeError as e:
            print(e)
            return


    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            self.intrinsics.model = rs2.distortion.none  
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

    def detectCallback(self, data):
         #Ok so we can easily get center pixel point and ID, nice! 
         #The tough part now is sending this with id to get respective real (x y and z) - make a function we can call
         #then publish frame specific for id and keep this position saved for future movebase - or we can just save robots position at detection and call it
         #because after I dont want to continue saving this position

         #what if instead of callback on depth topic we just keep it to this and just call that 
         #function everytime detection is made. Especially since sometimes there wont be a detection 
         # Then two statements for respective object and publish tf in here too? or is that too much
        for detect in data.detections:
            for identity in detect.results:
                self.pixelx = detect.bbox.center.x
                self.pixely = detect.bbox.center.y
                self.id = identity.id
                #print(self.id)
    
    def createMarker(self, x, y, z, id):
        myMarker = Marker()
        myMarker.header.frame_id = "camera_color_optical_frame"      
        #myMarker.header.seq = 1
        #myMarker.header.stamp = rospy.get_rostime()
        #myMarker.ns = "window"
        myMarker.id = id
        myMarker.type = myMarker.SPHERE # sphere
        myMarker.action = myMarker.ADD
        myMarker.pose.position.x = x
        myMarker.pose.position.y = y
        myMarker.pose.position.z = z
        myMarker.pose.orientation.x = 0
        myMarker.pose.orientation.y = 0
        myMarker.pose.orientation.z = 0
        myMarker.pose.orientation.w = 1
        #myMarker.mesh_resource = "package://project/models/window_buena.stl";
        #myMarker.mesh_resource = "package://object_pose/meshes/lowpoly_doge.stl"
        myMarker.color.a = 1
        myMarker.color.r = 1
        myMarker.color.g = 0
        myMarker.color.b = 0
        myMarker.scale.x = 0.2
        myMarker.scale.y = 0.2
        myMarker.scale.z = 0.2
        self.vis_pub.publish(myMarker)

def tfBroadcaster(numba1, numba2, numba3, id):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "camera_color_optical_frame"
    if id == 18: 
        t.child_frame_id = "dog"
    if id == 64:
        t.child_frame_id = "potted_plant"   
    t.transform.translation.x = numba1
    t.transform.translation.y = numba2
    t.transform.translation.z = numba3
    #q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1    
    br.sendTransform(t)

def main():
    depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
    depth_info_topic = '/camera/depth/camera_info'
    detect_topic = '/detectnet/detections' #fix this for correct topic

    print ('')
    print ('object_pose_detection.py')
    print ('--------------------')
    print ('App to demontrate the usage of the /camera/depth topics.')
    print ('')
    print ('Application subscribes to %s and %s topics.' % (depth_image_topic, depth_info_topic))
    print ('Application then calculates and print the range to the closest object.')
    print ('If intrinsics data is available, it also prints the 3D location of the object')
    print ('If a confedence map is also available in the topic %s, it also prints the confidence grade.' % depth_image_topic.replace('depth', 'confidence'))
    print ('')
    
    listener = ImageListener(depth_image_topic, depth_info_topic, detect_topic)
    vis_pub = rospy.Publisher("visualization_marker", Marker)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()