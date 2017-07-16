#!/usr/bin/env python  
import rospy

import numpy as np

import tf
import tf2_ros
import geometry_msgs.msg

init = False
angle = 0
norm = 0

def message_from_transform(T):
    msg = geometry_msgs.msg.Transform()
    q = tf.transformations.quaternion_from_matrix(T)
    translation = tf.transformations.translation_from_matrix(T)
    msg.translation.x = translation[0]
    msg.translation.y = translation[1]
    msg.translation.z = translation[2]
    msg.rotation.x = q[0]
    msg.rotation.y = q[1]
    msg.rotation.z = q[2]
    msg.rotation.w = q[3]
    return msg

def publish_transforms():

    global init, angle, norm

    T0_obj = tf.transformations.concatenate_matrices(
        tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_from_euler(0.79,0,0.79)),
        tf.transformations.translation_matrix((0,1,1)))

    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"
    object_transform.transform = message_from_transform(T0_obj)
    br.sendTransform(object_transform)

    T0_robo = tf.transformations.concatenate_matrices(
        tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_about_axis(1.5,(0,0,1))),
        tf.transformations.translation_matrix((0,-1,0)))

    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"
    robot_transform.transform = message_from_transform(T0_robo)
    br.sendTransform(robot_transform)

    if not init:
        init = True

        Trobo_camera = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix((0,0.1,0.1)),
            tf.transformations.quaternion_matrix(
                tf.transformations.quaternion_from_euler(0,0,0)))

        T0_camera = T0_robo.dot(Trobo_camera) # T1 * T2 does not work
        # obj (x,y,z,1) in base coordinate
        p_base = T0_obj[:,3]

        #p_robo = np.linalg.inv(T0_robo) * p_base

        #p_camera = np.linalg.inv(Trobo_camera) *p_robo
        #print("p_base:", p_base)
        # obj (x,y,z) in camera coordnate
        p_camera = np.linalg.inv(T0_camera).dot(p_base)[:3]
        

        x = [1,0,0]
        #print(p_camera)
        # find a vector normal to both vectors
        norm = np.cross(x, p_camera)

        p_camera_unit = p_camera / np.linalg.norm(p_camera)
        angle = np.arccos(np.dot(x, p_camera_unit))
        #angle = 3
        print("my p_camera",p_camera)


    Trobo_camera = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix((0,0.1,0.1)),
            tf.transformations.quaternion_matrix(
                tf.transformations.quaternion_about_axis(angle, norm)))

    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"
    camera_transform.transform = message_from_transform(Trobo_camera)
    br.sendTransform(camera_transform)

if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
