#!/usr/bin/env python


class SubConfig(object):
    # Topics

    sim_type = 'Sub'

    mover_topic = "/cmd_vel"
    arming_topic = "/arming"
    darknet_topic = "/darknet_ros/bounding_boxes"
    camera_topic = "/sj_camera/left_image_raw"
    left_camera_topic = "/sj_camera/left_image_raw"
    right_camera_topic = "/sj_camera/right_image_raw"
    imu_topic = "/mavros/imu/data"
    depth_topic = "/mavros/imu/diff_pressure"

    visualize = False

    # Camara Params
    camera_dims_x = 640.0
    camera_dims_y = 480.0

    # Gate
    gate_depth_speed = -0.4
    gate_depth_time = 2
    gate_forward_speed = 0.3
    gate_forward_time = 40.0
    templates_folder = 'templates_sub'
    gate_time = 50.0

    # PathMarker

    # Vamp Visual Servo
    visual_servo_forward_speed = 0.1

    area_ratio = 0.3
    duration = 3.0

    imu_topic = "/mavros/imu/data"
    jerk_topic = "/mavros/jerk"

    visual_servo_kp_yaw = -0.001
    visual_servo_kp_alt = -0.001
    target_seq = ['jia', 'asw']

class SimConfig(object):
    # Topics
    sim_type = 'Sim'

    mover_topic = "/rexrov/cmd_vel"
    arming_topic = "/rexrov/arming"
    darknet_topic = "/darknet_ros/bounding_boxes"
    camera_topic = "/rexrov/rexrov/camera/camera_image"
    left_camera_topic = "/rexrov/rexrov/cameraleft/camera_image"
    right_camera_topic = "/rexrov/rexrov/cameraright/camera_image"
    imu_topic = "/rexrov/imu"
    depth_topic = "/rexrov/pressure"

    visualize = True

    # Camara Params
    camera_dims_x = 768
    camera_dims_y = 492

    # Gate
    gate_depth_speed = -0.4
    gate_depth_time = 7
    gate_forward_speed = 0.3
    gate_forward_time = 27.0
    templates_folder = 'templates_sim/'
    gate_time = 30.0

    # PathMarker


    # Vamp Visual Servo
    visual_servo_forward_speed = 0.2

    area_ratio = 0.3
    duration = 3.0

    jerk_topic = '/rexrov/jerk'

    visual_servo_kp_alt = -0.0010
    visual_servo_kp_yaw = 0.0005
    target_seq = ['jia', 'Vet']

ConfigMap = {
    "Sub" : SubConfig, "Sim" : SimConfig
}
