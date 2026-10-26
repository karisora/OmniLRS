#!/usr/bin/env python3
"""Check a running Lumos simulation; --drive also tests forward motion and turning."""

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def stamp(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def yaw(message):
    q = message.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true', help='Move forward and turn briefly, then stop')
    parser.add_argument('--timeout', type=float, default=90, help='Overall wall-clock timeout, seconds')
    parser.add_argument('--output', type=Path, default=Path('build/lumos_ros2_check'))
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('check_lumos_ros2')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    images, infos, odometry = {'left': {}, 'right': {}}, {}, []
    subscriptions = []

    def image_callback(message, side):
        images[side][stamp(message)] = message
        if len(images[side]) > 30:
            del images[side][next(iter(images[side]))]

    for side in images:
        subscriptions.append(node.create_subscription(
            Image, f'/stereo/{side}/image_raw',
            lambda msg, side=side: image_callback(msg, side), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(
            CameraInfo, f'/stereo/{side}/camera_info',
            lambda msg, side=side: infos.update({side: msg}), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        Odometry, '/odom', lambda msg: odometry.__setitem__(slice(None), [msg]), 10))
    deadline = time.monotonic() + args.timeout

    def wait_for(condition, description):
        while not condition():
            if time.monotonic() > deadline:
                raise RuntimeError(f'Timed out waiting for {description}')
            rclpy.spin_once(node, timeout_sec=0.05)

    report = {}
    try:
        wait_for(lambda: publisher.get_subscription_count() > 0 and odometry and len(infos) == 2
                 and len(images['left']) >= 3 and len(images['right']) >= 3
                 and bool(images['left'].keys() & images['right'].keys())
                 and buffer.can_transform('odom', 'stereo_left_optical_frame', Time())
                 and buffer.can_transform('stereo_left_optical_frame', 'stereo_right_optical_frame', Time()),
                 '/cmd_vel subscriber, odometry, synchronized stereo images, CameraInfo and TF')
        synchronized_stamp = max(images['left'].keys() & images['right'].keys())
        pair = []
        for side in images:
            image, info = images[side][synchronized_stamp], infos[side]
            expected_frame = f'stereo_{side}_optical_frame'
            assert image.header.frame_id == info.header.frame_id == expected_frame
            assert (image.width, image.height) == (info.width, info.height) == (640, 480)
            assert image.encoding == 'rgb8'
            assert len(image.data) == image.step * image.height
            pixels = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.step)
            pixels = pixels[:, :image.width*3].reshape(image.height, image.width, 3)
            assert pixels.std() > 1, f'{side} image is blank'
            pair.append(pixels)
            times = sorted(images[side])
            report[side] = dict(frame_id=expected_frame, stamp_ns=synchronized_stamp,
                                width=image.width, height=image.height, encoding=image.encoding,
                                sim_rate_hz=(len(times)-1)*10**9/(times[-1]-times[0]),
                                k=list(info.k), p=list(info.p), pixel_std=float(pixels.std()))
        transform = buffer.lookup_transform('stereo_left_optical_frame', 'stereo_right_optical_frame', Time())
        translation = transform.transform.translation
        assert math.isclose(translation.x, .12, abs_tol=1e-6)
        assert abs(translation.y) < 1e-6 and abs(translation.z) < 1e-6
        assert math.isclose(infos['left'].p[3], 0, abs_tol=1e-6)
        assert math.isclose(-infos['right'].p[3]/infos['right'].p[0], translation.x, abs_tol=1e-6)
        np.testing.assert_allclose(infos['left'].k, infos['right'].k)
        np.testing.assert_allclose(np.array(infos['left'].p).reshape(3, 4)[:, :3],
                                   np.array(infos['left'].k).reshape(3, 3))
        np.testing.assert_allclose(np.array(infos['right'].p).reshape(3, 4)[:, :3],
                                   np.array(infos['right'].k).reshape(3, 3))
        report['baseline_m'] = translation.x
        print('Stereo images, calibration, optical TF and /cmd_vel subscriber: OK', flush=True)

        def drive(linear, angular, seconds):
            start = stamp(odometry[0])
            command = Twist()
            command.linear.x, command.angular.z = linear, angular
            while stamp(odometry[0]) - start < seconds * 10**9:
                if time.monotonic() > deadline:
                    raise RuntimeError('Timed out during the driving check')
                publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=0.05)

        if args.drive:
            initial = odometry[0].pose.pose.position
            drive(.1, 0., 2.5)
            drive(0., 0., 1.)
            final = odometry[0].pose.pose.position
            distance = math.hypot(final.x-initial.x, final.y-initial.y)
            assert distance > .10, f'Forward motion too small: {distance} m'
            heading = yaw(odometry[0])
            drive(0., .3, 2.)
            drive(0., 0., 1.)
            turn = math.atan2(math.sin(yaw(odometry[0])-heading), math.cos(yaw(odometry[0])-heading))
            assert turn > .1, f'Positive yaw command did not turn left: {turn} rad'
            assert abs(odometry[0].twist.twist.linear.x) < .02
            assert abs(odometry[0].twist.twist.angular.z) < .05
            report['drive'] = dict(forward_distance_m=distance, left_turn_rad=turn, stopped=True)
            print(f'Forward: {distance:.3f} m; left turn: {turn:.3f} rad; stop: OK', flush=True)
        args.output.mkdir(parents=True, exist_ok=True)
        PILImage.fromarray(np.concatenate(pair, axis=1)).save(args.output/'stereo.png')
        (args.output/'ros2_check.json').write_text(json.dumps(report, indent=2)+'\n')
        print(f'Saved {args.output}/stereo.png and ros2_check.json', flush=True)
    finally:
        if args.drive:
            for _ in range(5):
                publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
