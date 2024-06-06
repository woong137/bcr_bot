#!/usr/bin/env python3

import rospy
import tf


def print_base_footprint_position():
    rospy.init_node('base_footprint_position_printer')

    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            # Lookup the transformation from 'map' to 'base_footprint'
            (trans, rot) = listener.lookupTransform(
                'map', 'bcr_bot_0/base_footprint', rospy.Time(0))

            print("Translation: %.2f, %.2f, %.2f" % (
                trans[0], trans[1], trans[2]))

            # Convert rotation from quaternion to euler angles
            euler = tf.transformations.euler_from_quaternion(rot)
            print("Rotation: %.2f, %.2f, %.2f" % (
                euler[0], euler[1], euler[2]))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()


if __name__ == '__main__':
    try:
        print_base_footprint_position()
    except rospy.ROSInterruptException:
        pass
