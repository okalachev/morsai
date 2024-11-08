import tf.transformations


def quaternion_from_orientation(o):
    return o.x, o.y, o.z, o.w


def euler_from_orientation(o, axes='rzyx'):
    q = quaternion_from_orientation(o)
    return tf.transformations.euler_from_quaternion(q, axes)
