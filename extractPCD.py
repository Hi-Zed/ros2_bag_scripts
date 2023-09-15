from pathlib import Path
from rosbags.highlevel import AnyReader
import open3d as o3d
import cv2

from collections import namedtuple
import math
import struct
import argparse

from rosbags.typesys.types import sensor_msgs__msg__PointField

_DATATYPES = {sensor_msgs__msg__PointField.INT8: ('b', 1),
              sensor_msgs__msg__PointField.UINT8: ('B', 1),
              sensor_msgs__msg__PointField.INT16: ('h', 2),
              sensor_msgs__msg__PointField.UINT16: ('H', 2),
              sensor_msgs__msg__PointField.INT32: ('i', 4),
              sensor_msgs__msg__PointField.UINT32: ('I', 4),
              sensor_msgs__msg__PointField.FLOAT32: ('f', 4),
              sensor_msgs__msg__PointField.FLOAT64: ('d', 8)}


def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step


def read_points_list(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message. This function returns a list of namedtuples.
    It operates on top of the read_points method. For more efficient access use read_points directly.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: List of namedtuples containing the values for each point
    @rtype: list
    """

    if field_names is None:
        field_names = [f.name for f in cloud.fields]

    Point = namedtuple("Point", field_names)

    return [Point._make(l) for l in read_points(cloud, field_names, skip_nans, uvs)]

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

def extract_pcd(source, dest):
    src = Path(source)

    with AnyReader([src]) as reader:
        connections = [x for x in reader.connections if x.topic == '/ouster/imu']
        f = open(dest+'/imu.txt', 'w')
        f.write('timestamp, orientation (x, y, z, w), angular velocity, linear acceleration\n')
        print("Extracting IMU data")
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            f.write(str(msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec)+',')
            f.write(str(msg.orientation.x)+','+str(msg.orientation.y)+','+str(msg.orientation.z)+','+str(msg.orientation.w)+',')
            f.write(str(msg.angular_velocity.x)+','+str(msg.angular_velocity.y)+','+str(msg.angular_velocity.z)+',')
            f.write(str(msg.linear_acceleration.x)+','+str(msg.linear_acceleration.y)+','+str(msg.linear_acceleration.z)+'\n')

        connections = [x for x in reader.connections if x.topic == '/ouster/points']
        print("Extracting pcd data")
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            cloud_data = list(read_points(msg, skip_nans=True, field_names=['x', 'y', 'z']))
            full_cloud = o3d.geometry.PointCloud()
            xyz = [(x, y, z) for x, y, z in cloud_data]
            full_cloud.points = o3d.utility.Vector3dVector(xyz)
            pcd_time = str(msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec)
            pcd_name = dest+'/'+pcd_time+'.pcd'
            o3d.io.write_point_cloud(pcd_name, full_cloud)

def extract_images(source, dest):
    src = Path(source)

    with AnyReader([src]) as reader:
        connections = [x for x in reader.connections if x.topic == '/oak/rgb/image_raw/compressed']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            img = cv2.imdecode(msg.data, cv2.IMREAD_COLOR)
            img_time = str(msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec)
            cv2.imwrite(dest+'/'+img_time+'.jpg', img)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', help='location of the bag folder', type=str, required=True)
    parser.add_argument('--out', help='location to save the output', type=str, required=True)
    parser.add_argument('--image', help='extract images', action='store_true', required=False)
    args = parser.parse_args()

    if args.image :
        extract_images(args.bag, args.out)
    else:
        print('other')
        # main(args.bag, args.out)