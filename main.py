from rosbags.interfaces import ConnectionExtRosbag2
from rosbags.rosbag2 import Reader, Writer
from pathlib import Path
from typing import cast
from rosbags.serde import deserialize_cdr

if __name__ == '__main__':
    src = Path('/home/gianluca/airlab_outdoor_bag_2')
    dst = Path('/home/gianluca/airlab_outdoor_bag_3')

    with Reader(src) as reader, Writer(dst) as writer:
        conn_map = {}
        for conn in reader.connections:
            ext = cast(ConnectionExtRosbag2, conn.ext)
            conn_map[conn.id] = writer.add_connection(
                conn.topic,
                conn.msgtype,
                ext.serialization_format,
                ext.offered_qos_profiles,
            )

        for conn, timestamp, data in reader.messages():
            msg = deserialize_cdr(data, conn.msgtype)
            if transforms := getattr(msg, 'transforms', None):
                if transforms[0].header.frame_id == 'map':
                    continue

            writer.write(conn_map[conn.id], timestamp, data)
