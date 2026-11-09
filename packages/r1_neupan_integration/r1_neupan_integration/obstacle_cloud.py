"""Read unmodified 3-D sensor observations with per-point acquisition times."""
from __future__ import annotations

import numpy as np


def decode_xyz_timestamps(message):
    """Decode PointCloud2 XYZ and optional absolute FLOAT64 point timestamps.

    Returns (xyz[3,N], stamps_ns[N]); invalid XYZ samples are omitted. Point
    times are never replaced with receive time. If absent, the cloud header
    time is used and callers can inspect the returned per_point_time flag.
    """
    fields = {field.name: field for field in message.fields}
    if any(name not in fields for name in ("x", "y", "z")):
        raise ValueError("pointcloud has no xyz")
    width, height = int(message.width), int(message.height)
    point_step, row_step = int(message.point_step), int(message.row_step)
    if width < 0 or height < 1 or point_step < 1 or row_step < width * point_step:
        raise ValueError("invalid pointcloud dimensions")
    if len(message.data) < row_step * height:
        raise ValueError("truncated pointcloud")
    endian = ">" if message.is_bigendian else "<"
    names = ["x", "y", "z"]
    formats = [endian+"f4"] * 3
    offsets = []
    for name in names:
        field = fields[name]
        if field.datatype != 7 or int(field.count) != 1:
            raise ValueError("xyz must be scalar FLOAT32")
        offsets.append(int(field.offset))
    per_point = "timestamp" in fields
    if per_point:
        field = fields["timestamp"]
        if field.datatype != 8 or int(field.count) != 1:
            raise ValueError("timestamp must be scalar FLOAT64")
        names.append("timestamp");formats.append(endian+"f8");offsets.append(int(field.offset))
    if any(offset < 0 or offset + np.dtype(fmt).itemsize > point_step
           for offset,fmt in zip(offsets,formats)):
        raise ValueError("pointcloud field outside point step")
    dtype = np.dtype(dict(names=names,formats=formats,offsets=offsets,itemsize=point_step))
    data = memoryview(message.data)
    rows = [np.frombuffer(data, dtype=dtype, count=width, offset=row*row_step)
            for row in range(height)]
    samples = np.concatenate(rows) if rows else np.empty(0,dtype=dtype)
    xyz = np.vstack([samples[name].astype(np.float64) for name in ("x","y","z")])
    valid = np.isfinite(xyz).all(axis=0)
    header_stamp = int(message.header.stamp.sec)*1_000_000_000 + int(message.header.stamp.nanosec)
    if header_stamp <= 0:
        raise ValueError("invalid cloud header time")
    if per_point:
        stamp_float = samples["timestamp"]
        valid &= (np.isfinite(stamp_float) & (stamp_float > 0) &
                  (stamp_float < np.iinfo(np.int64).max))
        # Livox timestamps are absolute nanoseconds in FLOAT64; conversion
        # preserves the sensor's float64 precision (about 256 ns at epoch).
        stamps = stamp_float[valid].astype(np.int64)
    else:
        stamps = np.full(np.count_nonzero(valid),header_stamp,dtype=np.int64)
    return xyz[:,valid],stamps,per_point



def project_cloud_to_world(xyz, stamps_ns, header_stamp_ns, bin_ns, max_span_ns, lookup):
    """Transform each acquisition-time bin using a caller-supplied 4x4 TF.

    Returns original-height world points, never a point-cloud warp to mimic
    robot geometry. A missing TF or invalid time rejects the complete sample.
    """
    xyz = np.asarray(xyz,dtype=np.float64)
    stamps = np.asarray(stamps_ns,dtype=np.int64).reshape(-1)
    if (xyz.ndim != 2 or xyz.shape[0] != 3 or xyz.shape[1] != stamps.size or
            not np.isfinite(xyz).all() or type(header_stamp_ns) is not int or
            type(bin_ns) is not int or type(max_span_ns) is not int or
            header_stamp_ns <= 0 or bin_ns <= 0 or max_span_ns < bin_ns):
        raise ValueError("invalid timestamped cloud")
    delta = stamps - header_stamp_ns
    # Float64 encoding at epoch can round a timestamp fractionally below its
    # header; accept only the same sub-microsecond rounding as the 3-D filter.
    if np.any(delta < -1024) or np.any(delta > max_span_ns):
        raise ValueError("point timestamp outside cloud span")
    bins = np.maximum(delta,0) // bin_ns
    world = np.empty_like(xyz)
    for index in np.unique(bins):
        tf_stamp = header_stamp_ns + int(index)*bin_ns + bin_ns//2
        tf = np.asarray(lookup(tf_stamp),dtype=np.float64)
        if tf.shape != (4,4) or not np.isfinite(tf).all():
            raise ValueError("invalid timestamped transform")
        selected = bins == index
        world[:,selected] = tf[:3,:3] @ xyz[:,selected] + tf[:3,3:4]
    return world


def project_scan_to_world(scan, bin_ns, max_span_ns, lookup):
    """Preserve rear laser beam height and acquisition time in world coordinates."""
    start = int(scan.header.stamp.sec)*1_000_000_000 + int(scan.header.stamp.nanosec)
    ranges = np.asarray(scan.ranges,dtype=np.float64)
    if (start <= 0 or ranges.ndim != 1 or not np.isfinite(scan.angle_min) or
            not np.isfinite(scan.angle_increment) or
            not np.isfinite(scan.time_increment) or scan.time_increment < 0):
        raise ValueError("invalid timestamped laser scan")
    angles=scan.angle_min+np.arange(ranges.size)*scan.angle_increment
    valid=np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    times=start+np.rint(np.arange(ranges.size)*float(scan.time_increment)*1e9).astype(np.int64)
    local=np.vstack((ranges[valid]*np.cos(angles[valid]),
                     ranges[valid]*np.sin(angles[valid]),
                     np.zeros(np.count_nonzero(valid))))
    return project_cloud_to_world(local,times[valid],start,bin_ns,max_span_ns,lookup)


def transform_matrix(transform):
    """Convert a geometry_msgs transform to a finite normalized rigid matrix."""
    t=transform.translation;q=transform.rotation
    values=np.array([t.x,t.y,t.z,q.x,q.y,q.z,q.w],dtype=np.float64)
    if not np.isfinite(values).all():
        raise ValueError("invalid sensor transform")
    norm=np.linalg.norm(values[3:])
    if norm < 1e-12:
        raise ValueError("invalid sensor rotation")
    x,y,z,w=values[3:]/norm
    matrix=np.eye(4)
    matrix[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    matrix[:3,3]=values[:3]
    return matrix
