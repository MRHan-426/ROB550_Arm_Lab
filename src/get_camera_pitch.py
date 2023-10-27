import pyrealsense2 as rs
import math

pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.accel)
pipeline.start(config)
# task 1, get extrinsic matrix by hand.
try:
    for _ in range(30):
        frames = pipeline.wait_for_frames()
        accel_frame = frames.first_or_default(rs.stream.accel)
        if not accel_frame:
            continue

        accel = accel_frame.as_motion_frame().get_motion_data()
        print("Accelerometer: x = {:.2f}, y = {:.2f}, z = {:.2f}".format(accel.x, accel.y, accel.z))
        
        pitch = math.atan2(accel.y, math.sqrt(accel.x**2 + accel.z**2)) 
        roll = math.atan2(-accel.x, accel.z)
        pitch = math.degrees(pitch) 
        roll = math.degrees(roll) 
        print("Pitch:", pitch, "Roll:", roll)

finally:
    pipeline.stop()

