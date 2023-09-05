import pyrealsense2 as rs
import math
# 创建一个管道对象
pipeline = rs.pipeline()

# 配置并启动管道
config = rs.config()
config.enable_stream(rs.stream.accel)  # 启用加速度计数据流
pipeline.start(config)

try:
    # 获取一些加速度计样本
    for _ in range(30):  # 获取10个样本
        frames = pipeline.wait_for_frames()  # 等待帧
        accel_frame = frames.first_or_default(rs.stream.accel)
        if not accel_frame:
            continue

        # 获取加速度计的数据
        accel = accel_frame.as_motion_frame().get_motion_data()
        print("Accelerometer: x = {:.2f}, y = {:.2f}, z = {:.2f}".format(accel.x, accel.y, accel.z))
        
        pitch = math.atan2(accel.y, math.sqrt(accel.x**2 + accel.z**2)) 
        roll = math.atan2(-accel.x, accel.z)
        pitch = math.degrees(pitch) 
        roll = math.degrees(roll) 
        print("Pitch:", pitch, "Roll:", roll)

finally:
    pipeline.stop()

