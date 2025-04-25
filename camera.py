import pyrealsense2 as rs

c = rs.context()
devices = c.query_devices()
for dev in devices:
    print("Devices:", dev.get_info(rs.camera_info.name))
    print("Serial number:", dev.get_info(rs.camera_info.serial_number))