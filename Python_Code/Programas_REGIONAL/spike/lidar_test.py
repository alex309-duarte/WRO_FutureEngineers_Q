from rplidar import RPLidar
PORT = '/dev/ttyUSB0'  
lidar = RPLidar(PORT, baudrate=115200)

lidar.start_motor()

for i, scan in enumerate(lidar.iter_scans()):
    print(f'{i}: {len(scan)} mediciones')  
    # cada elemento: (quality, angle, distance_mm)
    if i > 20:  
        break

lidar.stop()
lidar.stop_motor()
lidar.disconnect()