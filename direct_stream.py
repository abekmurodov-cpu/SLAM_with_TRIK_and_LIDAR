import socket

# Set up the Direct Socket Server
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('0.0.0.0', 9090))
server.listen(1)

brick.display().clear()
brick.display().addLabel("Waiting for Laptop...", 10, 10)
brick.display().redraw()

# Accept connection & set non-blocking for real-time control
conn, addr = server.accept()
conn.setblocking(False) 

brick.display().clear()
brick.display().addLabel("Two-Way Stream LIVE!", 10, 10)
brick.display().redraw()

brick.encoder("E3").reset()
brick.encoder("E4").reset()

while True:
    try:
        # 1. SEND SENSORS (Encoders, IMU, LiDAR)
        e3 = brick.encoder("E3").read()
        e4 = brick.encoder("E4").read()
        gyro = brick.gyroscope().read()
        gz = gyro[2] # Z-Axis (Rotation)
        
        scan = brick.lidar().read()
        scan_str = ",".join(str(x) for x in scan)
        
        # Format: "LeftEnc,RightEnc,GyroZ:dist1,dist2..."
        data = f"{e3},{e4},{gz}:{scan_str}\n"
        conn.sendall(data.encode('utf-8'))
        
        # 2. RECEIVE MOTORS (From Laptop)
        try:
            cmd = conn.recv(1024).decode('utf-8')
            if cmd:
                parts = cmd.strip().split(',')
                if len(parts) == 2:
                    brick.motor("M3").setPower(int(parts[0]))
                    brick.motor("M4").setPower(int(parts[1]))
        except:
            pass # No command received this millisecond

        script.wait(50) # 20 Hz
        
    except Exception as e:
        brick.motor("M3").powerOff()
        brick.motor("M4").powerOff()
        break
