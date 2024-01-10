This is the TeamCode folder, where all of your OpModes, module classes, base classes, etc will go.

This folder has both a field centric and robot centric mecanum drive implementation for TeleOp. 
    - Note the field centric code relies on the Rev IMU, which as of writing stops working whenever the robot collides with something.
    - Your best bet is to ask in the FTC discord #programming if the IMU still has this problem - https://discord.gg/first-tech-challenge
    - If no, use program as is; if yes, an old expansion hub or external gyro will be needed
    - If using old expansion hub, replace the line imu = hardwareMap.get(IMU.class, "imu");
      with imu = hardwareMap.get(IMU.class, "imu 1"); in the BaseLinearOpMode class