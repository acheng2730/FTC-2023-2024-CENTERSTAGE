package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Uses Rev IMU and encoders for localization
// PID controllers for X, Y, robot angle - tuned with FTC Dashboard
// Drives to a certain spot on the field, not a certain distance
@Autonomous(name = "Autonomous-IMU")
public class Auton_IMU extends BaseLinearOpMode {
    double prevTime = 0;
    ElapsedTime driveTime = new ElapsedTime();
    double curPoseY = 0, curPoseX = 0; // Current position on field in inches

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        sleep(500);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        updatePosition();

        driveFieldCentric(2000, 2000, 180);
    }

    public void driveFieldCentric(int xTarget, int yTarget, int tTarget) {

        PIDController xControl = new PIDController(1, 0, 0);
        PIDController yControl = new PIDController(1, 0, 0);
        PIDController thetaControl = new PIDController(1, 0, 0);

        double angle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double x = xControl.calculate(xTarget, curPoseX);
        double y = yControl.calculate(yTarget, curPoseY);
        double t = thetaControl.calculate(tTarget, Math.toDegrees(AngleUnit.normalizeRadians(angle)));
        double x_rotated = x * Math.cos(angle) - y * Math.sin(angle);
        double y_rotated = x * Math.sin(angle) + y * Math.cos(angle);

        double denominator = Math.max(Math.abs(y_rotated) + Math.abs(x_rotated) + Math.abs(t), 1);
        topLeft.setPower((y_rotated + x_rotated + t) / denominator);
        backLeft.setPower((y_rotated - x_rotated + t) / denominator);
        topRight.setPower((y_rotated - x_rotated - t) / denominator);
        backRight.setPower((y_rotated + x_rotated - t) / denominator);
    }

    public void updatePosition() {
        double angle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // apply mecanum kinematic model (with wheel velocities [ticks per sec])
        double xV = (topLeft.getVelocity() + topRight.getVelocity() + backLeft.getVelocity() + backRight.getVelocity()) * 0.482;

        double yV = (-topLeft.getVelocity() + topRight.getVelocity() + backLeft.getVelocity() - backRight.getVelocity()) * 0.482;

        // rotate the vector
        double nx = (xV * Math.cos(angle)) - (yV * Math.sin(angle));
        double nY = (xV * Math.sin(angle)) + (yV * Math.cos(angle));
        xV = nx;
        yV = nY;

        // integrate velocity over time
        curPoseY += (yV * (driveTime.seconds() - prevTime)) / conversionFactor; // <-- Tick to inch conversion factor
        curPoseX += (xV * (driveTime.seconds() - prevTime)) / conversionFactor;
        prevTime = driveTime.seconds();
    }
}
