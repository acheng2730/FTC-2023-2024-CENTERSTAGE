package org.firstinspires.ftc.teamcode.Auton_noGyro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AutonomousNoGyro")
public class Auton_noGyro_rightSpikeRight extends BaseLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        sleep(500);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();

        driveFor(1, 0, 0, 1000, timer);
        sleep(100);

        driveFor(-1, 0, 0, 1000, timer);
        sleep(100);

        driveFor(0, 1, 0, 1000, timer);
        sleep(100);

        driveFor(0, -1, 0, 1000, timer);
        sleep(100);

        driveFor(0, 0, 90, 1000, timer);
        sleep(100);


    }

    public void driveFor(double y, double x, double t, int milliseconds, ElapsedTime timer) {
        x *= 1.1;
        timer.reset();
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(t), 1);
        double topLeftPower = (y + x + t) / denominator;
        double backLeftPower = (y - x + t) / denominator;
        double topRightPower = (y - x - t) / denominator;
        double backRightPower = (y + x - t) / denominator;

        topLeft.setPower(topLeftPower);
        backLeft.setPower(backLeftPower);
        topRight.setPower(topRightPower);
        backRight.setPower(backRightPower);
        while (timer.milliseconds() <= milliseconds) {
            setDrivePowers(topLeftPower, backLeftPower, topRightPower, backRightPower);
        }
    }

    public void setDrivePowers(double backLeftPow, double topLeftPow, double backRightPow, double topRightPow) {
        backLeft.setPower(backLeftPow);
        topLeft.setPower(topLeftPow);
        backRight.setPower(backRightPow);
        topRight.setPower(topRightPow);
    }
}
