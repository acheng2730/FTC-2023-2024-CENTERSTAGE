package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//note: cannot set very accurate positions without gyro

@Autonomous(name = "AutonomousNoGyro")
public class Auton_noGyro extends BaseLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sleep(500);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        driveFor(1, 0, 0, 3000, timer);
    }

    public void driveFor(double y, double x, double rx, int milliseconds, ElapsedTime timer) {
        x *= 1.1;
        timer.reset();
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double topLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double topRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

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
        backLeft.setPower(backLeftPow);
        topRight.setPower(topRightPow);
    }
}
