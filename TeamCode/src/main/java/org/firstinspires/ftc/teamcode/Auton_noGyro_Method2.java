package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//note: use for pid testing

@Autonomous(name = "AutonomousNoGyro")
public class Auton_noGyro_Method2 extends BaseLinearOpMode {

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

    }

    public void driveFor(double x, double y, double t, ElapsedTime timer, int ms) {
        timer.reset();
        double x_rotated = x * Math.cos(t - Math.PI / 4) - y * Math.sin(t - Math.PI / 4);
        x_rotated *= 1.1;
        double y_rotated = x * Math.sin(t - Math.PI / 4) + y * Math.cos(t - Math.PI / 4);
        while (timer.milliseconds() < ms) {
            topLeft.setPower(x_rotated + y_rotated + t);
            backLeft.setPower(x_rotated - y_rotated + t);
            topRight.setPower(x_rotated - y_rotated - t);
            backRight.setPower(x_rotated + y_rotated - t);
        }
    }
}
