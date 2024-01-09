package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// A field centric mecanum drivetrain, which controls the robot relative to the driver's POV and not the robot.
// It allows a more natural way to perform rotation while strafing and other evasive maneuvers
// by automatically correcting robot heading.
@TeleOp(name = "fieldCentric")
public class LinearTeleOp_fieldCentric extends BaseLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        boolean lastMovementLauncher = false;
        boolean toggleMovementLauncher = false;

        boolean lastMovementWrist = false;
        boolean toggleMovementWrist = false;

        boolean lastMovementCL = false;
        boolean toggleMovementCL = false;

        boolean lastMovementCR = false;
        boolean toggleMovementCR = false;

        while (opModeIsActive()) {
            int topLeftEncoderPos = topLeft.getCurrentPosition();
            int topRightEncoderPos = topRight.getCurrentPosition();
            int backLeftEncoderPos = backLeft.getCurrentPosition();
            int backRightEncoderPos = backRight.getCurrentPosition();
            int arm1EncoderPos = arm1.getCurrentPosition();
            int arm2EncoderPos = arm2.getCurrentPosition();
            double launcherPos = planeLauncher.getPosition();
            double clawAnglePos = clawAngle.getPosition();
            double clawLeftPos = clawLeft.getPosition();
            double clawRightPos = clawRight.getPosition();

            telemetry.addData("Wrist pos: ", clawAnglePos);
            telemetry.addData("topLeftPos: ", topLeftEncoderPos);
            telemetry.addData("topRightPos: ", topRightEncoderPos);
            telemetry.addData("backLeftPos: ", backLeftEncoderPos);
            telemetry.addData("backRightPos: ", backRightEncoderPos);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double t = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double angle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            telemetry.addData("IMU heading: ", Math.toDegrees(angle));
            telemetry.update();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(angle) - y * Math.sin(angle);
            double rotY = x * Math.sin(angle) + y * Math.cos(angle);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(t), 1);
            double topLeftPow = (rotY + rotX + t) / denominator;
            double backLeftPow = (rotY - rotX + t) / denominator;
            double topRightPow = (rotY - rotX - t) / denominator;
            double backRightPow = (rotY + rotX - t) / denominator;

            topLeft.setPower(topLeftPow);
            backLeft.setPower(backLeftPow);
            topRight.setPower(topRightPow);
            backRight.setPower(backRightPow);

            arm1.setDirection(DcMotorSimple.Direction.REVERSE);
            double armPower = (-.2 * gamepad1.left_trigger + .35 * gamepad1.right_trigger);
            arm1.setPower(armPower);
            arm2.setPower(armPower);


            planeLauncher.setDirection(Servo.Direction.REVERSE);
            boolean launcherCurrentMovement = gamepad1.a;
            if (launcherCurrentMovement && !lastMovementLauncher) {
                toggleMovementLauncher = !toggleMovementLauncher;
                if (toggleMovementLauncher) {
                    planeLauncher.setPosition(0);
                } else {
                    planeLauncher.setPosition(.7);
                }
            }
            lastMovementLauncher = launcherCurrentMovement;

            boolean leftClawCurrentMovement = gamepad1.left_bumper;
            if (leftClawCurrentMovement && !lastMovementCL) {
                toggleMovementCL = !toggleMovementCL;
                if (toggleMovementCL) {
                    clawLeft.setPosition(.15); // close position
                } else {
                    clawLeft.setPosition(.35);
                }
            }
            lastMovementCL = leftClawCurrentMovement;

            boolean rightClawCurrentMovement = gamepad1.right_bumper;
            if (rightClawCurrentMovement && !lastMovementCR) {
                toggleMovementCR = !toggleMovementCR;
                if (toggleMovementCR) {
                    clawRight.setPosition(.65);
                } else {
                    clawRight.setPosition(.85); // close position
                }
            }
            lastMovementCR = rightClawCurrentMovement;

            boolean toggleWrist = gamepad1.x;
            if (toggleWrist && !lastMovementWrist) {
                toggleMovementWrist = !toggleMovementWrist;
                if (toggleMovementWrist) {
                    clawAngle.setPosition(0);
                } else {
                    clawAngle.setPosition(1);
                }
            }
            lastMovementWrist = toggleWrist;
        }
    }
}
