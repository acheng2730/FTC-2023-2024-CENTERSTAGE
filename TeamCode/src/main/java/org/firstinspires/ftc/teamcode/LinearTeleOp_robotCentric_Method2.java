package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// A more accurate implementation of Mecanum drive, using target angle/power as inputs instead of direct joystick values
@TeleOp(name = "robotCentric-method2")
public class LinearTeleOp_robotCentric_Method2 extends BaseLinearOpMode {
    double curPoseY = 0, curPoseX = 0;
    ElapsedTime driveTime = new ElapsedTime();
    double prevTime = 0;
    double conversionFactor = 162.15;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        // The program will loop several times while a button is being pressed, causing servos to jitter.
        // This boolean logic solves that problem by ensuring actions occur only once per press.
        // This is only needed because we use a single button to toggle each servo; using a button to set a single servo position
        // can be done intuitively.
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

            updatePosition();

            telemetry.addData("curPoseX: ", curPoseX);
            telemetry.addData("curPoseY: ", curPoseY);
            telemetry.addData("Wrist Pos: ", clawAnglePos);
            telemetry.addData("topLeftPos: ", topLeftEncoderPos);
            telemetry.addData("topRightPos: ", topRightEncoderPos);
            telemetry.addData("backLeftPos: ", backLeftEncoderPos);
            telemetry.addData("backRightPos: ", backRightEncoderPos);
            telemetry.update();

            // Mecanum drivetrain
            double strafe = gamepad1.left_stick_x * 1.1;
            double drive = gamepad1.left_stick_y * -1;
            double turn = gamepad1.right_stick_x;

            double theta = Math.atan2(drive, strafe); // Desired bot heading
            double power = Math.hypot(strafe, drive); // Desired power

            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4); // Rotating the bot heading here by -45 degrees makes the
            // mecanum wheels' vectors the x,y components of the power vector
            double max = Math.max(Math.abs(sin), Math.abs(cos)); // Avoid power clipping

            double topLeftPow = power * cos / max + turn;
            double backLeftPow = power * sin / max + turn;
            double topRightPow = power * sin / max - turn;
            double backRightPow = power * cos / max - turn;

            topLeft.setPower(topLeftPow);
            backLeft.setPower(backLeftPow);
            topRight.setPower(topRightPow);
            backRight.setPower(backRightPow);


            arm1.setDirection(DcMotorSimple.Direction.REVERSE); // We use a motor on each side of the arm to support its weight better
            double armPower = (-.2 * gamepad1.left_trigger + .35 * gamepad1.right_trigger); // Slow the downwards movement b/c gravity

            arm1.setPower(armPower);
            arm2.setPower(armPower);

            // Continuation of boolean logic mentioned above:
            // First press, Current gets set to TRUE, the if() loop runs, and toggle gets set to TRUE. Servo moves to TRUE position.
            // Immediately after the press, CurrentMovement becomes FALSE and the outer if() loop does not run.
            // If the button is pressed again, Current is TRUE, the if() loop runs, now setting the toggle to FALSE.
            // Servo moves to FALSE position.

            // We use a screw on a servo to hold and release a rubber band that propels our paper airplane
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

            // Left and right claws can move independently of each other
            boolean leftClawCurrentMovement = gamepad1.left_bumper;
            if (leftClawCurrentMovement && !lastMovementCL) {
                toggleMovementCL = !toggleMovementCL;
                if (toggleMovementCL) {
                    clawLeft.setPosition(.15); // closed position
                } else {
                    clawLeft.setPosition(.35); // open position
                }
            }
            lastMovementCL = leftClawCurrentMovement;

            // Opposite of left claw
            boolean rightClawCurrentMovement = gamepad1.right_bumper;
            if (rightClawCurrentMovement && !lastMovementCR) {
                toggleMovementCR = !toggleMovementCR;
                if (toggleMovementCR) {
                    clawRight.setPosition(.65); // open position
                } else {
                    clawRight.setPosition(.85); // closed position
                }
            }
            lastMovementCR = rightClawCurrentMovement;

            // The wrist has 2 positions: one for scoring and one for collecting pixels.
            // In order to get full range of motion, we used a servo programmer.
            boolean toggleWrist = gamepad1.x;
            if (toggleWrist && !lastMovementWrist) {
                toggleMovementWrist = !toggleMovementWrist;
                if (toggleMovementWrist) {
                    clawAngle.setPosition(0.5);
                } else {
                    clawAngle.setPosition(1);
                }
            }
            lastMovementWrist = toggleWrist;
        }
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
