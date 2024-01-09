package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// A more accurate implementation of Mecanum drive, using target angle/power as inputs instead of direct joystick values
@TeleOp(name = "robotCentric")
public class LinearTeleOp_robotCentricMethod2 extends BaseLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
}
