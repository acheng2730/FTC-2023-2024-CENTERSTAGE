package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// A more accurate implementation of Mecanum drive, using target angle/power as inputs instead of direct joystick values
@TeleOp(name = "robotCentric-method2")
public class LinearTeleOp_robotCentric_Method2 extends BaseLinearOpMode {
    double curPoseY = 0, curPoseX = 0; // Current position on field in inches
    ElapsedTime driveTime = new ElapsedTime();
    double prevTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // The program will loop several times while a button is being pressed, causing servos to jitter.
        // This boolean logic solves that problem by ensuring actions occur only once per press.
        // This is only needed because we use a single button to toggle each servo; using a button to set a single servo position
        // can be done intuitively.
        boolean lastMovementLauncher = false;
        boolean toggleMovementLauncher = false;

        boolean lastMovementWristUp = false;
        boolean toggleMovementWristUp = false;

        boolean lastMovementWristDown = false;
        boolean toggleMovementWristDown = false;

        boolean lastMovementClaw = false;
        boolean toggleMovementClaw = false;

        boolean lastModeHook = false;
        boolean toggleModeHook = false;

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
            telemetry.addData("Claw Pos (L/R): ", clawLeftPos + "/" + clawRightPos);
            telemetry.addData("topLeftPos: ", topLeftEncoderPos);
            telemetry.addData("topRightPos: ", topRightEncoderPos);
            telemetry.addData("backLeftPos: ", backLeftEncoderPos);
            telemetry.addData("backRightPos: ", backRightEncoderPos);
            telemetry.addData("Hook Mode: ", hook.getZeroPowerBehavior());
            telemetry.update();


            // Mecanum drivetrain implementation
            // MUST READ: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
            // https://www.youtube.com/watch?v=gnSW2QpkGXQ

            double strafe = gamepad1.left_stick_x * 1.1;
            double drive = gamepad1.left_stick_y * -1;
            double turn = gamepad1.right_stick_x;

            double theta = Math.atan2(drive, strafe); // Desired bot heading
            double power = Math.hypot(strafe, drive); // Desired power

            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4); // Rotating the bot heading here by -45 degrees makes the
            // mecanum wheels' vectors the x,y components of the power vector
            double max = Math.max(Math.abs(sin), Math.abs(cos)); // Scale motors so at least one is max power

            double topLeftPow = power * cos / max + turn;
            double backLeftPow = power * sin / max + turn;
            double topRightPow = power * sin / max - turn;
            double backRightPow = power * cos / max - turn;

            if ((power + Math.abs(turn)) > 1) { // Avoid power clipping
                topLeftPow /= power + turn;
                backLeftPow /= power + turn;
                topRightPow /= power + turn;
                backRightPow /= power + turn;
            }

            topLeft.setPower(topLeftPow);
            backLeft.setPower(backLeftPow);
            topRight.setPower(topRightPow);
            backRight.setPower(backRightPow);


            arm1.setDirection(DcMotorSimple.Direction.REVERSE); // We used a motor on each side of the arm to support its weight better
            double armPower = (-.2 * gamepad1.left_trigger + .5 * gamepad1.right_trigger); // Slower downwards movement b/c gravity

            arm1.setPower(armPower);
            arm2.setPower(armPower);

            // Continuation of boolean logic mentioned above:
            // First press, Current gets set to TRUE, the if() loop runs, and toggle gets set to TRUE. Servo moves to TRUE position.
            // Immediately after the press, CurrentMovement becomes FALSE and the outer if() loop does not run.
            // If the button is pressed again, Current is TRUE, the if() loop runs, now setting the toggle to FALSE.
            // Servo moves to FALSE position.

            // When hanging, use BRAKE, when playing, use FLOAT so driver 2 doesn't have to feed line while scoring.
            boolean hookCurrentMode = gamepad2.start;
            if (hookCurrentMode && !lastModeHook) {
                toggleModeHook = !toggleModeHook;
                if (toggleModeHook) {
                    hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } else {
                    hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }
            hook.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

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

            // Claw hands - considered letting left and right move independently, but rebuilt the
            // claw and decided not to.
            boolean clawCurrentMovement = gamepad1.left_bumper;
            if (clawCurrentMovement && !lastMovementClaw) {
                toggleMovementClaw = !toggleMovementClaw;
                if (toggleMovementClaw) {
                    clawLeft.setPosition(0); // closed position
                    clawRight.setPosition(1);
                } else {
                    clawLeft.setPosition(.3); // open position
                    clawRight.setPosition(.7);
                }
            }
            lastMovementClaw = clawCurrentMovement;

            // The wrist has 2 positions: one for scoring and one for collecting pixels.
            // In order to get 360 degree range of motion, we used a servo programmer.
            boolean toggleWristUp = gamepad1.x;
            if (toggleWristUp && !lastMovementWristUp) {
                toggleMovementWristUp = !toggleMovementWristUp;
                if (toggleMovementWristUp) {
                    clawAngle.setPosition(clawAngle.getPosition()+.05);
                }
            }
            lastMovementWristUp = toggleWristUp;

            boolean toggleWristDown = gamepad1.y;
            if (toggleWristDown && !lastMovementWristDown) {
                toggleMovementWristDown = !toggleMovementWristDown;
                if (toggleMovementWristDown) {
                    clawAngle.setPosition(clawAngle.getPosition()-.05);
                }
            }
            lastMovementWristDown = toggleWristDown;
        }
    }

    public void updatePosition() { // uses encoders to determine position on the field
        // MUST READ: https://ftc-tech-toolbox.vercel.app/docs/odo/Mecanum
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
