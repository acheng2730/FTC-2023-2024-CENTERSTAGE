package org.firstinspires.ftc.teamcode;

import static dev.frozenmilk.dairy.calcified.hardware.sensor.CalcifiedIMUKt.fromImuOrientationOnRobot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.frozenmilk.dairy.calcified.Calcified;
import dev.frozenmilk.dairy.calcified.gamepad.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.calcified.gamepad.EnhancedNumberSupplier;
import dev.frozenmilk.dairy.calcified.gamepad.EnhancedNumberSupplierKt;
import dev.frozenmilk.dairy.calcified.hardware.controller.LambdaController;
import dev.frozenmilk.dairy.calcified.hardware.controller.LambdaControllerKt;
import dev.frozenmilk.dairy.calcified.hardware.controller.PController;
import dev.frozenmilk.dairy.calcified.hardware.motor.CalcifiedEncoder;
import dev.frozenmilk.dairy.calcified.hardware.motor.CalcifiedMotor;
import dev.frozenmilk.dairy.calcified.hardware.motor.DegreesEncoder;
import dev.frozenmilk.dairy.calcified.hardware.motor.Direction;
import dev.frozenmilk.dairy.calcified.hardware.motor.RadiansEncoder;
import dev.frozenmilk.dairy.calcified.hardware.motor.ZeroPowerBehaviour;
import dev.frozenmilk.dairy.calcified.hardware.sensor.AnalogInput;
import dev.frozenmilk.dairy.calcified.hardware.sensor.CalcifiedIMU;
import dev.frozenmilk.dairy.calcified.hardware.sensor.DigitalInput;
import dev.frozenmilk.dairy.calcified.hardware.sensor.DigitalOutput;
import dev.frozenmilk.dairy.calcified.hardware.servo.CalcifiedContinuousServo;
import dev.frozenmilk.dairy.calcified.hardware.servo.CalcifiedServo;
import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.OpModeLazyCell;
import dev.frozenmilk.util.angle.Angle;
import dev.frozenmilk.util.angle.AngleDegrees;
import dev.frozenmilk.util.angle.AngleRadians;
import dev.frozenmilk.util.cell.Cell;
import dev.frozenmilk.util.orientation.AngleBasedRobotOrientation;
import dev.frozenmilk.util.profile.ProfileConstraints;
import dev.frozenmilk.util.profile.ProfileStateComponent;
@TeleOp(name="calcified test")
public class CalcifiedRobotTesting extends BaseOpMode {
    boolean lastMovementLauncher = false;
    boolean toggleMovementLauncher = false;
    boolean lastMovementWrist = false;
    boolean toggleMovementWrist = false;
    boolean lastMovementCL = false;
    boolean toggleMovementCL = false;
    boolean lastMovementCR = false;
    boolean toggleMovementCR = false;

    public CalcifiedRobotTesting() {
        FeatureRegistrar.checkFeatures(this, Calcified.INSTANCE);
    }

    @Override
    public void init() {
        initHardware();

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init_loop() {
        // won't use
    }

    @Override
    public void start() {
        clawAngle.setPosition(1);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y * -1;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x * 1.1;

        double denominator = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1);
        double topLeftPow = (drive + turn + strafe) / denominator;
        double backLeftPow = (drive + turn - strafe) / denominator;
        double topRightPow = (drive - turn - strafe) / denominator;
        double backRightPow = (drive - turn + strafe) / denominator;

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

    @Override
    public void stop() {
    }
}