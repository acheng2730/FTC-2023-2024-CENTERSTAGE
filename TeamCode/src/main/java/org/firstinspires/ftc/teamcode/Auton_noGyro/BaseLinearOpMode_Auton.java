package org.firstinspires.ftc.teamcode.Auton_noGyro;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

// A working knowledge of Java is helpful here:
// abstract classes and inheritance
public abstract class BaseLinearOpMode_Auton extends LinearOpMode {
    public IMU imu;
    DcMotorEx topLeft, topRight, backLeft, backRight;
    DcMotorEx arm1, arm2;
    DcMotorEx hook;
    Servo planeLauncher;
    Servo clawAngle, clawLeft, clawRight;
    ElapsedTime timer = new ElapsedTime();
    double conversionFactor = 95; // NeveRest 40 motor ticks/inch

    public void initHardware() throws InterruptedException {
        // Hubs
        List<LynxModule> allHubs;
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        topLeft = hardwareMap.get(DcMotorEx.class, "topLeft");
        topRight = hardwareMap.get(DcMotorEx.class, "topRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
        hook = hardwareMap.get(DcMotorEx.class, "hook");
        planeLauncher = hardwareMap.get(Servo.class, "launcher");
        clawAngle = hardwareMap.get(Servo.class, "clawAngle");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE); // For mecanum drive

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setDirection(DcMotorSimple.Direction.REVERSE); // We used a motor on each side of the arm to support its weight better

        hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // sets encoder values to 0 in init
        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hook.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // does not stop encoder readings, but allows us to run motors at full power

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
    }

    public void driveTo(int pos, double time) { // pos in inches; time in seconds
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setTargetPosition((int) (pos * conversionFactor));
        topRight.setTargetPosition((int) (pos * conversionFactor));
        backLeft.setTargetPosition((int) (pos * conversionFactor));
        backRight.setTargetPosition((int) (pos * conversionFactor));

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topLeft.setVelocity(pos * conversionFactor / time);
        topRight.setVelocity(pos * conversionFactor / time);
        backLeft.setVelocity(pos * conversionFactor / time);
        backRight.setVelocity(pos * conversionFactor / time);
    }

    public void armTo(int pos, double time) { // pos in ticks and time in seconds
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setTargetPosition(pos);
        arm2.setTargetPosition(pos);

        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm1.setVelocity(pos / time);
        arm2.setVelocity(pos / time);
    }

    public void clawOpen() {
        clawLeft.setPosition(.2);
        clawRight.setPosition(.8);
    }

    public void clawClose() {
        clawLeft.setPosition(0.05);
        clawRight.setPosition(.95);
    }

    @Override
    public abstract void runOpMode() throws InterruptedException;

}