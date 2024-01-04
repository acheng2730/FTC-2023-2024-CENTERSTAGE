package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public abstract class BaseLinearOpMode extends LinearOpMode {
    public IMU imu;
    DcMotor topLeft, topRight, backLeft, backRight;
    DcMotor arm1, arm2;
    Servo planeLauncher;
    Servo clawAngle, clawLeft, clawRight;
    ElapsedTime timer = new ElapsedTime();

    /* int topLeftEncoderPos = topLeft.getCurrentPosition();
    int topRightEncoderPos = topRight.getCurrentPosition();
    int backLeftEncoderPos = backLeft.getCurrentPosition();
    int backRightEncoderPos = backRight.getCurrentPosition();
    int arm1EncoderPos = arm1.getCurrentPosition();
    int arm2EncoderPos = arm2.getCurrentPosition();
    double launcherPos = planeLauncher.getPosition();
    double clawAnglePos = clawAngle.getPosition();
    double clawLeftPos = clawLeft.getPosition();
    double clawRightPos = clawRight.getPosition(); */
    public void initHardware() throws InterruptedException {
        // Hubs
        List<LynxModule> allHubs;
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        topLeft = hardwareMap.get(DcMotor.class, "topLeft");
        topRight = hardwareMap.get(DcMotor.class, "topRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        planeLauncher = hardwareMap.get(Servo.class, "launcher");
        clawAngle = hardwareMap.get(Servo.class, "clawAngle");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        //topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
    }


    @Override
    public abstract void runOpMode() throws InterruptedException;

}