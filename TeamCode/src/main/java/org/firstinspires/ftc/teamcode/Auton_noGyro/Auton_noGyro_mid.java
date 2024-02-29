package org.firstinspires.ftc.teamcode.Auton_noGyro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "midSpike")
public class Auton_noGyro_mid extends BaseLinearOpMode_Auton {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        sleep(500);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();

        driveTo(10,1);
        sleep(100);

        armTo(90,.5);
        clawAngle.setPosition(0.1);
        sleep(100);

        clawOpen();
    }
}
