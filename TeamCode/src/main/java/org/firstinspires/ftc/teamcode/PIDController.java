package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double Kp, Ki, Kd;
    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double calculate(double reference, double state) {
        ElapsedTime timer = new ElapsedTime();
        double integralSum = 0;
        double lastError = 0;
        double lastReference = reference;
        double a = 0.8; // a can be anything from 0 < a < 1
        double previousFilterEstimate = 0;
        double currentFilterEstimate = 0;
        double error = reference - state;
        double errorChange = (error - lastError);

        // filter out high frequency noise to increase derivative performance
        currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;
        integralSum += error * timer.seconds();
        if (integralSum > .25) {
            integralSum = .25;
        }
        if (integralSum < -.25) {
            integralSum = -.25;
        }

        if (reference != lastReference) {
            integralSum = 0;
        }

        double derivative = (error - lastError) / timer.seconds();

        double output = (error * Kp) + (derivative + Kd) + (integralSum * Ki);

        lastError = error;
        lastReference = reference;

        timer.reset();

        return output;
    }
}

