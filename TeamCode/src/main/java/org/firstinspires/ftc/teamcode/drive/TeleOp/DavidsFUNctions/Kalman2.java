package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class Kalman2 {

    double estimatedState; // your initial statex
    public static double modelCovariance = 0.1;//q
    public static double sensorCovariance = 2;//r
    public static double covarianceGuess = 1;//p
    double kalmanGain;//k

    double previousState = estimatedState;
    double previousCovariance = covarianceGuess;

    public Vector2d Kalmanized(Vector2d vector) {

        // Prediction Step
        double delta = vector.getX() - previousState;
        estimatedState = previousState + delta;
        previousCovariance += modelCovariance;

        // Update Step (using only one sensor measurement)
        double measurement = vector.getX(); // Simulated noisy measurement.
        kalmanGain = previousCovariance / (previousCovariance + sensorCovariance);
        estimatedState = estimatedState + kalmanGain * (measurement - estimatedState);
        previousCovariance = (1 - kalmanGain) * previousCovariance;

        // Update previous state and covariance
        previousState = estimatedState;

        Vector2d kalmanizedVector = new Vector2d(estimatedState, vector.getY());

        return kalmanizedVector;
    }
}


