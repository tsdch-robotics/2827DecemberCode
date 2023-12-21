package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Kalman {

    double x = 0; // your initial state
    double Q = .1; // your model covariance.1
    double R = 2; // your sensor covariance.4
    double p = 1; // your initial covariance guess
    double K; // Kalman gain

    double x_previous = x;
    double p_previous = p;
    double u = 0;
    double z = 0;

    public Vector2d Kalmanized(Vector2d myVector) {

        // Prediction Step
        u = myVector.getX() - x_previous;
        x = x_previous + u;
        p = p_previous + Q;

        // Update Step (using only one sensor measurement)
        z = myVector.getX(); // Simulated noisy measurement.
        K = p / (p + R);
        x = x + K * (z - x);
        p = (1 - K) * p;

        // Update previous state and covariance
        x_previous = x;
        p_previous = p;

        Vector2d KalmaizedVector = new Vector2d(x, myVector.getY());

        return KalmaizedVector;
    }
}
