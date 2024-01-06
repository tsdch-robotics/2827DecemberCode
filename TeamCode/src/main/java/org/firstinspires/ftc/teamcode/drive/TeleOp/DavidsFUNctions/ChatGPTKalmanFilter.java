package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.SingularMatrixException;



public class ChatGPTKalmanFilter {
    private RealMatrix Q; // Process noise covariance
    private RealMatrix R; // Measurement noise covariance
    private RealMatrix state; // State
    private RealMatrix P; // Estimation error covariance

    public ChatGPTKalmanFilter(double[] initialState, double[][] initialP, double[][] processNoise, double[][] measurementNoise) {
        this.state = new Array2DRowRealMatrix(initialState);
        this.P = new Array2DRowRealMatrix(initialP);
        this.Q = new Array2DRowRealMatrix(processNoise);
        this.R = new Array2DRowRealMatrix(measurementNoise);
    }

    public void predict() {
        // Prediction step
        state = state;
        P = P.add(Q);
    }

    public void correct(double[] measurement) {
        // Correction step
        try {
            //RealMatrix K = P.multiply(P.transpose()).multiply(P.transpose().add(R)).inverse();

       //     state = state.add(K.multiply(new Array2DRowRealMatrix(measurement).subtract(state)));
          //  P = P.subtract(K.multiply(P)).multiply(P.transpose());
        } catch (SingularMatrixException e) {
            // Handle the case where the matrix is singular (not invertible)
            e.printStackTrace();
        }
    }

    public double[] getState() {
        return state.getColumn(0);
    }
}
