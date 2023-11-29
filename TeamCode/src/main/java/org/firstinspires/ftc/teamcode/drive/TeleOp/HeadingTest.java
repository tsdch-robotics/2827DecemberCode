package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class HeadingTest extends LinearOpMode {
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        //HardwareMap hardwareMap = hardwareMap;

        imu = hardwareMap.get(BNO055IMU.class, "imu"); // "imu" should match your configuration name

        // Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        // Wait for the IMU to calibrate
        while (!imu.isGyroCalibrated()) {
            telemetry.addData("IMU", "Calibrating...");
            telemetry.update();
        }
        telemetry.addData("IMU", "Calibrated.");
        telemetry.update();

        waitForStart();

        double zeroHeading = imu.getAngularOrientation().firstAngle; // Set the current heading as the zero position

        while (opModeIsActive()) {
            double currentHeading = imu.getAngularOrientation().firstAngle; // Get the current heading
            double relativeHeading = currentHeading - zeroHeading; // Calculate heading relative to the zero position

            telemetry.addData("Zero Heading", zeroHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Relative Heading", relativeHeading);
            telemetry.update();

            // Add your robot control logic here using the relative heading
        }
    }
}