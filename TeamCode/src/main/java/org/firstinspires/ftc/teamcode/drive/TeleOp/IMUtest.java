
package org.firstinspires.ftc.teamcode.drive.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "Special", group = "TeleOp")
public class IMUtest extends OpMode {

    double rotate = 0;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;


    private boolean gyroSquareRequested = false;
    private BNO055IMU imu; // Gyro sensor
    private boolean gyroResetRequested = false;

    private double kp = 0.02; // Proportional gain (tune this value)
    private double ki = 0.001; // Integral gain (tune this value)
    private double kd = 0.001; // Derivative gain (tune this value)
    private double integral = 0;
    private double prevError = 0;
    private boolean rotatingTo90 = false;
    double setpoint  = 0;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");
        rearLeftMotor = hardwareMap.dcMotor.get("BL");
        rearRightMotor = hardwareMap.dcMotor.get("BR");


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Initialize the gyro sensor
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //imuParams.LOGO_FACING_DIR, USB_FACING_DIR
        imuParams.calibrationDataFile = "BNO055IMUCalibration.json"; // Set this to your calibration file
        imuParams.loggingEnabled = true;
        imuParams.loggingTag = "IMU";
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);
        //  IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        //          LOGO_FACING_DIR, USB_FACING_DIR));
        //  imu.initialize(parameters);
    }
    @Override
    public void loop() {

      /* if (Math.abs(gamepad1.left_stick_x) != gamepad1.left_stick_x && Math.abs(gamepad1.left_stick_y) != gamepad1.left_stick_y){
           //-1, -1
       }*

       */


        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
            setpoint = setpoint;
        }
        else if (gamepad1.left_stick_x < -.7 && gamepad1.left_stick_y == 0) {
            setpoint = -90;
        }
        else if (gamepad1.left_stick_x > .7 && gamepad1.left_stick_y == 0) {
            setpoint = 90;
        }

        else if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y > .7) {
            setpoint = 0;
        }
        else if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y < -.7) {
            setpoint = 0;
        }

        else if (gamepad1.left_stick_x != 0 && gamepad1.left_stick_y != 0 && (Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2))) >= .7) {
            setpoint = Math.toDegrees(Math.atan(gamepad1.left_stick_x / gamepad1.left_stick_y));
        }
        else{
            setpoint = setpoint;
        }

        // setpoint = xcdfsg

        //double setpoint = 90; // The target heading (in degrees)

// Calculate the error (how far off the robot is from the setpoint)
        double error = setpoint - getPitch();

// Update the integral term (helps eliminate steady-state error)
        integral += error;

// Calculate the derivative term (helps reduce overshooting)
        double derivative = error - prevError;

// Calculate the PID output (rotate)
        rotate = kp * error + ki * integral + kd * derivative;

// Update the previous error for the next iteration
        prevError = error;

// Apply the PID output to the robot's rotation
// You may need to adjust the scaling factor to match your robot's behavior
        rotate = Range.clip(rotate, -1.0, 1.0);


        // Get joystick inputs from the gamepad
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;

        // reset gyro button
        if (gamepad1.a) {
            gyroResetRequested = true;
        }

        // Perform gyro reset if requested
        if (gyroResetRequested) {
            resetGyro();
            gyroResetRequested = false; // Reset the request flag
        }

        // Get the robot's heading from the gyro sensor
        double heading = getPitch();
        // Calculate the joystick inputs in the field-oriented frame of reference
        double fieldDrive = drive * Math.cos(Math.toRadians(heading)) - strafe * Math.sin(Math.toRadians(heading));
        double fieldStrafe = drive * Math.sin(Math.toRadians(heading)) + strafe * Math.cos(Math.toRadians(heading));
        // Calculate motor powers for mecanum drive

        double frontLeftPower = fieldDrive + fieldStrafe + rotate;
        double frontRightPower = fieldDrive - fieldStrafe - rotate;
        double rearLeftPower = fieldDrive - fieldStrafe + rotate;
        double rearRightPower = fieldDrive + fieldStrafe - rotate;

        // Ensure motor powers are within the valid range of -1 to 1
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
        rearLeftPower = Range.clip(rearLeftPower, -1.0, 1.0);
        rearRightPower = Range.clip(rearRightPower, -1.0, 1.0);

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        rearLeftMotor.setPower(rearLeftPower);
        rearRightMotor.setPower(rearRightPower);

        // Display motor powers on telemetry (optional)
        // telemetry.addData("Front Left Power", frontLeftPower);
        //  telemetry.addData("Front Right Power", frontRightPower);
        //   telemetry.addData("Rear Left Power", rearLeftPower);
        //     telemetry.addData("Rear Right Power", rearRightPower);
        //telemetry.update();

        // Update telemetry and control m otors
        telemetry.addData("Setpoint:", setpoint);
        telemetry.addData("Gyro Heading", heading);
        telemetry.update();

    }

    private double getPitch() {
        // Get the robot's heading from the gyro sensor
        return imu.getAngularOrientation().firstAngle;
    }
    private void resetGyro() {
        // Reset the gyro orientation to its initial state
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // Optional: Load a calibration file if available
        imu.initialize(parameters);
    }
    private void turnClockwise(){
        frontLeftMotor.setPower(1);
        frontRightMotor.setPower(-1);
        rearLeftMotor.setPower(1);
        rearRightMotor.setPower(-1);
    }

    private void turnCounter(){
        frontLeftMotor.setPower(-1);
        frontRightMotor.setPower(1);
        rearLeftMotor.setPower(-1);
        rearRightMotor.setPower(1);
    }



}