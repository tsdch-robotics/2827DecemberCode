
package org.firstinspires.ftc.teamcode.drive.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "BlueTeleOp", group = "TeleOp")
public class BlueTeleOp extends OpMode {





    double rotate = 0;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    private DcMotor slides;

    private DcMotor intake;


    private Servo arm1;
    private Servo arm2;

    private Servo wrist;
    private Servo finger1;

    private Servo finger2;




    private boolean gyroSquareRequested = false;
    private BNO055IMU imu; // Gyro sensor
    private boolean gyroResetRequested = false;

    private double kp = 0.02; // Proportional gain (tune this value)
    private double ki = 0.001; // Integral gain (tune this value)
    private double kd = 0.001; // Derivative gain (tune this value)
    private double integral = 0;
    private double prevError = 0;
    private boolean rotatingTo90 = false;
    double setpoint  = 90;

    int currentSlidesPosition;

    @Override
    public void init() {

        arm1 = hardwareMap.servo.get("arm1");
        arm2 = hardwareMap.servo.get("arm2");
        wrist = hardwareMap.servo.get("wrist");
        finger1 = hardwareMap.servo.get("finger1");
        finger2 = hardwareMap.servo.get("finger2");


        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");
        rearLeftMotor = hardwareMap.dcMotor.get("BL");
        rearRightMotor = hardwareMap.dcMotor.get("BR");
        intake = hardwareMap.dcMotor.get("intake");
        slides = hardwareMap.dcMotor.get("slides");
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

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

        telemetry.update();
        telemetry.addData("Position of slides", slides.getCurrentPosition());
        telemetry.addData("Position of arm1", arm1.getPosition());
        telemetry.addData("Position of arm2", arm2.getPosition());
        telemetry.addData("Position of finger1", finger1.getPosition());
        telemetry.addData("Position of finger2", finger2.getPosition());
        telemetry.addData("Position of wrist", wrist.getPosition());




        //wrist
        if(gamepad2.left_bumper){
            wrist.setPosition(.4 );//intaking

        }
        if(gamepad2.right_bumper){
            wrist.setPosition(.5);
        }
        //fingers

        if(gamepad2.left_trigger > .2){
            finger1.setPosition(0);
            finger2.setPosition(1);
        }
        if(gamepad2.right_trigger > .2){
            finger1.setPosition(.7);
            finger2.setPosition(.5);

        }


        //intake
        if(gamepad1.left_trigger < .5){
            intake.setPower(gamepad1.right_trigger);
        }
        if(gamepad1.right_trigger < .5){
            intake.setPower(-gamepad1.left_trigger);

        }

        if(gamepad2.dpad_up && slides.getCurrentPosition() < 3000){

            currentSlidesPosition = slides.getCurrentPosition();
            slides.setTargetPosition(currentSlidesPosition + 200);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.setPower(1);
        }
        if(gamepad2.dpad_down && slides.getCurrentPosition() > 0){


            slides.setTargetPosition(currentSlidesPosition - 200);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.setPower(1);
        }




        //arm control
        if(gamepad2.dpad_right){
//inmtake
            arm1.setPosition(.95);
            arm2.setPosition(.05);
         //   wrist.setPosition(.2);
        }
        if(gamepad2.dpad_left){


            arm1.setPosition(0);
            arm2.setPosition(1);
        }




        if(gamepad2.a){
            slides.setTargetPosition(0);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.setPower(1);
        }
        if(gamepad2.x){
            slides.setTargetPosition(1000);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.setPower(1);
        }
        if(gamepad2.y){
            slides.setTargetPosition(2300);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.setPower(1);
        }
        if(gamepad2.b){
            slides.setTargetPosition(3900);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.setPower(1);
        }




       /* if (gamepad1.left_bumper){
            setpoint = 90;
        }
        if (gamepad1.right_bumper){

            setpoint = 45;
        }*/

        // Calculate the error (how far off the robot is from the setpoint)
        double error = setpoint - getPitch();

        // Update the integral term (helps eliminate steady-state error)
        integral += error;

        // Calculate the derivative term (helps reduce overshooting)
        double derivative = error - prevError;









        // Calculate the PID output (rotate)
        // Check if the B button is pressed for rotating to 90 degrees
        if (gamepad1.left_bumper && !rotatingTo90) {
            setpoint = 90;
            rotatingTo90 = true;
            integral = 0; // Reset the integral term
        }
        if (gamepad1.right_bumper && !rotatingTo90) {
            setpoint = 135;
            rotatingTo90 = true;
            integral = 0; // Reset the integral term
        }





        if (rotatingTo90) {
            // Calculate the PID output (rotate)
            rotate = kp * error + ki * integral + kd * derivative;

            // Check if the robot is close enough to 90 degrees and stop
            if (Math.abs(error) < 5) {
                rotatingTo90 = false;
                rotate = 0;
            }
        } else {
            // Use right stick control when not rotating to 90 degrees
            rotate = -gamepad1.right_stick_x;
        }
        // Update the previous error for the next iteration
        prevError = error;

        // Apply the PID output to the robot's rotation
        // You may need to adjust the scaling factor to match your robot's behavior
        rotate = Range.clip(rotate, -1.0, 1.0);


        // Get joystick inputs from the gamepad
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;


        if(gamepad1.dpad_left){
            strafe = .75;
        }
        if(gamepad1.dpad_right){
            strafe = -.75;
        }

        // reset gyro button
   /*     if (gamepad1.a) {
            gyroResetRequested = true;
        }*/

        //TODO: update gyro reset?

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

        // Update telemetry and control motors
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