
package org.firstinspires.ftc.teamcode.drive.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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





    int currentSlidesPosition;
    public int targetSlidesHeight;


    public enum SlidePosition {

        THREATEN, //waiting to stab
        STAB, //stabing

        ZERO, //0
        LOW, //500
        MEDIUM,//1500
        HIGH;//2500
    }

    public SlidePosition slideTarget;
    public SlidePosition slidePreload;
    public SlidePosition slideCurrentState;



//slide PID stuff
    public int slidePIDtargetPosition;
    public int slidePIDerror;
    public int slidePIDintegral;
    public int slidePIDderivative;
    public int slidePIDlastError;
    public int slidePIDcorrection;
    public int slidePIDpower;

    private double slideKp = 0.1; // Proportional gain
    private double slideKi = 0.01; // Integral gain
    private double slideKd = 0.001; // Derivative gain

    private int slideTargetPosition = 0;
    private int slideError, slideIntegral, slideDerivative, slideLastError;
    private int slideCorrection, slidePower;







    public void halt(int seconds) {
        try {
            Thread.sleep(seconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

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


        slideTarget = SlidePosition.ZERO;
        slideCurrentState = SlidePosition.ZERO;



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





        //gamepad1 controls

        //wrist

        //fingers

        if(gamepad1.left_bumper){
        //    finger1.setPosition(release);
            //finger2.setPosition(.5);
        }
        if(gamepad1.right_bumper){
          //  finger1.setPosition(1);
            //finger2.setPosition(release);
        }


        if(gamepad1.dpad_down){

        }
        if(gamepad1.dpad_left){
            slidePreload = SlidePosition.LOW;
        }
        if(gamepad1.dpad_up){
            slidePreload = SlidePosition.HIGH;
        }
        if(gamepad1.dpad_right){
            slidePreload = SlidePosition.MEDIUM;
        }
        if(gamepad1.a){

            if (slideCurrentState == SlidePosition.THREATEN){
                slideTarget = SlidePosition.STAB;
            }else{
                slideTarget = SlidePosition.THREATEN;
            }
        }else if(gamepad1.b){
            slideTarget = slidePreload;
        }

        //intake
        if(gamepad1.left_trigger < .5){
            intake.setPower(gamepad1.right_trigger);
        }
        if(gamepad1.right_trigger < .5){
            intake.setPower(-gamepad1.left_trigger);

        }





        //arm control


//THIS is slider PID powersetting code:


/*  public int slidePIDtargetPosition;
    public int slidePIDerror;
    public int slidePIDintegral;
    public int slidePIDderivative;
    public int slidePIDlastError;
    public int slidePIDcorrection;
    public int slidePIDkp;
    public int slidePIDpower;

*/

        // Calculate PID components
        slidePIDerror = slidePIDtargetPosition - currentSlidesPosition;
        slidePIDintegral += slidePIDerror;
        slideDerivative = slideError - slideLastError;

// Calculate PID correction
        slideCorrection = slideKp * slideError + slideKi * slideIntegral + slideKd * slideDerivative;

// Calculate motor power
        slidePower = /* Your base slide motor power logic here */ + slideCorrection;

// Apply motor power with bounds to avoid exceeding limits
        slides.setPower(Range.clip(slidePower, -1, 1));

// Update last error
        slideLastError = slideError;



//THIS is drivcode:


        rotate = Range.clip(rotate, -1.0, 1.0);

        // Get joystick inputs from the gamepad
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;

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
}