
package org.firstinspires.ftc.teamcode.drive.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.PIDclass;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.PIDhangClass;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.sliderMachineState;

import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.Halt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "TeleOp2", group = "TeleOp")
public class TeleOp2 extends OpMode {

//custom funcitons, used to save code space

    public ElapsedTime scoreWaitingTime = new ElapsedTime();

    public ElapsedTime hang1Time = new ElapsedTime();
    public ElapsedTime hang2Time = new ElapsedTime();


    public ElapsedTime sliderTime = new ElapsedTime();

    public ElapsedTime matchTime = new ElapsedTime();

    PIDhangClass hangPID = new PIDhangClass();

    public static int top = 4500;
    public static int bottom = 800;



    public boolean matchTimeNotStarted = true;
    public boolean ThirtySecWarning = true;
    public boolean FifteenSecWarning = true;


    public int hangTargetPos = 0;

    private Servo paperAirplane;


    public double slow = 1;


    public ElapsedTime debounceTime = new ElapsedTime();
    sliderMachineState executeSlides = new sliderMachineState();
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;
    private DcMotor slides;
    private DcMotor intake;



    private DcMotor hang1;
    private DcMotor hang2;

    private Servo arm1;
    private Servo arm2;

    private Servo wrist;
    private Servo finger1;

    private Servo finger2;


    private boolean gyroSquareRequested = false;
    private BNO055IMU imu; // Gyro sensor
    private boolean gyroResetRequested = false;

    public static double release = 0;

    private sliderMachineState.slidePosition preloadPos = sliderMachineState.slidePosition.LOW;
    private sliderMachineState.slidePosition exocutePos = sliderMachineState.slidePosition.THREATEN;

//TODO import my "halt" funciton
    //TODO adjust slider pid;



    @Override
    public void init() {

        arm1 = hardwareMap.servo.get("arm1");
        arm2 = hardwareMap.servo.get("arm2");
        wrist = hardwareMap.servo.get("wrist");
        finger1 = hardwareMap.servo.get("finger1");
        finger2 = hardwareMap.servo.get("finger2");
        arm1.setDirection(Servo.Direction.REVERSE);

        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");
        rearLeftMotor = hardwareMap.dcMotor.get("BL");
        rearRightMotor = hardwareMap.dcMotor.get("BR");
        intake = hardwareMap.dcMotor.get("intake");
        slides = hardwareMap.dcMotor.get("slides");


        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        paperAirplane = hardwareMap.servo.get("paperAirplane");



        hang1 = hardwareMap.dcMotor.get("hang1");
        hang2 = hardwareMap.dcMotor.get("hang2");

        hang2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //hang2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //hang1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hang2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



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
        //  imu.initialize(parameters);//todo; delete this comment
        scoreWaitingTime.reset();
        scoreWaitingTime.startTime();

        sliderTime.reset();
        sliderTime.startTime();

        hang1Time.reset();
        hang1Time.startTime();
        hang2Time.reset();
        hang2Time.startTime();

        debounceTime.reset();
        debounceTime.startTime();


        paperAirplane.setPosition(0.58);

        //waitForStart();
      //  runtime.reset();

    }

    @Override
    public void loop() {


        //telemetry
        telemetry.update();
        //telemetry.addData("Position of slides", slides.getCurrentPosition());
        telemetry.addData("Position of arm1", arm1.getPosition());
        telemetry.addData("Position of arm2", arm2.getPosition());
        //telemetry.addData("Position of finger1", finger1.getPosition());
        //telemetry.addData("Position of finger2", finger2.getPosition());
        telemetry.addData("Position of wrist", wrist.getPosition());
        telemetry.addData("exocute pos", exocutePos);
        telemetry.addData("preload pos", preloadPos);



        telemetry.addData("hang1", hang1.getCurrentPosition());
        telemetry.addData("hang2", hang2.getCurrentPosition());

        telemetry.addData("debounce time", debounceTime.milliseconds());
        telemetry.addData("scoreWaitingTime", scoreWaitingTime.milliseconds());



        //gamepad1 controls

        //wrist

        //fingers

        if (gamepad1.left_bumper) {
            finger1.setPosition(sliderMachineState.Finger1Loose);
            //finger2.setPosition(.5);
        }
        if (gamepad1.right_bumper) {
            //  finger1.setPosition(1);
            finger2.setPosition(sliderMachineState.Finger2Loose);
        }






       /* if(gamepad1.a){

            exocutePos = sliderMachineState.slidePosition.STAB;
            scoreWaitingTime.reset();
        }*/
        if(gamepad1.b){

            exocutePos = sliderMachineState.slidePosition.THREATEN;
            scoreWaitingTime.reset();
        }
//TODO fix potential issues with ellapsed tiime, as it it called in a seprate file


        //intake code


        if (gamepad1.left_trigger < .5) {
            intake.setPower(gamepad1.right_trigger);
        }
        if (gamepad1.right_trigger < .5) {
            intake.setPower(-gamepad1.left_trigger);

        }
       //second controller


        if (gamepad2.dpad_down) {

            preloadPos = sliderMachineState.slidePosition.LOW;


        }
        if (gamepad2.dpad_left) {

            preloadPos = sliderMachineState.slidePosition.MEDIUM;
        }
        if (gamepad2.dpad_up) {

            preloadPos = sliderMachineState.slidePosition.HIGH;
        }
        if (gamepad2.dpad_right) {

            preloadPos = sliderMachineState.slidePosition.REALLYHIGH;

        }

        //exocute pos
        if(gamepad2.y){
            exocutePos = preloadPos;
            scoreWaitingTime.reset();
        }
//stab
        if(gamepad2.a && exocutePos == sliderMachineState.slidePosition.THREATEN && debounceTime.milliseconds() > 500){


            exocutePos = sliderMachineState.slidePosition.STAB;
            scoreWaitingTime.reset();
            debounceTime.reset();
        }
        if(gamepad2.a && (exocutePos == sliderMachineState.slidePosition.STAB || exocutePos == sliderMachineState.slidePosition.STABAFTERSTAB) && debounceTime.milliseconds() > 500){

            exocutePos = sliderMachineState.slidePosition.RESTAB;
            scoreWaitingTime.reset();
            debounceTime.reset();
        }
        if(gamepad2.a && exocutePos == sliderMachineState.slidePosition.RESTAB && debounceTime.milliseconds() > 500){

            exocutePos = sliderMachineState.slidePosition.STABAFTERSTAB;
            scoreWaitingTime.reset();
            debounceTime.reset();
        }

        if(gamepad2.x){

            exocutePos = sliderMachineState.slidePosition.THREATEN;
            scoreWaitingTime.reset();
        }


//TODO fix potential issues with ellapsed tiime, as it it called in a seprate file


        //intake code

        if(gamepad1.a){
            slow = .5;
        }else{
            slow = 1;
        }

//hang



        if (gamepad2.b){

            paperAirplane.setPosition(0);

        }

        hangTargetPos = (int) (Math.round(hang1.getCurrentPosition() + 100 * (float) -gamepad2.left_stick_y));


        hangPID.magicPID(hang1, hangTargetPos, hang1Time);
        hangPID.magicPID(hang2, hangTargetPos, hang2Time);






//power slides

        executeSlides.magicalMacro(slides, arm1, arm2, wrist, finger1, finger2,
                exocutePos, scoreWaitingTime, sliderTime, false);


//Drive Code
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        // reset gyro button
   /*     if (gamepad1.a) {
            gyroResetRequested = true;
        }*/

        //TODO: update gyro reset?

        // Perform gyro reset if requested
        if (gyroResetRequested) {
            resetGyro();
            gyroResetRequested = false; // Reset the request
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
        frontLeftMotor.setPower(slow * frontLeftPower);
        frontRightMotor.setPower(slow * frontRightPower);
        rearLeftMotor.setPower(slow * rearLeftPower);
        rearRightMotor.setPower(slow *rearRightPower);

        // Update telemetry and control motors
        telemetry.addData("Gyro Heading", heading);
        telemetry.addData("slidesTime %2d", sliderTime.time());
        telemetry.update();

    }

    //This is used for field centric code but is outside of the loop
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