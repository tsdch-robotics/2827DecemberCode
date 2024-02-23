
package org.firstinspires.ftc.teamcode.drive.TeleOp;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Autonomous.Misc.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.PIDclass;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.PIDhangClass;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.sliderMachineState;

import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Run this TeleOp!", group = "TeleOp")
public class TeleOp extends OpMode {

//custom funcitons, used to save code space

    public boolean nearBoard = false;
    public Pose2d myPose;
    double IMUsetpoint = 0;
    double rotate = 0;
    double strafe = 0;
    private double IMUkp = 0.02; // Proportional gain (tune this value)
    private double IMUki = 0.001; // Integral gain (tune this value)
    private double IMUkd = 0.001; // Derivative gain (tune this value)
    private double IMUintegral = 0;
    private double IMUprevError = 0;
    private boolean rotatingTo90 = false;

    double distanceLeftsetpoint = 0;
    double distanceOffsetLeft = 0;
    double distanceOffsetRight = 0;

    boolean align = false;

    public static double sensorLeftkp = 0.02; // Proportional gain (tune this value)
    public static double sensorRightkp = 0.02;

    public static double sensorLeftki = 0.001; // Integral gain (tune this value)
    public static double sensorRightki = 0.001;

    public static double sensorLeftkd = 0.001;
    public static double sensorRigtkd = 0.001;

    // Derivative gain (tune this value)
    public double sensorLeftintegral = 0;
    public double sensorRightintegral = 0;

    public double sensorLeftprevError = 0;
    public double sensorRightprevError = 0;

    /*double sensorLefterror = sensorLeftsetpoint - tsL.getDistance(DistanceUnit.INCH);
            double sensorRighterror = sensorRightsetpoint - tsR.getDistance(DistanceUnit.INCH);

            // Update the integral term (helps eliminate steady-state error)
            sensorLeftintegral += sensorLefterror;
            sensorRightintegral += sensorRighterror;

            // Calculate the derivative term (helps reduce overshooting)
            double sensorLeftderivative = sensorLefterror - sensorLeftprevError;
            double sensorRightderivative = sensorRighterror - sensorLeftprevError;

            thingu = snesorLeftkp * sensorLefterror + sensorLeftki * sensorLeftintegral + sensorLeftkd * sensorLeftderivative;
            thingu2 = snesorRightkp * sensorLefterror + sensorRightki * sensorRightintegral + sensorRightkd * sensorRightderivative;

            // Update the previous error for the next iteration
            sensorRightprevError = sensorLefterror;
            sensorRightprevError = sensorLefterror;
*/

    DistanceSensor tsL;
    DistanceSensor tsR;

    PIDclass slidesPID = new PIDclass();

    public boolean recentDpad = true;

    int heightOffset = 0;
    private boolean currentlyScoring = false;

    TouchSensor touchSensor;

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

    private Servo intakeLeft;
    private Servo intakeRight;

    private Servo finger2;

    private boolean gyroSquareRequested = false;
    private BNO055IMU imu; // Gyro sensor
    public double IMUposeTransfercorrection = 0;
    private boolean gyroResetRequested = false;


    public static double release = 0;

    private sliderMachineState.slidePosition preloadPos = sliderMachineState.slidePosition.LOW;
    private sliderMachineState.slidePosition exocutePos = sliderMachineState.slidePosition.THREATEN;

    int lastManualPosition = 800;

//TODO import my "halt" funciton
    //TODO adjust slider pid;

    @Override
    public void init() {

       /*SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14.5, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);*/

        arm1 = hardwareMap.servo.get("arm1");
        arm2 = hardwareMap.servo.get("arm2");
        wrist = hardwareMap.servo.get("wrist");
        finger1 = hardwareMap.servo.get("finger1");
        finger2 = hardwareMap.servo.get("finger2");
        arm2.setDirection(Servo.Direction.REVERSE);

        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");
        rearLeftMotor = hardwareMap.dcMotor.get("BL");
        rearRightMotor = hardwareMap.dcMotor.get("BR");
        intake = hardwareMap.dcMotor.get("intake");
        slides = hardwareMap.dcMotor.get("slides");

        intakeLeft = hardwareMap.servo.get("intakeLeft");
        intakeRight = hardwareMap.servo.get("intakeRight");


        //tsL = hardwareMap.get(DistanceSensor.class, "tsL");
        //tsR = hardwareMap.get(DistanceSensor.class, "tsR");

        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        paperAirplane = hardwareMap.servo.get("paperAirplane");

        touchSensor = hardwareMap.get(TouchSensor.class, "touch");



        hang1 = hardwareMap.dcMotor.get("hang1");
        hang2 = hardwareMap.dcMotor.get("hang2");



        hang2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

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


//getting rid of imu initialization
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



        IMUposeTransfercorrection = (Math.toDegrees(PoseStorage.currentPose.getHeading()) + 90);

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


        paperAirplane.setPosition(.5);

        //waitForStart();
        //  runtime.reset();

    }

    @Override
    public void loop() {


        /*telemetry.addData("x", myPose.getX());
        telemetry.addData("y", myPose.getY());
        telemetry.addData("heading", myPose.getHeading());
*/

        //telemetry
        telemetry.update();
        telemetry.addData("Position of slides", slides.getCurrentPosition());
        telemetry.addData("Position of arm1", arm1.getPosition());
        telemetry.addData("Position of arm2", arm2.getPosition());
        //telemetry.addData("Position of finger1", finger1.getPosition());
        //telemetry.addData("Position of finger2", finger2.getPosition());
        telemetry.addData("Position of wrist", wrist.getPosition());
        telemetry.addData("exocute pos", exocutePos);
        telemetry.addData("preload pos", preloadPos);
        telemetry.addData("IMU correction", IMUposeTransfercorrection);
        //telemetry.addData("tsL", tsL.getDistance(DistanceUnit.INCH));
        //telemetry.addData("tsR", tsR.getDistance(DistanceUnit.INCH));

        if(touchSensor.isPressed()){
            telemetry.addLine("Touchsensor is pressed");
        }else{
            telemetry.addLine("Touchsensor is not pressed");
        }



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

        if (gamepad1.left_trigger > .1) {
            intake.setPower(-gamepad1.left_trigger);
        }else if (gamepad1.right_trigger > .1) {
            intake.setPower(gamepad1.right_trigger);
        }else if (gamepad2.left_trigger > .2) {
            intake.setPower(gamepad2.left_trigger);
        }else if (gamepad2.right_trigger > .2) {
            intake.setPower(-gamepad2.right_trigger);
        }else{
            intake.setPower(0);
        }

        if (gamepad1.dpad_down ) {
            intakeLeft.setPosition(.35);
            intakeRight.setPosition(.35);

        }else if (gamepad1.dpad_up ) {
            intakeLeft.setPosition(.2);
            intakeRight.setPosition(.2);

        }else if (gamepad1.dpad_left ) {
            intakeLeft.setPosition(.25);
            intakeRight.setPosition(.25);

        } else if (gamepad1.dpad_right ) {
            intakeLeft.setPosition(.11);
            intakeRight.setPosition(.11);

        }



        //7.5

        // 10.5-14.9

        //4.4, 32 rotations

        //32/4.4sec == 7.27 rotations per second * 60 = 436rpm
        //1150/60sec == 19.6 rotations per second = 1150 rpm
        //320/60sed == 5.33 rotations per second = 320rpm
        //435/60sed == 7.25 rotations per second = 435rpm

        //second controller

        if (gamepad2.dpad_down) {

            heightOffset = 0;
            preloadPos = sliderMachineState.slidePosition.LOW;
            recentDpad = true;
        }
        if (gamepad2.dpad_left) {

            heightOffset = 0;
            preloadPos = sliderMachineState.slidePosition.MEDIUM;
            recentDpad = true;
        }


        if (gamepad2.dpad_up) {

            heightOffset = 0;
            preloadPos = sliderMachineState.slidePosition.HIGH;
            recentDpad = true;
        }

        if (gamepad2.dpad_right) {

            heightOffset = 0;
            preloadPos = sliderMachineState.slidePosition.REALLYHIGH;
            recentDpad = true;
        }

/*
        if(gamepad2.b && slides.getCurrentPosition() > 600){
            heightOffset = (int) (Math.round(heightOffset + 50 * (float) -gamepad1.right_stick_y));
        }else if(gamepad1.dpad_down){
            heightOffset -= 150;

        }else if(gamepad1.dpad_up){
            heightOffset += 150;

        }else if(gamepad1.dpad_right){
            heightOffset += 75;

        }else if(gamepad1.dpad_left){
            heightOffset += 75;

        }

        else{
            heightOffset = (int) (Math.round(heightOffset + 200 * (float) -gamepad2.left_stick_y));
        }*/

        if(Math.abs(gamepad2.left_stick_y) > 0.1){

            exocutePos = sliderMachineState.slidePosition.MANUAL;
            if(slides.getCurrentPosition() > 0 && slides.getCurrentPosition() < 3000){
                slides.setPower(-gamepad2.left_stick_y);
                lastManualPosition = slides.getCurrentPosition();

            }
        } else if (exocutePos == sliderMachineState.slidePosition.MANUAL) {


            slidesPID.magicPID(slides, lastManualPosition, sliderTime);

        }




        //exocute pos
        if(gamepad2.y && recentDpad){
            exocutePos = preloadPos;
            scoreWaitingTime.reset();
        }

        if(gamepad2.y && !recentDpad){
            exocutePos = sliderMachineState.slidePosition.RESTORESTICK;
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
            recentDpad = false;
        }else if(gamepad1.x){

            exocutePos = sliderMachineState.slidePosition.THREATEN;
            scoreWaitingTime.reset();
            recentDpad = false;

            intakeLeft.setPosition(.45);
            intakeRight.setPosition(.45);

        }





//TODO fix potential issues with ellapsed tiime, as it it called in a seprate file


        //intake code

        if(gamepad1.a){
            slow = .5;
        }else{
            slow = 1;
        }

//hang



        if (gamepad1.b) {

            paperAirplane.setPosition(.6);
        }

        hangTargetPos = (int) ( ((Math.round(hang1.getCurrentPosition())+ Math.round(hang2.getCurrentPosition()))/2) + 400 * (float) -gamepad2.right_stick_y);


        hangPID.magicPID(hang1, hangTargetPos, hang1Time);
        hangPID.magicPID(hang2, hangTargetPos, hang2Time);



//power slides

        executeSlides.magicalMacro(slides, arm1, arm2, wrist, finger1, finger2,
                exocutePos, scoreWaitingTime, sliderTime, touchSensor, false, heightOffset, lastManualPosition);


        //chekc if scoring
        if(exocutePos == sliderMachineState.slidePosition.HIGH ||
                exocutePos == sliderMachineState.slidePosition.REALLYHIGH ||
                exocutePos == sliderMachineState.slidePosition.LOW ||
                exocutePos == sliderMachineState.slidePosition.MEDIUM){

            currentlyScoring = true;
        }else{
            currentlyScoring = false;
        }



//Drive Code
        double drive = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;


       //distance sensor code

        double sensorLeftsetpoint = 5;
        double sensorRightsetpoint = 5;

       /*if(gamepad1.y){
            nearBoard = true;
        }else{
            nearBoard = false;
        }

        if(tsL.getDistance(DistanceUnit.INCH) < 20 && tsR.getDistance(DistanceUnit.INCH) < 20 && nearBoard) {

            double sensorLefterror = sensorLeftsetpoint - tsL.getDistance(DistanceUnit.INCH);
            double sensorRighterror = sensorRightsetpoint - tsR.getDistance(DistanceUnit.INCH);

            // Update the integral term (helps eliminate steady-state error)
            sensorLeftintegral += sensorLefterror;
            sensorRightintegral += sensorRighterror;

            // Calculate the derivative term (helps reduce overshooting)
            double sensorLeftderivative = sensorLefterror - sensorLeftprevError;
            double sensorRightderivative = sensorRighterror - sensorRightprevError;

            distanceOffsetLeft = sensorLeftkp * sensorLefterror + sensorLeftki * sensorLeftintegral + sensorLeftkd * sensorLeftderivative;
            distanceOffsetRight = sensorRightkp * sensorRighterror + sensorRightki * sensorRightintegral + sensorRigtkd * sensorRightderivative;

            // Update the previous error for the next iteration
            sensorLeftprevError = sensorLefterror;
            sensorRightprevError = sensorRighterror;

            // Apply the PID output to the robot's rotation
            // You may need to adjust the scaling factor to match your robot's behavior
            //distanceOffsetLeft = Range.clip(distanceOffsetLeft, -1.0, 1.0);
            //distanceOffsetRight = Range.clip(distanceOffsetRight, -1.0, 1.0);
        }else{
            distanceOffsetLeft = 0;
            distanceOffsetRight = 0;
        }*/


        //double rotate = -gamepad1.right_stick_x;

        // reset gyro button
        if (gamepad1.back) {
            gyroResetRequested = true;
        }

        //TODO: update gyro reset?

        // Perform gyro reset if requested
        if (gyroResetRequested) {
            IMUposeTransfercorrection = 0;
            resetGyro();
            gyroResetRequested = false; // Reset the request
        }

        // Get the robot's heading from the gyro sensor
        double heading = getPitch();
        // Calculate the joystick inputs in the field-oriented frame of reference
        double fieldDrive = drive * Math.cos(Math.toRadians(heading)) - strafe * Math.sin(Math.toRadians(heading));
        double fieldStrafe = drive * Math.sin(Math.toRadians(heading)) + strafe * Math.cos(Math.toRadians(heading));
        // Calculate motor powers for mecanum drive

        double frontLeftPower = fieldDrive + fieldStrafe + rotate;// + distanceOffsetLeft;
        double frontRightPower = fieldDrive - fieldStrafe - rotate;// + distanceOffsetRight;
        double rearLeftPower = fieldDrive - fieldStrafe + rotate;// + distanceOffsetLeft;
        double rearRightPower = fieldDrive + fieldStrafe - rotate;// + distanceOffsetRight;

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

        //imu pid
        // Calculate the error (how far off the robot is from the setpoint)
        double IMUerror = IMUsetpoint - heading;

        // Update the integral term (helps eliminate steady-state error)
        IMUintegral += IMUerror;

        // Calculate the derivative term (helps reduce overshooting)
        double IMUderivative = IMUerror - IMUprevError;

        // Calculate the PID output (rotate)
        // Check if the B button is pressed for rotating to 90 degrees
        /*if (gamepad1.y && !rotatingTo90) {
            IMUsetpoint = 90;
            rotatingTo90 = true;
            IMUintegral = 0; // Reset the integral term
        }*/

        if (rotatingTo90 && Math.abs(gamepad1.right_stick_x) < .2){
            // Calculate the PID output (rotate)
            rotate = IMUkp * IMUerror + IMUki * IMUintegral + IMUkd * IMUderivative;

            // Check if the robot is close enough to 90 degrees and stop
            if (Math.abs(IMUerror) < 5) {
                rotatingTo90 = false;
                rotate = 0;
            }
        }else if (Math.abs(gamepad1.right_stick_x) > .1){
            // Use right stick control when not rotating to 90 degrees
            rotate = -gamepad1.right_stick_x;
        }else {
            // Use right stick control when not rotating to 90 degrees
            rotate = -gamepad1.right_stick_x;
        }
        // Update the previous error for the next iteration
        IMUprevError = IMUerror;

        // Apply the PID output to the robot's rotation
        // You may need to adjust the scaling factor to match your robot's behavior
        rotate = Range.clip(rotate, -1.0, 1.0);

        //imu pid end


    }

    //This is used for field centric code but is outside of the loop
    private double getPitch() {
        // Get the robot's heading from the gyro sensor



/*
        double finalCorrectedHeading = imu.getAngularOrientation().firstAngle + IMUposeTransfercorrection;

        if (finalCorrectedHeading > 360){
            finalCorrectedHeading -= 360;
        }
        if (finalCorrectedHeading < 0){
            finalCorrectedHeading += 360;
        }
*/
        return (imu.getAngularOrientation().firstAngle);
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