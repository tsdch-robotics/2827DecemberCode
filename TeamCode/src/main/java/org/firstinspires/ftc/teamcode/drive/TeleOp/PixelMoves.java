
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


public class PixelMoves {

    public static double slideMaxPower = 1;//motor power
    public static double slideMinPower = 0.25;//motor power

    public static double slideHIGHPos = 2500;//tick position
    public static double slideMEDIUMPos = 1500;//tick position
    public static double slideLOWPos = 500;//tick position

    public static double slideZEROPos = 0;//tick position
    public static double slideNEGATIVEoffsetPos = -10;//tick position

    public static double slideTHREATENINGPos = 100;//tick position

    public static double slidePRESTABPos = 300;//tick position
    public static double slideSTABPos = 20;//tick position
    public static double stabberExpandPos = 5;//servo pos
    public static double stabberContractPos = 5;//servo pos
    public static double wristScoringLowPos = 5;//servo pos
    public static double wristScoringHighPos = 5;//servo pos
    public static double wristThreateningPos = 5;//servo pos
    public static double wristStabbingPos = 5;//servo pos
    public static double armScoringLowPos = 5;//servo pos
    public static double armScoringHighPos = 5;//servo pos








    public double sliderPowerOutput (DcMotor slider, int targetTick,){



//the goal is to return a slider power using PID, and a servo position
//
}