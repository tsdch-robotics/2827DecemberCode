package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import android.transition.Slide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.moveWithBasicEncoder;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.Halt;


public class sliderMachineState {

    moveWithBasicEncoder moveByEncoder = new moveWithBasicEncoder();
    Halt halt = new Halt();

   public enum slidePosition {

        THREATEN, //waiting to stab
        STAB, //stabing


        LOW, //500
        MEDIUM,//1500
        HIGH;//2500
    }

    public static final int LOWpos = 800;
    public static final int THREATENINGpos = 10;
    public static final int midSTABpos = 200;
    public static final int STABBINGpos = 0;
    public static final int MEDIUMpos = 1600;
    public static final int HIGHpos = 2800;
    //todo adjust these values

    public static final double stabFinger1Tight = 1;//servo pos
    public static final double stabFinger2Tight = .58;//servo pos

    public static final double Finger1Loose = 0.6;//servo pos
    public static final double Finger2Loose = 0.51;//servo pos

    public static final double wristThreaten = .75;//servo pos
    public static final double wristStab = 0.75;//servo pos
    public static final double wristScore = 0.3;//servo pos


    public static final double armThreaten = 0.17;//servo pos
    public static final double armStab = 0.06;//servo pos
    public static final double armScore = 0.85;//servo pos




    public void magicalMacro (DcMotor slider, Servo arm1, Servo arm2,
                              Servo wrist, Servo stabberLeft,
                              Servo stabberRight, slidePosition targetMachineState, ElapsedTime time){


        if (time == null) {
            // Handle the case where time is null (log an error, throw an exception, or any other appropriate action)
            return;
        }


        switch (targetMachineState) {
            case THREATEN:

                stabberLeft.setPosition(Finger1Loose);
                stabberRight.setPosition(Finger2Loose);
                moveByEncoder.powerSlider(slider, THREATENINGpos);
                wrist.setPosition(wristThreaten);
                arm1.setPosition(armThreaten);
                arm2.setPosition(armThreaten);//need to make servos go the same direction

                break;


            case STAB:

                if (!halt.halt(50, time)){
                    moveByEncoder.powerSlider(slider, midSTABpos);
                }
                if (halt.halt(50, time)) {
                    wrist.setPosition(wristStab);//unfortunalty this will use combine time so a second step has to acound for the time that has already been taken
                    arm1.setPosition(armStab);
                    arm2.setPosition(armStab);
                    if (halt.halt(400, time)) {//so really this is only halting by this - the previous halt
                        moveByEncoder.powerSlider(slider, STABBINGpos);
                        if (halt.halt(700, time) && slider.getCurrentPosition() <= (midSTABpos + 10)) {//so really this is only halting by this - the previous halt
                            stabberLeft.setPosition(stabFinger1Tight);
                            stabberRight.setPosition(stabFinger2Tight);
                        }
                    }
                }


                break;
            case LOW:


                moveByEncoder.powerSlider(slider, LOWpos);

                arm1.setPosition(armScore);
                arm2.setPosition(armScore);
                if (halt.halt(800, time)) {
                    wrist.setPosition(wristScore);
                }

                break;
            case MEDIUM:

                moveByEncoder.powerSlider(slider, MEDIUMpos);

                arm1.setPosition(armScore);
                arm2.setPosition(armScore);
                if (halt.halt(1000, time)) {
                    wrist.setPosition(wristScore);
                }

                break;
            case HIGH:

                moveByEncoder.powerSlider(slider, HIGHpos);

                arm1.setPosition(armScore);
                arm2.setPosition(armScore);
                if (halt.halt(1500, time)) {
                    wrist.setPosition(wristScore);
                }
                break;



        }



    }



}
