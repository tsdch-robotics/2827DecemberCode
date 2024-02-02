package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.moveWithBasicEncoder;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.Halt;
import org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions.sleep;

@Config
public class sliderMachineState {

    //moveWithBasicEncoder moveByEncoder = new moveWithBasicEncoder();
    PIDclass slidesPID = new PIDclass();
    Halt halt = new Halt();


    public static int rowHeightVal = 300;

    public static int zeroToBaseOffset = 600;


    public int finalHeight;

    sleep sleep = new sleep();
   public enum slidePosition {

        ROWSCORING,
        MANUAL,
        THREATEN, //waiting to stab
        STAB, //stabing
        RESTAB,
        STABAFTERSTAB,
        LOW, //500
        MEDIUM,//1500
        HIGH,//2500
        REALLYHIGH;
    }

    public static int LOWpos = 800;
    public static int THREATENINGpos = 10;
    public static int midSTABpos = 800;
    public static int STABBINGpos = -200;
    public static int MEDIUMpos = 1300;
    public static int HIGHpos = 1800;
    public static int ReallyHIGHpos = 3000;
    //todo adjust these values fortnite

    public static double stabFinger1Tight = 0.82;//servo pos
    public static double stabFinger2Tight = .58;//servo pos

    public static double Finger1Loose = 0.5;//servo pos
    public static double Finger2Loose = 0.51;//servo pos

    public static double wristThreaten = .6;//servo pos
    public static double wristStab = 0.44;//servo pos
    public static double wristScore = 0.3;//servo pos

    public static double armThreaten = 0.2;//servo pos
    public static double armStab = 0.05;//servo pos
    public static double armScore = .99;//servo pos




    public void magicalMacro (DcMotor slider, Servo arm1, Servo arm2,
                              Servo wrist, Servo stabberLeft,
                              Servo stabberRight, slidePosition targetMachineState, ElapsedTime HaltTime, ElapsedTime PIDtime, TouchSensor toucher, boolean auto, int rowHeight){


        if (HaltTime == null || PIDtime == null) {
            // Handle the case where time is null (log an error, throw an exception, or any other appropriate action)
            return;
        }

        if (!auto){
            switch (targetMachineState) {
                case THREATEN:

                    stabberLeft.setPosition(Finger1Loose);
                    stabberRight.setPosition(Finger2Loose);

                    slidesPID.zero(slider, HaltTime, toucher);
                    //slidesPID.magicPID(slider, THREATENINGpos, PIDtime);

                    wrist.setPosition(wristThreaten);
                    arm1.setPosition(armThreaten);
                    arm2.setPosition(armThreaten);//need to make servos go the same direction

                    break;


                case RESTAB:

                    stabberLeft.setPosition(Finger1Loose);
                    stabberRight.setPosition(Finger2Loose);

                    if(halt.halt(300, HaltTime)){


                        if(slider.getCurrentPosition() < 600){
                            slider.setPower(1);
                        }else{
                            slider.setPower(0);
                        }

                        wrist.setPosition(wristStab);
                        arm1.setPosition(armStab);
                        arm2.setPosition(armStab);//need to make servos go the same direction

                    }

                    break;

                case STABAFTERSTAB:

                    slidesPID.zero(slider, PIDtime, toucher);

                    if(halt.halt(200, HaltTime)){
                        stabberLeft.setPosition(stabFinger1Tight);
                        stabberRight.setPosition(stabFinger2Tight);


                    }




                    break;


                case STAB:

                    if (!halt.halt(400, HaltTime)){

                        //slidesPID.magicPID(slider, midSTABpos, PIDtime);
                        /*if(slider.getCurrentPosition() < 500){
                            slider.setPower(.5);
                        }else{
                            slider.setPower(0);
                        }*/

                        slidesPID.magicPID(slider, midSTABpos, PIDtime);

                        wrist.setPosition(wristStab);
                        arm1.setPosition(armStab);
                        arm2.setPosition(armStab);
                    }


                    if (halt.halt(400, HaltTime)) {//so really this is only halting by this - the previous halt

                        slidesPID.zero(slider, PIDtime, toucher);

                        if (halt.halt(600, HaltTime)){

                            slidesPID.zero(slider, PIDtime, toucher);

                            stabberLeft.setPosition(stabFinger1Tight);
                            stabberRight.setPosition(stabFinger2Tight);
                        }

                    }


                    break;
                case LOW:


                    finalHeight = LOWpos;

                    slidesPID.magicPID(slider, finalHeight, PIDtime);



                    arm1.setPosition(armScore);
                    arm2.setPosition(armScore);
                    if (halt.halt(0, HaltTime)) {
                        wrist.setPosition(wristScore);
                    }

                    break;
                case MEDIUM:

                    finalHeight = MEDIUMpos;

                    slidesPID.magicPID(slider, finalHeight, PIDtime);



                    arm1.setPosition(armScore);
                    arm2.setPosition(armScore);
                    if (halt.halt(0, HaltTime)) {
                        wrist.setPosition(wristScore);
                    }

                    break;
                case HIGH:


                    finalHeight = HIGHpos;

                    slidesPID.magicPID(slider, finalHeight, PIDtime);

                    arm1.setPosition(armScore);
                    arm2.setPosition(armScore);
                    if (halt.halt(0, HaltTime)) {
                        wrist.setPosition(wristScore);
                    }
                    break;
                case REALLYHIGH:


                    finalHeight = ReallyHIGHpos;

                    slidesPID.magicPID(slider, finalHeight, PIDtime);

                    arm1.setPosition(armScore);
                    arm2.setPosition(armScore);
                    if (halt.halt(0, HaltTime)) {
                        wrist.setPosition(wristScore);
                    }
                    break;

                case MANUAL:

                    break;

                case ROWSCORING:

                    finalHeight = (rowHeight * rowHeightVal) + zeroToBaseOffset ;

                    slidesPID.magicPID(slider, finalHeight, PIDtime);

                    arm1.setPosition(armScore);
                    arm2.setPosition(armScore);

                    wrist.setPosition(wristScore);



                    break;




            }
        }else{
            /*switch (targetMachineState) {
                case THREATEN:

                    stabberLeft.setPosition(Finger1Loose);
                    stabberRight.setPosition(Finger2Loose);


                    slidesPID.magicPID(slider, THREATENINGpos, PIDtime);


                    wrist.setPosition(wristThreaten);
                    arm1.setPosition(armThreaten);
                    arm2.setPosition(armThreaten);//need to make servos go the same direction

                    break;


                case STAB:


                    slidesPID.magicPID(slider, midSTABpos, PIDtime);


                    sleep.sleep(1000);

                    wrist.setPosition(wristStab);//unfortunalty this will use combine time so a second step has to acound for the time that has already been taken
                    arm1.setPosition(armStab);
                    arm2.setPosition(armStab);
                    sleep.sleep(1000);


                    slidesPID.magicPID(slider, STABBINGpos, PIDtime);

                    sleep.sleep(1000);
                    stabberLeft.setPosition(stabFinger1Tight);
                    stabberRight.setPosition(stabFinger2Tight);
                    sleep.sleep(1000);

                    break;


                case LOW:


                    slidesPID.magicPID(slider, LOWpos, PIDtime);

                    arm1.setPosition(armScore);
                    arm2.setPosition(armScore);
                    sleep.sleep(800);
                        wrist.setPosition(wristScore);


                    break;
                case MEDIUM:

                    slidesPID.magicPID(slider, MEDIUMpos, PIDtime);

                    arm1.setPosition(armScore);
                    arm2.setPosition(armScore);
                    sleep.sleep(1000);
                    wrist.setPosition(wristScore);


                    break;
                case HIGH:

                    slidesPID.magicPID(slider, HIGHpos, PIDtime);

                    arm1.setPosition(armScore);
                    arm2.setPosition(armScore);
                    sleep.sleep(1500);
                    wrist.setPosition(wristScore);

                    break;


                case STABAFTERSTAB:

                    slidesPID.magicPID(slider, STABBINGpos - 10, PIDtime);


                    sleep.sleep(300);
                    stabberLeft.setPosition(stabFinger1Tight);
                    stabberRight.setPosition(stabFinger2Tight);


                    break;

            }*/
        }






    }



}
