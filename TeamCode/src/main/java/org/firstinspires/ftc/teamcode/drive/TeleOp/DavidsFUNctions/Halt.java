package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import com.qualcomm.robotcore.util.ElapsedTime;



public class Halt {


    public boolean halt(int millis, ElapsedTime time) {

        if (time == null) {
            // Handle the case where time is null (log an error, throw an exception, or any other appropriate action)
            return false;
        }else {
           /* if(startingHalt) {
                time.reset();
                startingHalt = false;
            }*/
            // Perform background tasks here while waiting
            if (time.milliseconds() < millis) {
                return false;
            } else {
                //time.reset();
                return true;
            }//now the time will just reset when you hit the execute slides button
        }
    }
}
