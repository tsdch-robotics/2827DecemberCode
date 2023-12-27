package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class sleep {

    public void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            // Handle the exception if needed
            e.printStackTrace();
        }
    }
}
