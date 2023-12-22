package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Halt {

    public void halt(int millis, ElapsedTime time) {
        time.reset();

        // Perform background tasks here while waiting
        while (time.milliseconds() < millis) {
            // Your background tasks go here
            // This loop will run until the elapsed time reaches the specified milliseconds
        }
    }
}
