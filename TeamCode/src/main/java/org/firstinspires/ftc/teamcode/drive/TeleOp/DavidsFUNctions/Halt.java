package org.firstinspires.ftc.teamcode.drive.TeleOp.DavidsFUNctions;

public class Halt {

    public void halt(int seconds) {
        try {
            Thread.sleep(seconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
