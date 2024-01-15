package org.firstinspires.ftc.teamcode.drive.Autonomous.PixelColorSensing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class OldPixelBlue extends OpMode {



    //CVstuff




    OpenCvWebcam webcam = null;

    public boolean greaterThanTargetPercentBluePixels1 = false;
    public boolean greaterThanTargetPercentBluePixels2 = false;

    public double targetPixPercent1 = 0.5;
    public double targetPixPercent2 = 0.5;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(new ColorAnalysisPipeline());

        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle error if needed
            }
        });
    }

    @Override
    public void loop() {
        // Your logic in the main loop (if needed)
    }

    class ColorAnalysisPipeline extends OpenCvPipeline {


        Mat roi1 = new Mat();
        Mat roi2 = new Mat();


        public boolean firstTime = true;

        Mat blueMask = new Mat();

        Mat inputMat = new Mat();
        Mat output = new Mat();

        Scalar blueLower = new Scalar(90, 50, 50);     // Lower bound for blue
        Scalar blueUpper = new Scalar(130, 255, 255);  // Upper bound for blue


        Rect rect1 = new Rect(50, 150, 250, 300);
        Rect rect2 = new Rect(350, 150, 250, 300);


        @Override
        public Mat processFrame(Mat input) {


            output.release();
            inputMat.release();


            //firstTime = false;

//clear mat

            input.copyTo(this.inputMat);

            // Display the original image for debugging
            this.inputMat.copyTo(this.output);// include?

            // Define two rectangles for blue pixel analysis


            // Extract the regions of interest
            roi1 = this.inputMat.submat(rect1);
            roi2 = this.inputMat.submat(rect2);


            // Count the number of blue pixels in each rectangle
            int bluePixels1 = countBluePixels(roi1);
            int bluePixels2 = countBluePixels(roi2);



            // Output telemetry
            telemetry.addData("Blue Pixels in Rectangle 1", bluePixels1);
            telemetry.addData("Blue Pixels in Rectangle 2", bluePixels2);
            telemetry.addData("Rectangle1 Area", rect1.area());
            telemetry.addData("Rectangle2 Area", rect2.area());


            //Compute target percents and determen if met
            if(bluePixels1/rect1.area() > targetPixPercent1){
                greaterThanTargetPercentBluePixels1 = true;
            }else if(bluePixels2/rect2.area() > targetPixPercent2){
                greaterThanTargetPercentBluePixels2 = true;
            }
            else{
                greaterThanTargetPercentBluePixels1 = false;
                greaterThanTargetPercentBluePixels2 = false;
            }

            if(bluePixels1 > bluePixels2 && greaterThanTargetPercentBluePixels1){
                telemetry.addLine("Middle");
            }else if(bluePixels1 < bluePixels2 && greaterThanTargetPercentBluePixels2){
                telemetry.addLine("Right");
            }else{
                telemetry.addLine("Left");
            }



            // Draw rectangles on the output frame for visualization
            Imgproc.rectangle(this.output, rect1, new Scalar(255, 0, 0), 2); // Blue for rectangle 1
            Imgproc.rectangle(this.output, rect2, new Scalar(255, 0, 0), 2); // Blue for rectangle 2


            //Clear the mat after it has bean used



                //inputMat.release();
            roi1.release();
            roi2.release();


           // return this.inputMat;//was output
            return this.output;//was output


            //Clear the mat after it has bean used





        }

        private int countBluePixels(Mat image) {
            // Convert the image to HSV color space

            //blueMask.release();


            Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2HSV);

            // Define the blue color range
            //Scalar blueLower = new Scalar(90, 50, 50);
            //Scalar blueUpper = new Scalar(130, 255, 255);

            // Create a mask for blue pixels

            //Mat blueMask = new Mat();
            Core.inRange(image, blueLower, blueUpper, blueMask);

            // Count the number of blue pixels
            return Core.countNonZero(blueMask);
        }
    }
}