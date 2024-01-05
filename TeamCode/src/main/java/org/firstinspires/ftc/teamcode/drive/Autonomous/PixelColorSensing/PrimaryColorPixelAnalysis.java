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
public class PrimaryColorPixelAnalysis extends OpMode {

    OpenCvWebcam webcam = null;

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

        Mat input = new Mat();
        Mat output = new Mat();

        /*Scalar redLower = new Scalar(0, 0, 100);
       * Scalar redUpper = new Scalar(50, 50, 255);
       * Scalar yellowLower = new Scalar(20, 100, 100);
       * Scalar yellowUpper = new Scalar(30, 255, 255);
       * Scalar blueLower = new Scalar(100, 0, 0);
       * Scalar blueUpper = new Scalar(255, 50, 50);
         */

        Scalar redLower = new Scalar(0, 50, 50);       // Lower bound for red
        Scalar redUpper = new Scalar(20, 255, 255);    // Upper bound for red
        Scalar yellowLower = new Scalar(20, 50, 50);   // Lower bound for yellow
        Scalar yellowUpper = new Scalar(40, 255, 255); // Upper bound for yellow
        Scalar blueLower = new Scalar(90, 50, 50);     // Lower bound for blue
        Scalar blueUpper = new Scalar(130, 255, 255);  // Upper bound for blue


        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(this.input);
            Imgproc.cvtColor(this.input, this.input, Imgproc.COLOR_RGB2HSV); //multi color veiw!

           /// input.copyTo(this.input);
            // Display the original image for debugging
            //this.input.copyTo(this.output);//normal

            // Define a rectangle for color analysis
            Rect analysisRect = new Rect(100, 100, 500, 300);

            // Extract the region of interest
            Mat roi = this.input.submat(analysisRect);

            // Count the number of pixels in each color range
            int redPixels = countPixels(roi, redLower, redUpper);
            int yellowPixels = countPixels(roi, yellowLower, yellowUpper);
            int bluePixels = countPixels(roi, blueLower, blueUpper);

            // Output telemetry
            telemetry.addData("Red Pixels", redPixels);
            telemetry.addData("Yellow Pixels", yellowPixels);
            telemetry.addData("Blue Pixels", bluePixels);




            // Draw rectangle on the output frame for visualization
            Imgproc.rectangle(this.input, analysisRect, new Scalar(255, 255, 255), 2);

            return this.input;
        }

        private int countPixels(Mat image, Scalar lowerBound, Scalar upperBound) {
            Mat mask = new Mat();
            Core.inRange(image, lowerBound, upperBound, mask);
            return Core.countNonZero(mask);
        }
    }
}
