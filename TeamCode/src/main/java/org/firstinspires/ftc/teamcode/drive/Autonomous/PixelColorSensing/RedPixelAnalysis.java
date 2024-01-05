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
public class RedPixelAnalysis extends OpMode {

    OpenCvWebcam webcam = null;

    public boolean greaterThanTargetPercentRedPixels1 = false;
    public boolean greaterThanTargetPercentRedPixels2 = false;

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

        Mat input = new Mat();
        Mat output = new Mat();

        Scalar redLower = new Scalar(0, 50, 100);       // Lower bound for red
        Scalar redUpper = new Scalar(10, 255, 255);    // Upper bound for red

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(this.input);

            // Display the original image for debugging
            this.input.copyTo(this.output);

            // Define two rectangles for red pixel analysis
            Rect rect1 = new Rect(50, 150, 250, 300);
            Rect rect2 = new Rect(350, 150, 250, 300);

            // Extract the regions of interest
            Mat roi1 = this.input.submat(rect1);
            Mat roi2 = this.input.submat(rect2);

            // Count the number of red pixels in each rectangle
            int redPixels1 = countRedPixels(roi1);
            int redPixels2 = countRedPixels(roi2);

            // Output telemetry
            telemetry.addData("Red Pixels in Rectangle 1", redPixels1);
            telemetry.addData("Red Pixels in Rectangle 2", redPixels2);
            telemetry.addData("Rectangle1 Area", rect1.area());
            telemetry.addData("Rectangle2 Area", rect2.area());

            // Compute target percents and determine if met
            if (redPixels1 / rect1.area() > targetPixPercent1) {
                greaterThanTargetPercentRedPixels1 = true;
            } else if (redPixels2 / rect2.area() > targetPixPercent2) {
                greaterThanTargetPercentRedPixels2 = true;
            } else {
                greaterThanTargetPercentRedPixels1 = false;
                greaterThanTargetPercentRedPixels2 = false;
            }

            if (redPixels1 > redPixels2 && greaterThanTargetPercentRedPixels1) {
                telemetry.addLine("Middle");
            } else if (redPixels1 < redPixels2 && greaterThanTargetPercentRedPixels2) {
                telemetry.addLine("Right");
            } else {
                telemetry.addLine("Left");
            }

            // Draw rectangles on the output frame for visualization
            Imgproc.rectangle(this.output, rect1, new Scalar(0, 0, 255), 2); // Red for rectangle 1
            Imgproc.rectangle(this.output, rect2, new Scalar(0, 0, 255), 2); // Red for rectangle 2

            return this.output;
        }

        private int countRedPixels(Mat image) {
            // Convert the image to HSV color space
            Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2HSV);

            // Create a mask for red pixels
            Mat redMask = new Mat();
            Core.inRange(image, redLower, redUpper, redMask);

            // Count the number of red pixels
            return Core.countNonZero(redMask);
        }
    }
}
