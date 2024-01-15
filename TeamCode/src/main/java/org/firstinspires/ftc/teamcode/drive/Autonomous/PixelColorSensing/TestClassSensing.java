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
public class TestClassSensing extends OpMode {

    OpenCvWebcam webcam = null;
    ColorAnalysisPipeline colorAnalysisPipeline;

    @Override
    public void init() {
        colorAnalysisPipeline = new ColorAnalysisPipeline();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(colorAnalysisPipeline);

        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
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

        Mat hsvImage;
        Mat analysisRegion;

        Scalar redLower = new Scalar(0, 50, 50);
        Scalar redUpper = new Scalar(20, 255, 255);
        Scalar yellowLower = new Scalar(20, 50, 50);
        Scalar yellowUpper = new Scalar(40, 255, 255);
        Scalar blueLower = new Scalar(90, 50, 50);
        Scalar blueUpper = new Scalar(130, 255, 255);

        ColorAnalysisPipeline() {
            hsvImage = new Mat();
            analysisRegion = new Mat();
        }

        @Override
        public Mat processFrame(Mat input) {
            hsvImage.release(); // Release the previous hsvImage Mat
            hsvImage = input.clone(); // Clone the input Mat

            Imgproc.cvtColor(hsvImage, hsvImage, Imgproc.COLOR_RGB2HSV);

            Rect analysisRect = new Rect(100, 100, 500, 300);
            analysisRegion.release(); // Release the previous analysisRegion Mat
            analysisRegion = hsvImage.submat(analysisRect);

            int redPixels = countPixels(analysisRegion, redLower, redUpper);
            int yellowPixels = countPixels(analysisRegion, yellowLower, yellowUpper);
            int bluePixels = countPixels(analysisRegion, blueLower, blueUpper);

            telemetry.addData("Red Pixels", redPixels);
            telemetry.addData("Yellow Pixels", yellowPixels);
            telemetry.addData("Blue Pixels", bluePixels);

            Imgproc.rectangle(input, analysisRect, new Scalar(255, 255, 255), 2);

            return input;
        }

        private int countPixels(Mat image, Scalar lowerBound, Scalar upperBound) {
            Mat mask = new Mat();
            Core.inRange(image, lowerBound, upperBound, mask);
            return Core.countNonZero(mask);
        }
    }
}
