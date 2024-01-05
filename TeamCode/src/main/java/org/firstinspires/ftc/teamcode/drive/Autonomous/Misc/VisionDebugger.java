package org.firstinspires.ftc.teamcode.drive.Autonomous.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCameraFactory;


@Autonomous
@Disabled
public class VisionDebugger extends OpMode {


    OpenCvWebcam webcam1 = null;
    double totalLeftAvg = 0;
    double totalRightAvg = 0;
    int frameCount = 0;


    @Override
    public void init(){

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }//TODO: adjust width and height baced on specific camera

            @Override
            public void onError(int errorCode) {

            }
        });


    }
    @Override
    public void loop() {

    }


    class examplePipeline extends OpenCvPipeline{

        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        double leftavgfin;
        double rightavgfin;
        Mat outPut = new Mat();

        Scalar rectColor = new Scalar(0.0, 0.0, 255.0);


        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
//1280,720
            Rect leftRect = new Rect(200, 100, 400, 500);
            Rect rightRect = new Rect(800, 100, 400, 500);//midile is 640

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 20);
            Imgproc.rectangle(outPut, rightRect, rectColor, 20);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);

            /*Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);//bad
*/
            Core.inRange(YCbCr, new Scalar(90, 90, 0), new Scalar(120, 255, 255), leftCrop);
            Core.inRange(YCbCr, new Scalar(90, 90, 0), new Scalar(120, 255, 255), rightCrop);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];

     /*       telemetry.addLine("pipeline running");
            telemetry.addData("LeftValue", leftavgfin);
            telemetry.addData("RightValue", rightavgfin);
*/
            //return outPut;

            if (leftavgfin > rightavgfin && (Math.abs(leftavgfin-rightavgfin)) >= 1.5){
                telemetry.addLine("A");
                telemetry.addData("LeftValue", leftavgfin);
                telemetry.addData("RightValue", rightavgfin);
            }
            else if (rightavgfin > leftavgfin &&  (Math.abs(leftavgfin-rightavgfin)) >= 1.5){
                telemetry.addLine("B");
                telemetry.addData("LeftValue", leftavgfin);
                telemetry.addData("RightValue", rightavgfin);
            }

            else{
                telemetry.addLine("C");
            }

            return outPut;
        }



    }
}
