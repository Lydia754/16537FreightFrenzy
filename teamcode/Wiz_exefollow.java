


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import net.frogbots.skystone.opmodes.util.AutoSuperclass2021;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

@Autonomous

public class Wiz_exefollow extends LinearOpMode {


        OpenCvCamera WebCam;
    Thing myThing;
    static int pos = 0;
    @Override
    public void runOpMode() {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            WebCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            myThing = new Thing();
            WebCam.setPipeline(myThing);

            WebCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    WebCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
                @Override
                public void onError(int errorCode) {

                }
            });

            //WebCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
            waitForStart();
            while (opModeIsActive()) {
                telemetry.addData("Analysis", myThing.getAnalysis());
                telemetry.addData("Position", myThing.position);
                telemetry.update();
                sleep(2000);
         // if ...
            }
        }

    public static class Thing extends OpenCvPipeline {
        public enum CapPosition {
            A,
            B,
            C
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Point TopLeftPoint = new Point(0, 115);
        static final Point TopLeftPoint2 = new Point(50, 115);
        static final Point TopLeftPoint3 = new Point(100, 115);
        static final int Region_width = 30;
        static final int Region_height = 25;
        Point region2_pointA = new Point(TopLeftPoint2.x, TopLeftPoint2.y);
        Point region2_pointB = new Point(TopLeftPoint2.x + Region_width, TopLeftPoint2.y + Region_height);
        Point region1_pointA = new Point(TopLeftPoint.x, TopLeftPoint.y);
        Point region1_pointB = new Point(TopLeftPoint.x + Region_width, TopLeftPoint.y + Region_height);
        Mat region1_Cb;
        Mat region2_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;
        int avg2;
        int iscap = 50;
        private volatile CapPosition position = CapPosition.A;
        void inputToCb(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb,Cb,1);
        }
        @Override
        public void init (Mat firstFrame){
           inputToCb(firstFrame);
            region1_Cb= Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb= Cb.submat(new Rect(region2_pointA, region2_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            avg1 = (int)Core.mean(region1_Cb).val[0];
            avg2 = (int)Core.mean(region2_Cb).val[0];


            Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);
            Imgproc.rectangle(input, region2_pointA, region2_pointB, BLUE, 2);


            position = CapPosition.A;
            if (avg1 > iscap){
                position = CapPosition.A;
            }
            else if (avg2 > iscap){
                position = CapPosition.B;
            }
            else{
            position = CapPosition.C;
            }
            Imgproc.rectangle(input,region1_pointA,region1_pointB, GREEN, -1);
            Imgproc.rectangle(input,region2_pointA,region2_pointB, GREEN, -1);
            return input;
        }
        public int getAnalysis(){
            return avg1;
        }

    }

}


