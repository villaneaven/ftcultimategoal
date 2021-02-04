package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */
@Autonomous
public class EasyOpenCVExample extends LinearOpMode
{
    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor armleft;
    DcMotor armright;

    DcMotor shooter;
    DcMotor intake;

    Servo claw;
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
    double power = .5;
    boolean A = false;


    @Override
    public void runOpMode()
    {

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armleft = hardwareMap.get(DcMotor.class, "armleft");
        armleft.setDirection(DcMotor.Direction.REVERSE);


        armright = hardwareMap.get(DcMotor.class, "armright");
        armright.setDirection(DcMotor.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });



        waitForStart();

        while (opModeIsActive()) {

            int amountofrings = 5;

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR && A == false) {

                amountofrings = 4;
                A = true;

                if (amountofrings == 4 && A == true){

                    DriveLeft(power);
                    sleep(600);

                    NoDrive();
                    sleep(100);

                    DriveForward(power);
                    sleep(3950);

                    NoDrive();
                    sleep(100);

                    /*moved up to half way

                    armleft.setPower(.45);
                    armright.setPower(-.45);
                    sleep(500);

                    NoDrive();
                    sleep(100);

                    //moved wobble out of the way

                    intake.setPower(1);
                    shooter.setPower(85);
                    sleep(3000);

                    NoDrive();
                    sleep(100);

                    //rings have been shot

                    DriveForward(power);
                    sleep(1500);

                    NoDrive();
                    sleep(100);


                     */

                    DriveRight(power);
                    sleep(1900);

                    NoDrive();
                    sleep(100);

                    //up to the third box

                    tiltLeft(power);
                    sleep(1900);

                    NoDrive();
                    sleep(100);

                    //back facing third box

                    armleft.setPower(.45);
                    armright.setPower(-.45);
                    sleep(350);

                    NoDrive();
                    sleep(10);

                    claw.setPosition(-.95);
                    sleep(1000);

                    //arm is raised and wobble has been dropped

                    DriveForward(power);
                    sleep(1500);

                    NoDrive();
                    sleep(100);

                    //robot is parked

                }


            }

            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE && A == false) {
                amountofrings = 1;
                A = true;
                if (amountofrings == 1 && A == true){

                    DriveLeft(power);
                    sleep(600);

                    NoDrive();
                    sleep(100);

                    DriveForward(power);
                    sleep(3280);

                    NoDrive();
                    sleep(100);

                    //moved up half way

                    DriveRight(power);
                    sleep(500);

                    NoDrive();
                    sleep(100);

                    //up to the second box

                    tiltLeft(power);
                    sleep(1900);

                    NoDrive();
                    sleep(100);

                    //back facing third box

                    armleft.setPower(.45);
                    armright.setPower(-.45);
                    sleep(350);

                    NoDrive();
                    sleep(10);

                    claw.setPosition(-.95);
                    sleep(1000);

                    //arm is raised and wobble has been dropped

                    DriveForward(power);
                    sleep(300);

                    NoDrive();
                    sleep(100);

                    //robot is parked
                }

            }

            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE && A == false) {
                amountofrings = 0;
                A = true;

                if (amountofrings == 0 && A == true){

                    //shooter.setPower(1);

                    DriveLeft(power);
                    sleep(600);

                    NoDrive();
                    sleep(100);

                    DriveForward(power);
                    sleep(2635);//2500

                    NoDrive();
                    sleep(10);

                    DriveRight(power);
                    sleep(600);

                    NoDrive();
                    sleep(100);

                    DriveForward(power);
                    sleep(300);

                    NoDrive();
                    sleep(100);



                    /*DriveLeft(power);
                    sleep(600);

                    NoDrive();
                    sleep(100);

                    DriveForward(power);
                    sleep(2600);

                    NoDrive();
                    sleep(100);

                    //moved up half way

                    DriveRight(power);
                    sleep(1900);

                    NoDrive();
                    sleep(100);

                    //up to the second box

                    tiltLeft(power);
                    sleep(1900);

                    NoDrive();
                    sleep(100);

                    //back facing first box

                    armleft.setPower(.45);
                    armright.setPower(-.45);
                    sleep(350);

                    NoDrive();
                    sleep(10);

                    claw.setPosition(-.95);
                    sleep(1000);

                    //arm is raised and wobble has been dropped


                    NoDrive();
                    sleep(100);

                    //robot is parked*/
                }
            }


        }
    }
    public void DriveForward (double power){

        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        rightBack.setPower(-power);


    }

    public void DriveLeft(double power){

        leftFront.setPower(-power);
        leftBack.setPower(power);
        rightFront.setPower(-power);
        rightBack.setPower(power);
    }

    public void DriveBack(double power){

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    public void DriveRight(double power){

        leftFront.setPower(power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(-power);
    }

    public void tiltLeft(double power){

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(-power);
        rightBack.setPower(-power);
    }
    public void tiltRight(double power){

        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }


    public void NoDrive (){

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        armright.setPower(0);
        armleft.setPower(0);

    }

    public void shootStuff(){
        intake.setPower(0);
        shooter.setPower(0);
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;


        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {

            inputToCb(firstFrame);


            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));

        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);


            avg1 = (int) Core.mean(region1_Cb).val[0];



            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR;
            if (avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else {
                position = RingPosition.NONE;
            }


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            return input;
        }



        public int getAnalysis()
        {
            return avg1;
        }



    }
}

