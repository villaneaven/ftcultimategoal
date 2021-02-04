

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Gamepad", group="Linear Opmode")
//@Disabled
public class Gamepad extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    DcMotor leftBack = null;
    DcMotor rightFront = null;
    DcMotor leftFront = null;
    DcMotor rightBack = null;
    DcMotor shooter = null;
    DcMotor intake = null;
    DcMotor armleft = null;
    DcMotor armright = null;

    Servo claw = null;

    double goalshootSpeed;
    double stickshootSpeed;
    double power = 1;
    double axisY;
    double axisX;
    double axisZ;
    double armaxisY;
    double intakeaxisY;
    double intakeaxisX;
    double leftVal;
    double rightVal;
    double sideVal;


    /*
        double right;
        double left;
        double slide;
    */


    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setDirection(DcMotor.Direction.FORWARD);

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        armleft = hardwareMap.get(DcMotor.class, "armleft");
        armleft.setDirection(DcMotor.Direction.REVERSE);


        armright = hardwareMap.get(DcMotor.class, "armright");
        armright.setDirection(DcMotor.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");





        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            //------Joystick Control------\\

            axisY = -gamepad1.left_stick_y; //slide forward and backwards
            axisZ = -gamepad1.left_stick_x; //slide left and right
            axisX = -gamepad1.right_stick_x; //tilt left and right


            leftVal = axisY + axisX;
            rightVal = axisY - axisX;
            sideVal = axisZ;



            leftFront.setPower(leftVal);
            leftFront.setPower(leftVal + sideVal);
            leftBack.setPower(leftVal);
            leftBack.setPower(leftVal - sideVal);
            rightFront.setPower(rightVal);
            rightFront.setPower(rightVal + sideVal);
            rightBack.setPower(rightVal);
            rightBack.setPower(rightVal - sideVal);


            armaxisY = -gamepad2.right_stick_y;

            armleft.setPower(.4*armaxisY);
            armright.setPower(-.4*armaxisY);

            //intakeaxisX = -gamepad1.right_trigger;
            intakeaxisY = gamepad1.right_trigger;

            intake.setPower(intakeaxisY);
            //intake.setPower(intakeaxisX);




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            goalshootSpeed = -.85;
            stickshootSpeed = -.67;

            if (gamepad2.right_bumper) {
                shooter.setPower(goalshootSpeed);
            }


            if (gamepad2.left_bumper){
                shooter.setPower(0);
            }

            if (gamepad2.a) {
                shooter.setPower(stickshootSpeed);
            }


            if (gamepad2.b){
                shooter.setPower(0);
            }


            if(gamepad1.a){
                intake.setPower(1);
            }

            if(gamepad1.y){
                intake.setPower(0);
            }


            if (gamepad2.dpad_left){//opens
                claw.setPosition(-.95);

            }

            if (gamepad2.dpad_right){//closes
                claw.setPosition(.95);
            }

            if (gamepad2.b){
                claw.setPosition(0);
            }






            telemetry.addData("Shoot Speed", shooter.getPower());


        }
    }
}





