

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

    double shootSpeed;
    double power = 1;
    double axisY;
    double axisX;
    double axisZ;
    double armaxisY;
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
            axisX = gamepad1.right_stick_x; //tilt left and right


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


            armaxisY = -gamepad2.left_stick_y;

            armleft.setPower(armaxisY);
            armright.setPower(-armaxisY);




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            shootSpeed = -.85;

            if (gamepad1.right_bumper) {
                shootSpeed = shootSpeed + .10;
                shooter.setPower(shootSpeed);
            }

            /*if (gamepad1.y){
                shootSpeed = shootSpeed - .10;
                shooter.setPower(shootSpeed);
            }*/


            if (gamepad1.left_bumper){
                shooter.setPower(0);
            }

            if(gamepad1.a){
                intake.setPower(1);
            }

            if(gamepad1.y){
                intake.setPower(0);
            }



            if (gamepad1.dpad_up){
                armleft.setPower(.5);
                armright.setPower(-.5);

            }else {
                armleft.setPower(0);
            }
            if (gamepad1.dpad_down){
                armleft.setPower(-.5);
                armright.setPower(.5);
            }else {
                armright.setPower(0);
            }



            if (gamepad1.dpad_left){
                claw.setPosition(.5);
            }

            if (gamepad1.dpad_right){
                claw.setPosition(-.5);
            }

            if (gamepad1.b){
                claw.setPosition(0);
            }




            telemetry.addData("Shoot Speed", shooter.getPower());


        }
    }
}





