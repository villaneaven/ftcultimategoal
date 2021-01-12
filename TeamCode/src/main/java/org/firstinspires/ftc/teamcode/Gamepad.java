

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

    double shootSpeed;
    double power = 1;
    double axisY;
    double axisX;
    double axisZ;
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



        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            //------Joystick Control------\\

            axisY = gamepad1.left_stick_y; //slide forward and backwards
            axisZ = gamepad1.left_stick_x; //slide left and right
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
/*
            right = gamepad1.right_stick_y;
            left = gamepad1.left_stick_y;
            slide = -gamepad1.left_stick_x;


*/
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            shootSpeed = -.74;




            if (gamepad1.a) {
                shootSpeed = shootSpeed + .10;
                shooter.setPower(shootSpeed);
            }

            if (gamepad1.y){
                shootSpeed = shootSpeed - .10;
                shooter.setPower(shootSpeed);
            }

            if (gamepad1.b){
                shooter.setPower(0);
            }




            telemetry.addData("Shoot Speed", shooter.getPower());


        }
    }
}





