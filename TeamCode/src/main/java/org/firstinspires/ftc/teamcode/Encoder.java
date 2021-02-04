package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@Autonomous(name="Encoder", group="Linear Opmode")
//@Disabled
public class Encoder extends LinearOpMode {

    DcMotor leftBack = null;
    DcMotor rightFront = null;
    DcMotor leftFront = null;
    DcMotor rightBack = null;
    DcMotor shooter = null;
    DcMotor intake = null;
    DcMotor armleft = null;
    DcMotor armright = null;

    Servo claw = null;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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


        waitForStart();

        double GOBILDA_TICKS_PER_REV = 103.6;
        double power = .5;

        DriveForwardDistance(power, 1000);




    }

    public void DriveForwardDistance(double power, int distance){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(-distance);
        leftBack.setTargetPosition(-distance);
        rightFront.setTargetPosition(-distance);
        rightBack.setTargetPosition(-distance);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){

            //wait for target position is reached

        }

        NoDrive();
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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



}


