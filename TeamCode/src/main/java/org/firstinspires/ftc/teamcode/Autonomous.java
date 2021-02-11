/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.io.instructions.FourRegisterDecodedInstruction;

import static org.firstinspires.ftc.teamcode.EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Linear Opmode")

public class Autonomous extends LinearOpMode {

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



    double power = .5;



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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            DriveLeft(power);
            sleep(600);

            NoDrive();
            sleep(100);

           DriveForward(power);
           sleep(2500);

           NoDrive();
           sleep(10);

            /*DriveLeft(power);
            sleep(600);

            NoDrive();
            sleep(100);

            DriveForward(power);
            sleep(2380);

            NoDrive();
            sleep(10);

            DriveRight(power);
            sleep(600);

            NoDrive();
            sleep(10);

            intake.setPower(1);
            shooter.setPower(-.85);

            sleep(5000);

            NoDrive();
            sleep(10);

            DriveForward(power);
            sleep(200);

            NoDrive();
            sleep(10);*/

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
        intake.setPower(0);
        shooter.setPower(0);
    }
}


