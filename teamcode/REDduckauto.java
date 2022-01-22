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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class REDduckauto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    // Declare OpMode members.
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor arm = null;
    private DcMotor intake;
    private int DpadPos = 0;
    private double movespeed = 1;
    private double turnspeed = 0.75;
    private  int Rightpressed = 0;
    private Servo servoliftNeg;
    private Servo servoliftPos;
    private Servo mailbox;
    private Servo OL;
    private Servo OR;
    private Servo OS;
    private DcMotor ducks;
    private double speed ;


    @Override
public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        OR = hardwareMap.get(Servo.class, "OR");
        OL = hardwareMap.get(Servo.class, "OL");
        OS = hardwareMap.get(Servo.class, "OS");
        OL.setPosition(0);
        OR.setPosition(1);
        OS.setPosition(0.8);

        servoliftNeg = hardwareMap.get(Servo.class, "Sneg");
        servoliftPos = hardwareMap.get(Servo.class, "Spos");

        mailbox = hardwareMap.get(Servo.class, "mailbox");
        servoliftNeg.setPosition(1);
        servoliftPos.setPosition(0);
        //0.85
        //servoliftNeg.setPosition(0.8);
        // servoliftPos.setPosition(0.2);
        //0.15`
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        intake = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");
        ducks = hardwareMap.get(DcMotor.class, "ducks");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ducks.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized");


    double LBPower;
    double LFPower;
    double RBPower;
    double RFPower;
    double YAxis;
    double XAxis;

    waitForStart();
    gobackward(.5, 200);
    straferight(0.5, 1000);
    //duck spin stop
    duckspin(200000);
    //goforward(0.5, 3000);
    telemetry.addData("Path", "Complete");
    telemetry.update();
    sleep(1000);

}

    private void VFBup() {
        servoliftNeg.setPosition(0.8);
        servoliftPos.setPosition(0.2);
        mailbox.setPosition(0.6);
    }
    private void duckspin (long time){
        ducks.setPower(-0.3);
    }
    private void armup() {
    arm.setTargetPosition(500);
           if (arm.getCurrentPosition() >= 500) {
        arm.setPower(0);
    }
           arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           arm.setPower(0.4);}
    private void dump(){
        mailbox.setPosition(0);
    }
    private void armdown(){
        arm.setTargetPosition(0);
        if (arm.getCurrentPosition() <= 0) {
            arm.setPower(0);
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);
    }
    private void VFBdown(){
        servoliftNeg.setPosition(1);
        servoliftPos.setPosition(0);

    }
    private void gostop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    private void goforward(double speed, long time) {
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
        sleep(time);
        gostop();
    }
    private void gobackward(double speed, long time) {
        leftFront.setPower(speed * -1);
        leftBack.setPower(speed * -1);
        rightFront.setPower(speed * -1);
        rightBack.setPower(speed * -1);
        sleep(time);
        gostop();
    }
    private void strafeleft(double speed, long time) {
        leftFront.setPower(speed * -1);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed * -1);
        sleep(time);
        gostop();
    }
    private void straferight(double speed,long time) {
        leftFront.setPower(speed);
        leftBack.setPower(speed * -1);
        rightFront.setPower(speed * -1);
        rightBack.setPower(speed);
        sleep(time);
        gostop();
    }
    private void turnleft(double speed,long time) {
        leftFront.setPower(speed * -1);
        leftBack.setPower(speed * -1);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
        sleep(time);
        gostop();
    }
    private void turnright(double speed,long time) {
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed * -1);
        rightBack.setPower(speed * -1);
        sleep(time);
        gostop();
    }
    private void godiagonalNE(double speed,long time) {
        leftFront.setPower(speed);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(speed);
        sleep(time);
        gostop();
    }
    private void godiagonalSE(double speed,long time) {
        leftFront.setPower(0);
        leftBack.setPower(speed * -1);
        rightFront.setPower(speed * -1);
        rightBack.setPower(0);
        sleep(time);
        gostop();
    }
    private void godiagonalSW(double speed,long time) {
        leftFront.setPower(speed * -1);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(speed * -1);
        sleep(time);
        gostop();
    }
    private void godiagonalNW(double speed,long time) {
        leftFront.setPower(0);
        leftBack.setPower(speed );
        rightFront.setPower(speed );
        rightBack.setPower(0);
        sleep(time);
        gostop();
    }


}
