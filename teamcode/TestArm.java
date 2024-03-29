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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
//@Disabled
public class TestArm extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm = null;
    private Servo servoliftPos;
    private Servo servoliftNeg;
    private double SP = 0;
    private int DpadPos = 0;
    public void raisearm(double speed, int EncoderTarget){

    }

    @Override
    public void init() {
        servoliftNeg = hardwareMap.get(Servo.class, "Sneg");
        //0.85
        servoliftPos = hardwareMap.get(Servo.class, "Spos");
        //0.15
        telemetry.addData("Status", "Initialized");
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setTargetPosition(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized");
        servoliftNeg.setPosition(0.8);
        servoliftPos.setPosition(0.2);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

       if(gamepad1.dpad_down) {
           if (DpadPos != 1) {
               raisearm(0.5, 50);
               DpadPos = 1;
           }
       }
       else if(gamepad1.dpad_up) {
           if (DpadPos != -1) {
               arm.setTargetPosition(arm.getTargetPosition()+20);
               arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               arm.setPower(1);
               DpadPos = -1;
           }
        }

       else {
           DpadPos = 0;
       }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Arm Position", arm.getCurrentPosition());
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
    
    @Override
    public void stop() {
    }

}
