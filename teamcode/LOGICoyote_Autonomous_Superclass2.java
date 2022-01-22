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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class LOGICoyote_Autonomous_Superclass2 extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFront, rightFront, leftBack, rightBack, liftarm;
    public Servo pinballgrabbersL, pinballgrabbersR, clawservoL, clawservoR;
    public BNO055IMU imu;
    public RevColorSensorV3 colorFront, colorBottom;
  //  public Rev2mDistanceSensor distRF, distRB, distLF, distLB;

    Orientation lastAngles = new Orientation();
    double globalAngle;
    private double travelAngle = 0;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "motor00");
        rightFront = hardwareMap.get(DcMotor.class, "motor01");
        leftBack = hardwareMap.get(DcMotor.class, "motor02");
        rightBack = hardwareMap.get(DcMotor.class, "motor03");
        pinballgrabbersL = hardwareMap.get(Servo.class, "servo03");
        pinballgrabbersR = hardwareMap.get(Servo.class, "servo02");
        clawservoL = hardwareMap.get(Servo.class, "servo00");
        clawservoR = hardwareMap.get(Servo.class, "servo01");
        liftarm = hardwareMap.get(DcMotor.class, "motor04");
        //   colorBottom = hardwareMap.get(RevColorSensorV3.class, "colorBottom");
        // colorFront = hardwareMap.get(RevColorSensorV3.class, "colorFront");
        //  distLB = hardwareMap.get(Rev2mDistanceSensor.class, "distLB");
        //  distLF = hardwareMap.get(Rev2mDistanceSensor.class, "distLF");
        //  distRB = hardwareMap.get(Rev2mDistanceSensor.class, "distRB");
        // distRF = hardwareMap.get(Rev2mDistanceSensor.class, "distRF");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized IMU");
        telemetry.update();

        travelAngle = getAngle();

        waitForStart();
        runtime.reset();
        Autonomous_steps();
    }

    //THIS MUST BE OVERRIDDEN BY THE ACTUAL CLASS
    abstract public void Autonomous_steps();

    public void pinballopen() {
        pinballgrabbersR.setPosition(0.4);
        pinballgrabbersL.setPosition(0.3);
    }

    public void pinballclose() {
        pinballgrabbersR.setPosition(0.65);
        pinballgrabbersL.setPosition(0.05);
    }

    public void liftarmup(long time) {
        liftarm.setPower(1);
        sleep(time);
        liftarm.setPower(0);
    }

    public void liftarmupE(double speed, long time, int EncoderTarget) {
        liftarm.setPower(1);
        sleep(time);
        liftarm.setPower(0);

    }

    public void liftdownE(double speed, long time, int EncoderTarget) {
        liftarm.setPower(-1);
        sleep(time);
        liftarm.setPower(0);

    }

    public void liftarmdown(long time) {
        liftarm.setPower(-1);
        sleep(time);
        liftarm.setPower(0);
    }

    public void openclaw() {
        clawservoL.setPosition(0.6);
        clawservoR.setPosition(0.35);
    }

    public void closeclaw() {
        clawservoL.setPosition(0.35);
        clawservoR.setPosition(0.6);
    }

    public void clawsideL() {
        clawservoL.setPosition(0.6);
        clawservoR.setPosition(0.55);
    }

    public void clawsideR() {
        clawservoL.setPosition(0.6);
        clawservoR.setPosition(0.35);
    }

    public void clawwide() {
        //0.85
        //0.1
        clawservoL.setPosition(0.8);
        clawservoR.setPosition(0.2);
    }

    public void gostop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void goforward(double speed, long time, int EncoderTarget) {
        IMUMove_Time_Enc(speed, speed, speed, speed, time, -EncoderTarget, -EncoderTarget, -EncoderTarget, -EncoderTarget);
    }

    public void goforwarda(double speed, long time, int EncoderTarget) {
        IMUMove_Time_EncAngle(speed, speed, speed, speed, time, travelAngle, -EncoderTarget, -EncoderTarget, -EncoderTarget, -EncoderTarget);
    }

    public void gobackward(double speed, long time, int EncoderTarget) {
        IMUMove_Time_Enc(speed * -1, speed * -1, speed * -1, speed * -1, time, EncoderTarget, EncoderTarget, EncoderTarget, EncoderTarget);
    }

    public void gobackwarda(double speed, long time, int EncoderTarget) {
        IMUMove_Time_EncAngle(speed * -1, speed * -1, speed * -1, speed * -1, time, travelAngle, EncoderTarget, EncoderTarget, EncoderTarget, EncoderTarget);
    }

    public void strafeleft(double speed, long time, int EncoderTarget) {
        IMUMove_Time_Enc(speed * -1, speed, speed, speed * -1, time, EncoderTarget, EncoderTarget, -EncoderTarget, -EncoderTarget);
    }

    public void strafelefta(double speed, long time, int EncoderTarget) {
        IMUMove_Time_EncAngle(speed * -1, speed, speed, speed * -1, time, travelAngle, EncoderTarget, EncoderTarget, -EncoderTarget, -EncoderTarget);
    }

    public void straferight(double speed, long time, int EncoderTarget) {
        IMUMove_Time_Enc(speed, speed * -1, speed * -1, speed, time, -EncoderTarget, -EncoderTarget, EncoderTarget, EncoderTarget);
    }

    public void straferighta(double speed, long time, int EncoderTarget) {
        IMUMove_Time_EncAngle(speed, speed * -1, speed * -1, speed, time, travelAngle, -EncoderTarget, -EncoderTarget, EncoderTarget, EncoderTarget);
    }

    public void godiagonalNE(double speed, long time, int EncoderTarget) {
        IMUMove_Time_Enc(speed, 0, 0, speed, time, -EncoderTarget, 0, 0, -EncoderTarget);
    }

    public void godiagonalNEa(double speed, long time, int EncoderTarget) {
        IMUMove_Time_EncAngle(speed, 0, 0, speed, time, travelAngle, -EncoderTarget, 0, 0, -EncoderTarget);
    }

    public void godiagonalSE(double speed, long time, int EncoderTarget) {
        IMUMove_Time_Enc(0, speed * -1, speed * -1, 0, time, 0, -EncoderTarget, -EncoderTarget, 0);
    }

    public void godiagonalSEa(double speed, long time, int EncoderTarget) {
        IMUMove_Time_EncAngle(0, speed * -1, speed * -1, 0, time, travelAngle, 0, -EncoderTarget, -EncoderTarget, 0);
    }

    public void godiagonalSW(double speed, long time, int EncoderTarget) {
        IMUMove_Time_Enc(speed * -1, 0, 0, speed * -1, time, EncoderTarget, 0, 0, -EncoderTarget);
    }

    public void godiagonalSWa(double speed, long time, int EncoderTarget) {
        IMUMove_Time_EncAngle(speed * -1, 0, 0, speed * -1, time, travelAngle, EncoderTarget, 0, 0, -EncoderTarget);
    }

    public void godiagonalNW(double speed, long time, int EncoderTarget) {
        IMUMove_Time_Enc(0, speed, speed, 0, time, 0, -EncoderTarget, -EncoderTarget, 0);
    }

    public void godiagonalNWa(double speed, long time, int EncoderTarget) {
        IMUMove_Time_EncAngle(0, speed, speed, 0, time, travelAngle, 0, -EncoderTarget, -EncoderTarget, 0);
    }

    public void turnleft(double speed, long time, int EncoderTarget) {
        leftFront.setPower(speed * -1);
        leftBack.setPower(speed * -1);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
        sleep(time);
        gostop();
    }

    public void turnright(double speed, long time) {
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed * -1);
        rightBack.setPower(speed * -1);
        sleep(time);
        gostop();
    }

    public void setTravelAngle() {
        travelAngle = getAngle();
    }

    public void IMUMove_Time_Enc(double LFPower, double LBPower, double RFPower, double RBPower, long time, int LFencoder, int LBencoder, int RFencoder, int RBencoder) {
        //get the time this started so that we can control driveby time
        double thisStartTime = runtime.milliseconds();
        int thisStartPositionLF = leftFront.getCurrentPosition();
        int thisStartPositionLB = leftBack.getCurrentPosition();
        int thisStartPositionRF = rightFront.getCurrentPosition();
        int thisStartPositionRB = rightBack.getCurrentPosition();

        //get current angle - we want to keep this angle
        double startAngle = getAngle();
        //Factor is an amount of correction to apply, and it calculated from the max power we are sending to the motors
        double Factor = Math.max(Math.max(Math.max(LFPower, LBPower), Math.max(RFPower, RBPower)) / 20, 0.01);
        //keep moving in this direction for time or until opMode is disabled

        while (runtime.milliseconds() - thisStartTime < time && opModeIsActive() &&
                ((LFencoder == 0 ||
                        ((LFencoder <= 0 || leftFront.getCurrentPosition() - thisStartPositionLF < LFencoder) &&
                                (LFencoder >= 0 || leftFront.getCurrentPosition() - thisStartPositionLF > LFencoder))) &&
                        (LBencoder == 0 ||
                                ((LBencoder <= 0 || leftBack.getCurrentPosition() - thisStartPositionLB < LBencoder) &&
                                        (LBencoder >= 0 || leftBack.getCurrentPosition() - thisStartPositionLB > LBencoder))) &&
                        (RFencoder == 0 ||
                                ((RFencoder <= 0 || rightFront.getCurrentPosition() - thisStartPositionRF < RFencoder) &&
                                        (RFencoder >= 0 || rightFront.getCurrentPosition() - thisStartPositionRF > RFencoder))) &&
                        (RBencoder == 0 ||
                                ((RBencoder <= 0 || rightBack.getCurrentPosition() - thisStartPositionRB < RBencoder) &&
                                        (RBencoder >= 0 || rightBack.getCurrentPosition() - thisStartPositionRB > RBencoder)))
                )) {
            double LCorr = 0;
            double RCorr = 0;
            //find out if we are in need of correction - has our angle changed?
            double Correction = startAngle - getAngle();
            if (Correction > 1) {
                //A positive number means we need to correct with Left Turn
                RCorr += (Factor * Correction);
                LCorr -= (Factor * Correction);
            } else if (Correction < 1) {
                //A negative number means we need to correct with Right Turn
                RCorr -= (Factor * Math.abs(Correction));
                LCorr += (Factor * Math.abs(Correction));
            }
            //Drive in the direction indicated by our parameters adding in the corrections calculated.
            leftFront.setPower(LFPower + LCorr);
            leftBack.setPower(LBPower + LCorr);
            rightFront.setPower(RFPower + RCorr);
            rightBack.setPower(RBPower + RCorr);
        }
        gostop();
    }

    public void IMUMove_Time_EncAngle(double LFPower, double LBPower, double RFPower, double RBPower, long time, double startAngle, int LFencoder, int LBencoder, int RFencoder, int RBencoder) {
        //get the time this started so that we can control driveby time
        double thisStartTime = runtime.milliseconds();
        int thisStartPositionLF = leftFront.getCurrentPosition();
        int thisStartPositionLB = leftBack.getCurrentPosition();
        int thisStartPositionRF = rightFront.getCurrentPosition();
        int thisStartPositionRB = rightBack.getCurrentPosition();

        //Factor is an amount of correction to apply, and it calculated from the max power we are sending to the motors
        double Factor = Math.max(Math.max(Math.max(LFPower, LBPower), Math.max(RFPower, RBPower)) / 20, 0.01);
        //keep moving in this direction for time or until opMode is disabled

        while (runtime.milliseconds() - thisStartTime < time && opModeIsActive() &&
                ((LFencoder == 0 ||
                        ((LFencoder <= 0 || leftFront.getCurrentPosition() - thisStartPositionLF < LFencoder) &&
                                (LFencoder >= 0 || leftFront.getCurrentPosition() - thisStartPositionLF > LFencoder))) &&
                        (LBencoder == 0 ||
                                ((LBencoder <= 0 || leftBack.getCurrentPosition() - thisStartPositionLB < LBencoder) &&
                                        (LBencoder >= 0 || leftBack.getCurrentPosition() - thisStartPositionLB > LBencoder))) &&
                        (RFencoder == 0 ||
                                ((RFencoder <= 0 || rightFront.getCurrentPosition() - thisStartPositionRF < RFencoder) &&
                                        (RFencoder >= 0 || rightFront.getCurrentPosition() - thisStartPositionRF > RFencoder))) &&
                        (RBencoder == 0 ||
                                ((RBencoder <= 0 || rightBack.getCurrentPosition() - thisStartPositionRB < RBencoder) &&
                                        (RBencoder >= 0 || rightBack.getCurrentPosition() - thisStartPositionRB > RBencoder)))
                )) {
            double LCorr = 0;
            double RCorr = 0;
            //find out if we are in need of correction - has our angle changed?
            double Correction = startAngle - getAngle();
            if (Correction > 1) {
                //A positive number means we need to correct with Left Turn
                RCorr += (Factor * Correction);
                LCorr -= (Factor * Correction);
            } else if (Correction < 1) {
                //A negative number means we need to correct with Right Turn
                RCorr -= (Factor * Math.abs(Correction));
                LCorr += (Factor * Math.abs(Correction));
            }
            //Drive in the direction indicated by our parameters adding in the corrections calculated.
            leftFront.setPower(LFPower + LCorr);
            leftBack.setPower(LBPower + LCorr);
            rightFront.setPower(RFPower + RCorr);
            rightBack.setPower(RBPower + RCorr);
        }
        gostop();
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    public boolean IsSkystone() {
        //run three loops - if they do not agree, run 3 again
        int loopcnt = 0;
        int fallout = 9;
        boolean returnValue = false;
        boolean confirm = false;
        double readValue[];
        double checkvalue = 74;
        readValue = new double[3];
        while (!confirm && fallout > 0) {
            while (loopcnt < 3) {
                int greenValue = colorFront.green();
                // double distValue = colorFront.getDistance(DistanceUnit.MM);
                // double checkValue = (distValue*distValue*greenValue)/10000;
                //readValue[loopcnt] = checkValue;
                loopcnt++;
            }
            if (readValue[0] < checkvalue && readValue[1] < checkvalue && readValue[2] < checkvalue) {
                confirm = true;
                returnValue = true;
            }
            if (readValue[0] > checkvalue && readValue[1] > checkvalue && readValue[2] > checkvalue) {
                confirm = true;
                returnValue = false;
            }
            fallout--;
        }
        return returnValue;
    }

    //public boolean Liftarmupenough() {
        //  double LFValue = distLF.getDistance(DistanceUnit.MM);
        //  double RFValue = distRF.getDistance(DistanceUnit.MM);
        //double ColorDist = colorFront.getDistance(DistanceUnit.MM);
        // telemetry.addData("Left Front Distance", LFValue);
        // telemetry.addData("Right Front Distance", RFValue);
        //telemetry.addData("Color Distance", ColorDist);

      //  telemetry.update();

        //if (LFValue < 200 && RFValue < 200) return true;
        // else return false;
  //  }



   // public boolean backuptofoundation (){
       // double LBValue = distLF.getDistance(DistanceUnit.MM);
      //  double RBValue = distRF.getDistance(DistanceUnit.MM);
        //double ColorDist = colorFront.getDistance(DistanceUnit.MM);
        //telemetry.addData("Left Front Distance", LBValue);
       // telemetry.addData("Right Front Distance", RBValue);
      //  telemetry.addData("Color Distance", ColorDist);
        //telemetry.update();

       //if (LBValue < 10 && RBValue < 10) return true;
       // else return false;
    //}

   // public boolean CloseToBlocks (){
        //double LFValue = distLF.getDistance(DistanceUnit.MM);
        //double RFValue = distRF.getDistance(DistanceUnit.MM);
      //  double ColorDist = colorFront.getDistance(DistanceUnit.MM);
        //telemetry.addData("Left Front Distance", LFValue);
        //telemetry.addData("Right Front Distance", RFValue);
       // telemetry.addData("Color Distance", ColorDist);
        //telemetry.update();

        //if (ColorDist < 30 ) return true;
       // else return false;
    //}

    //public boolean MovetoBlocks(long defaultTime){
        //get the time this started so that we can control driveby time
       // double thisStartTime = runtime.milliseconds();
        //get current angle - we want to keep this angle
       // double startAngle = getAngle();
      //  byte zeroAngle = 0;
        //boolean contloop = true;
        //long loopcount = 0;
        //keep moving in this direction for time or until opMode is disabled
        //goforward(0.17, defaultTime, 54321);
        //gostop();
        //sleep(250);
        //while (contloop) {
            //if (CloseToBlocks() )
               // contloop = false;
           // else {
             //   goforward(0.15, 300, 54321);
               // sleep(100);
         //   }
           // loopcount++;
           // if (loopcount > 15)
           //     contloop = false;
    //    }
      //  return true;
   // }

    public void rotate(int degrees, double power, long timeout)
    {
        double  leftPower, rightPower;
        double thisStartTime = runtime.milliseconds();

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);
        leftBack.setPower(leftPower);
        rightBack.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0 && runtime.milliseconds() - thisStartTime < timeout) {}

            while (opModeIsActive() && getAngle() > degrees && runtime.milliseconds() - thisStartTime < timeout) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees && runtime.milliseconds() - thisStartTime < timeout) {}

        // turn the motors off.
        gostop();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }


}
