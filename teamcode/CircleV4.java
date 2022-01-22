

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import net.frogbots.skystone.opmodes.util.MaxSonarI2CXL;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class CircleV4 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor shooter = null;
    private DcMotor shooter2 = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private CRServo ringPusher;
    private Servo linkage;
    public RevColorSensorV3 enoughringssensor;
    private Servo wobblegrip;
    private Servo wobblearm;
    private DcMotor intake = null;
    private CRServo intakepusher;
    private RevColorSensorV3 colorside;
   // private RevColorSensorV3 singleshooter;
    private Servo intakeholder;
    private int A_pressed = 0;
    private int B_pressed = 0;
    private int X_pressed = 0;
    private int Y_pressed = 0;
    private int LB_pressed= 0;
    private int Up_pressed= 0;
    private int Down_pressed= 0;
    private int wobble_open = 1;
    private int wobblearm_down = 1;
    private int shootermove = 1;
    private int intakemove = 1;
    private MaxSonarI2CXL sonarfront;
    private MaxSonarI2CXL sonarback;
    private MaxSonarI2CXL sonarright;
    private MaxSonarI2CXL sonarleft;
    private  double currentTime;
    private int ring_exists = 0;
    public BNO055IMU imu;
    public double speed;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    private double travelAngle = 0;
    private double LF= 0;
    private double RF= 0;
    private double LB= 0;
    private double RB= 0;
    public boolean overrideA = false;
    public boolean overrideB = false;
    private double beginsittime;
    private boolean ringexists;
    private boolean ringgone;
    private double beginsittimeB;
    private double shooterspeed = 0.62;
    private double deadzone = 0.2;
    private double deadzone2 = (deadzone + 0.1);
    private double xyspeed = 1;
    private double turnspeed = 0.8;
    public Servo Autoclaw;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        shooter = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2= hardwareMap.get(DcMotor.class, "shooter2");
        ringPusher = hardwareMap.get(CRServo.class, "ringpusher");
        enoughringssensor = hardwareMap.get(RevColorSensorV3.class, "enough");
        colorside = hardwareMap.get(RevColorSensorV3.class, "colorside");
       // singleshooter = hardwareMap.get(RevColorSensorV3.class, "singleshooter");
        Autoclaw = hardwareMap.get(Servo.class, "AutoClaw");
        leftFront = hardwareMap.get(DcMotor.class, "motor03");
        rightFront = hardwareMap.get(DcMotor.class, "motor02");
        leftBack = hardwareMap.get(DcMotor.class, "motor01");
        rightBack = hardwareMap.get(DcMotor.class, "motor00");
        wobblegrip = hardwareMap.get(Servo.class, "servo01");
        wobblearm = hardwareMap.get(Servo.class, " servo00");
        intake = hardwareMap.get(DcMotor.class, "intake");
        linkage = hardwareMap.get(Servo.class, "linkage");
        intakeholder = hardwareMap.get(Servo.class, " intakeholder");
        sonarfront = hardwareMap.get(MaxSonarI2CXL.class, "sonarfront");
        sonarleft = hardwareMap.get(MaxSonarI2CXL.class, "sonarleft");
        sonarback = hardwareMap.get(MaxSonarI2CXL.class, "sonarback");
        sonarright = hardwareMap.get(MaxSonarI2CXL.class, "sonarright");
        imu =hardwareMap.get(BNO055IMU.class, "imu");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {


        // Setup a variable for the joysticks and triggers
        double left_YAxis;
        double left_XAxis;
        double right_YAxis;
        double right_XAxis;
        double left_trigger;
        double right_trigger;

        left_YAxis = -gamepad1.left_stick_y;
        left_XAxis = gamepad1.left_stick_x;
        right_YAxis = -gamepad1.right_stick_y;
        right_XAxis = gamepad1.right_stick_x;
        left_trigger = gamepad1.left_trigger;
        right_trigger = gamepad1.right_trigger;

        telemetry.addData("encoder val", shooter.getCurrentPosition());
        telemetry.addData("encoder val", shooter2.getCurrentPosition());
        travelAngle = getAngle();
        telemetry.addData("speed", shooterspeed);

        if (gamepad2.dpad_down){
            if (Down_pressed == 0) {
                shooterspeed -= 0.01;
                telemetry.addData("speed", shooterspeed);

            }
            Down_pressed = 1;
        } else
            Down_pressed = 0;

        if (gamepad2.dpad_up){
            if (Up_pressed == 0) {
                shooterspeed += 0.01;
                telemetry.addData("speed", shooterspeed);
            }
            Up_pressed = 1;
        } else
            Up_pressed = 0;


        if (gamepad1.y) {
            if (Y_pressed == 0) {
                if (shootermove == 1){
                    shooterstop();
                    shootermove = 0;
                }
                else {
                    shootergofast();
                    shootermove =1;
                }
            }
            Y_pressed = 1;
        } else
            Y_pressed = 0;

        if (gamepad1.right_bumper) {
            ringPusher.setPower(0.7);
        } else {
            ringPusher.setPower(0);
        }

      if (gamepad1.dpad_down){
         shootergo();
      }

      if (gamepad1.dpad_up) {
         overrideA = true;
         overrideB = true;
       }

      if (gamepad1.right_stick_button){
          overrideA = false;
          overrideB = false;
          resetAngle();
      }

       if (enoughringssensor.red() > 150 && overrideB == false){
            if (ringexists == false) {
                ringexists = true;
                beginsittime = getRuntime();
            } else {
                if ((getRuntime() - beginsittime > 0.5)) {
                    shootergofast();
                    intakestop();
                }
            }
       } else {
           ringexists = false;
       }

      if (colorside.red() < 200 && overrideA == false){
            if (ringgone == false) {
                ringgone = true;
                beginsittimeB = getRuntime();
            } else {
                if ((getRuntime() - beginsittimeB > 0.1)) {
                    shooterstop();
                    intakego();
                }
            }
      } else {
            ringgone = false;
      }

        //open
        if (gamepad1.x) {
            if (X_pressed == 0) {
                if (wobble_open == 1){
                    wobblegrip.setPosition(0.65);
                    wobble_open = 0;
                }
                else {
                    wobblegrip.setPosition(0.425);
                    wobble_open =1;
                }
            }
            X_pressed = 1;
        } else
            X_pressed = 0;

        if (gamepad1.b) {
            if (B_pressed == 0) {
                if ( wobblearm_down == 1){
                  //up
                    wobblearm.setPosition(0.01);
                    wobblearm_down = 0;
                }
                else {
                    //down
                    //0.32
                    wobblearm.setPosition(0.4);
                    wobblearm_down =1;
                }
            }
            B_pressed = 1;
        } else
            B_pressed = 0;


        if (left_trigger > 0) {
            turnleft(left_trigger);

        } else if (right_trigger > 0) {
            turnright(right_trigger);

        } else if (right_XAxis > deadzone) {
            straferight(right_XAxis * 0.5);

        } else if (right_XAxis < -deadzone) {
            strafeleft(right_XAxis * -0.5);

        } else if (right_YAxis > deadzone) {
            goforward(right_YAxis * 0.5);

        } else if (right_YAxis < -deadzone) {
            gobackward(right_YAxis * 0.5);
//.2
        } else if (left_XAxis > deadzone2) {
            straferight(left_XAxis);
//.2
        } else if (left_XAxis < deadzone2) {
            strafeleft(-left_XAxis);

        } else if (left_YAxis > deadzone) {
            goforward(left_YAxis);

        } else if (left_YAxis < -deadzone) {
            gobackward(left_YAxis);

        } else
            gostop();


        if (gamepad1.left_bumper) {
            if (LB_pressed == 0) {
                if (intakemove == 1){
                    intakestop();
                    intakemove = 0;
                }
                else {
                   intakego();
                    intakemove =1;
                }
            }
            LB_pressed = 1;
        } else
            LB_pressed = 0;

        if (gamepad1.a){
            intake.setPower(1);
        }

        while (gamepad1.dpad_left){
            ringPusher.setPower(-1);
            //XYgo('L',250);
        }

        if (gamepad1.dpad_right){
           XYgo('R',67);
           XYgo('F',188);
        }
        if (gamepad1.left_stick_button){
            intakeholder.setPosition(0.05);
        }
    }



    @Override
        public void stop () {
        }
            private void ringpush (long time){
                ringPusher.setPower(.7);
                }
            private void gostop (){
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }

          /*  private void goforward ( double speed){
            LF =(speed * 1);
            LB = (speed * 1);
            RF = (speed * 1);
            RB = (speed * 1);
            movez(LF,RF,LB,RB);
        }
            private void gobackward ( double speed){
                LF =(speed * 1);
                LB = (speed * 1);
                RF = (speed * 1);
                RB = (speed * 1);
                movez(LF,RF,LB,RB);
        }*/
          private void goforward ( double speed){
              leftFront.setPower(speed * xyspeed);
              leftBack.setPower(speed * xyspeed);
              rightFront.setPower(speed * xyspeed);
              rightBack.setPower(speed * xyspeed);
          }
        private void gobackward ( double speed){
             leftFront.setPower(speed * xyspeed);
             leftBack.setPower(speed * xyspeed);
             rightFront.setPower(speed * xyspeed);
             rightBack.setPower(speed * xyspeed);
    }
    private void strafeleft ( double speed){
        leftFront.setPower(speed * -xyspeed);
        leftBack.setPower(speed * xyspeed);
        rightFront.setPower(speed * xyspeed);
        rightBack.setPower(speed * -xyspeed);
    }
    private void straferight ( double speed){
        leftFront.setPower(speed * xyspeed);
        leftBack.setPower(speed * -xyspeed);
        rightFront.setPower(speed * -xyspeed);
        rightBack.setPower(speed * xyspeed);
    }
        /*   private void strafeleft ( double speed){
                LF =(speed * -1);
                LB = (speed * 1);
                RF = (speed * 1);
                RB = (speed * -1);
                movez(LF,RF,LB,RB);
        }
            private void straferight ( double speed){
                 LF = (speed * 1);
                 LB = (speed * -1);
                 RF= (speed * -1);
                 RB = (speed * 1);
                movez(LF,RF,LB,RB);
    }*/

            private void turnleft ( double speed){
                leftFront.setPower(speed * -turnspeed);
                leftBack.setPower(speed * -turnspeed);
                rightFront.setPower(speed * turnspeed);
                rightBack.setPower(speed * turnspeed);
                }
             private void turnright ( double speed){
                leftFront.setPower(speed * turnspeed);
                leftBack.setPower(speed * turnspeed);
                rightFront.setPower(speed * -turnspeed);
                rightBack.setPower(speed * -turnspeed);
                }
        private void shootergo(){
        shooter.setPower(0.55);
        shooter2.setPower(0.55);
        }
    private void shootergofast(){
        shooter.setPower(shooterspeed);
        shooter2.setPower(shooterspeed);
    }
    private void shooterstop(){
        shooter.setPower(0);
        shooter2.setPower(0);
    }
    private void intakego(){
        intake.setPower(-1);
    }
    private void intakestop(){
        intake.setPower(0);
    }
    private void XYgo(char sensor, int tdist){
        double FBspeed = 0.65;
        double LRspeed = 0.65;
        MaxSonarI2CXL sonarcurrent= sonarfront;
        if (sensor == 'F') sonarcurrent= sonarfront;
        if (sensor == 'B') sonarcurrent = sonarback;
        if (sensor == 'R') sonarcurrent = sonarright;
        if (sensor == 'L') sonarcurrent = sonarleft;
        double LF= 0;
        double RF= 0;
        double LB= 0;
        double RB= 0;

        if ((sensor == 'F' && tdist >= sonarcurrent.getDistanceSync()) || (sensor == 'B'  && tdist <= sonarcurrent.getDistanceSync()) ) {
            LF=(FBspeed*-1);
            RF=(FBspeed*-1);
            LB=(FBspeed*-1);
            RB=(FBspeed*-1);
        }

        if ((sensor == 'B' && tdist >= sonarcurrent.getDistanceSync()) || (sensor ==  'F' && tdist <= sonarcurrent.getDistanceSync())){
            LF = (FBspeed * 1);
            RF = (FBspeed * 1);
            LB = (FBspeed * 1);
            RB = (FBspeed * 1);

        }

        if ((sensor == 'L' && tdist >= sonarcurrent.getDistanceSync()) || (sensor == 'R' && tdist <= sonarcurrent.getDistanceSync())){
            LF=(LRspeed*1);
            RF=(LRspeed*-1);
            LB=(LRspeed*-1);
            RB=(LRspeed*1);
        }


        if ((sensor == 'R' && tdist >= sonarcurrent.getDistanceSync())|| (sensor == 'L' && tdist <= sonarcurrent.getDistanceSync())){
            LF=(LRspeed*-1);
            RF=(LRspeed*1) ;
            LB=(LRspeed*1);
            RB=(LRspeed*-1);
        }
        move(LF, RF, LB, RB, sonarcurrent, tdist);
    }
    public double getAngle(){
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
    public void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public void move(double LF, double RF,double LB,double RB, MaxSonarI2CXL sensor, int tdist) {
        int X = 0;  //Used to designate if we are traveling towards sensor or away from sensor direction
        double empval;
        if ((sensor == sonarleft)|| (sensor == sonarright)){
            empval= 0.03;
        }
        else empval = 0.014;

        if (sensor.getDistanceSync() > tdist) X = 1;
        while ((X == 1 && sensor.getDistanceSync() > tdist) || (X == 0 && sensor.getDistanceSync() < tdist)) {
            double SV;
            SV = Math.abs((tdist - sensor.getDistanceSync()) * (empval));
            //.2 - min
            SV = Math.min(Math.max(SV, 0.4), 1);

            double Factor = Math.max(Math.max(Math.max(LF*SV, LB*SV), Math.max(RF*SV, RB*SV)) / 20, 0.01);

            double LCorr = 0;
            double RCorr = 0;
            //find out if we are in need of correction - has our angle changed?
            double Correction = travelAngle - getAngle ();
            if (Correction > 1) {
                //A positive number means we need to correct with Left Turn
                RCorr += (Factor * Correction);
                LCorr -= (Factor * Correction);
            } else if (Correction < 1) {
                //A negative number means we need to correct with Right Turn
                RCorr -= (Factor * Math.abs(Correction));
                LCorr += (Factor * Math.abs(Correction));
            }
            leftFront.setPower((LF * SV) + (LCorr));
            rightFront.setPower((RF * SV) + (RCorr));
            leftBack.setPower((LB * SV) + (LCorr));
            rightBack.setPower((RB * SV)+ (RCorr));

            telemetry.addData("dist", sensor.getDistanceSync());
            telemetry.update();

        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }
    public void movez (double LF, double RF,double LB,double RB) {
        double LCorr = 0;
        double RCorr = 0;
        //find out if we are in need of correction - has our angle changed?
        double Correction = travelAngle - getAngle();
        if (Correction > 1) {
            //A positive number means we need to correct with Left Turn
            RCorr += (Correction);
            LCorr -= (Correction);
        } else if (Correction < 1) {
            //A negative number means we need to correct with Right Turn
            RCorr -= (Math.abs(Correction));
            LCorr += (Math.abs(Correction));
        }
        leftFront.setPower((LF) + (LCorr));
        rightFront.setPower((RF) + (RCorr));
        leftBack.setPower((LB) + (LCorr));
        rightBack.setPower((RB) + (RCorr));
    }

}


