/* Copyright Lydia & M1
Take it, I don't care
Programming is just copying
 */


package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class CheddarV1 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
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
    private int intakestop = 1;
    private Servo OL;
    private Servo OR;
    private Servo OS;
    private DcMotor ducks;
    private int a = 0;
    @Override
    public void init() {
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
    }


    @Override
    public void start() {
        runtime.reset();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        telemetry.addData("Mailbox",mailbox.getPosition());

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

        if (left_trigger > 0) {
            turnleft(-1* left_trigger);

        } else if (right_trigger > 0) {
            turnright(-1* right_trigger);
        }
        else if (right_XAxis > 0.2) {
            straferight(right_XAxis * 0.5);

        } else if (right_XAxis < -0.2) {
            strafeleft(right_XAxis * -0.5);

        } else if (right_YAxis > 0.2) {
            goforward(right_YAxis * 0.5);

        } else if (right_YAxis < -0.2) {
            gobackward(right_YAxis * 0.5);

        } else if (left_XAxis > 0.2) {
            straferight(left_XAxis);

        } else if (left_XAxis < -0.2) {
            strafeleft(-left_XAxis);

        } else if (left_YAxis > 0.2) {
            goforward(left_YAxis);

        } else if (left_YAxis < -0.2) {
            gobackward(left_YAxis);

        } else
            gostop(0);

        if(gamepad1.x) {
            intake.setPower(1);
        }
        if (gamepad1.b) {
            if (Rightpressed == 0) {
                if (intakestop == 1){
                    intake.setPower(-0.6);
                    intakestop = 0;
                }
                else {
                    intake.setPower(0);
                    intakestop =1;
                }
            }
            Rightpressed = 1;
        } else
            Rightpressed = 0;

        if (gamepad1.a){
            servoliftNeg.setPosition(1);
            servoliftPos.setPosition(0);
        }
        if (gamepad1.y){
            //0.8
            servoliftNeg.setPosition(0.6);
            //0.2
            servoliftPos.setPosition(0.4);
            //mailbox.setPosition(0.7);

        }

       if (gamepad1.right_bumper){
           arm.setTargetPosition(500);
           if (arm.getCurrentPosition() >= 500) {
               arm.setPower(0);
           }
           arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           arm.setPower(0.4);
       }
        if (gamepad1.left_bumper){
            mailbox.setPosition(1);
            arm.setTargetPosition(0);
            if (arm.getCurrentPosition() <= 0) {
                arm.setPower(0);
            }
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.4);
        }
        if (gamepad1.dpad_up){
        mailbox.setPosition(1);
        }
        if (gamepad1.dpad_down){
            //dump
        mailbox.setPosition(0);
        }
        if (gamepad1.dpad_right) {
            ducks.setPower(0.9);
        } else {
            ducks.setPower(0);
        }
        if (gamepad1.dpad_left){
            mailbox.setPosition(1);
        }
        //driver 2 controls
       //arm up
        if (gamepad2.right_bumper) {
            arm.setTargetPosition(750);
            if (arm.getCurrentPosition() >= 750) {
                arm.setPower(0);
            }
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.4);
        }
        //arm down
        if (gamepad2.left_bumper){
            mailbox.setPosition(1);
            arm.setTargetPosition(0);
            if (arm.getCurrentPosition() <= 0) {
                arm.setPower(0);
            }
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.4);
        }
        //dump
        if (gamepad2.y){
            mailbox.setPosition(0.7);
        }
        //lift down
        if (gamepad2.dpad_down){
            servoliftNeg.setPosition(1);
            servoliftPos.setPosition(0);
        }
        //lift up
        if (gamepad2.dpad_up){

            servoliftNeg.setPosition(0.6);
            //0.2
            servoliftPos.setPosition(0.4);
            //mailbox.setPosition(0.7);
        }
        if (gamepad2.a){
        mailbox.setPosition(1);
        }
        if (gamepad2.x){
            //blue duck spin
            ducks.setPower(0.9);
        }
        if (gamepad2.b){
            //red duck spin
            ducks.setPower(-0.9);
        }
    }


    @Override
    public void stop() {
    }
    private void gostop(double speed) {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    private void goforward(double speed) {
        leftFront.setPower(speed * movespeed);
        leftBack.setPower(speed * movespeed);
        rightFront.setPower(speed * movespeed);
        rightBack.setPower(speed * movespeed);
    }
    private void gobackward(double speed) {
        leftFront.setPower(speed * movespeed);
        leftBack.setPower(speed * movespeed);
        rightFront.setPower(speed * movespeed);
        rightBack.setPower(speed * movespeed);
    }
    private void strafeleft(double speed) {
        leftFront.setPower(speed * -1 * movespeed);
        leftBack.setPower(speed * movespeed);
        rightFront.setPower(speed * movespeed);
        rightBack.setPower(speed * -1 * movespeed);
    }
    private void straferight(double speed) {
        leftFront.setPower(speed * movespeed);
        leftBack.setPower(speed * -1 * movespeed);
        rightFront.setPower(speed * -1 * movespeed);
        rightBack.setPower(speed * movespeed);
    }
    private void turnleft(double speed) {
        leftFront.setPower(speed *  -1 * turnspeed);
        leftBack.setPower(speed * -1 * turnspeed);
        rightFront.setPower(speed * turnspeed);
        rightBack.setPower(speed * turnspeed);
    }
    private void turnright(double speed) {
        leftFront.setPower(speed * turnspeed);
        leftBack.setPower(speed * turnspeed);
        rightFront.setPower(speed *  -1 * turnspeed);
        rightBack.setPower(speed *  -1 * turnspeed);
    }

}