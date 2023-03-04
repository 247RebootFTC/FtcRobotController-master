package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import java.util.*;
import java.lang.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DigitalChannel;

//@Disabled
@TeleOp
public class Cycler extends LinearOpMode {

    /*CHANGE THESE*/

    //Value of slides at high junction (use ArmReader)
    double SLIDES_HIGH = 3100;

    //Value of slides at intake height (use ArmReader)
    double SLIDES_INTAKE = 500;

    //Value of forebar at intake position (use ServoZeroer and trial and error the values)
    double FOREBAR_INTAKE = 0.8;

    //Value of forebar at outtake position (use ServoZeroer and trial and error the values)
    double FOREBAR_OUTTAKE = 0.3;

    //Value of clawbar at cone grabbing position (use LinkageTest to find trial and error the values)
    double CLAWBAR_GRAB_POS = 0.8;

    int beginning = 0;


    //declare Drive Motors
    private DcMotor motorfrontLeft;
    private DcMotor motorfrontRight;
    private DcMotor motorbackLeft;
    private DcMotor motorbackRight;

    //Declare Mechanism Motors
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    //Declare CR Servos
    private CRServo intake;
    //Odometry Servos
    private CRServo leftOdo;
    private CRServo rightOdo;
    private CRServo frontOdo;

    //Declare Regular Servos
    private Servo leftForebar;
    private Servo rightForebar;
    private Servo rightLinkage;
    private Servo leftLinkage;
    private Servo left4bar;
    private Servo right4bar;
    private Servo claw;

    //Time Variable
    private ElapsedTime runtime = new ElapsedTime();

    //motor speed variables
    double LF;
    double RF;
    double LB;
    double RB;
    //double TL;
    //double TR;
    //POWER
    double pwr = 3;

    //Joystick position variables
    double X1;
    double Y1;
    double X2;
    double Y2;
    double iX1;
    double iY1;
    double iX2;
    double iY2;
    //double D1;
    //double D2;

    //analog values
    double joyScale = 0.6;
    double joyScale2 = 0.6;
    double motorMax = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize Drive Motors
        motorfrontLeft = hardwareMap.dcMotor.get("motorfrontLeft");
        motorfrontRight = hardwareMap.dcMotor.get("motorfrontRight");
        motorbackLeft = hardwareMap.dcMotor.get("motorbackLeft");
        motorbackRight = hardwareMap.dcMotor.get("motorbackRight");

        //Initialize Mechanism Motors
        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        rightSlide = hardwareMap.dcMotor.get("rightSlide");

        //Initialize CR (Continuous Rotation) Servos
        intake = hardwareMap.crservo.get("intake");

        //Odometry Servos
        leftOdo = hardwareMap.crservo.get("leftOdo");
        rightOdo = hardwareMap.crservo.get("rightOdo");
        frontOdo = hardwareMap.crservo.get("frontOdo");

        //Initialize Regular Servos
        leftForebar = hardwareMap.servo.get("leftForebar");
        rightForebar = hardwareMap.servo.get("rightForebar");
        leftLinkage =  hardwareMap.servo.get("leftLinkage");
        rightLinkage = hardwareMap.servo. get("rightLinkage");
        left4bar =  hardwareMap.servo.get("left4bar");
        right4bar = hardwareMap.servo. get("right4bar");
        claw = hardwareMap.servo. get("claw");

        //CHECK THESE

        //Initialize Drive Motors' Directions
        motorfrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorfrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorbackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorbackRight.setDirection(DcMotor.Direction.REVERSE);

        //Initialize Mechanism Motors' Directions
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        //Initialize Servos' Directions
        leftLinkage.setDirection(Servo.Direction.REVERSE);
        rightLinkage.setDirection(Servo.Direction.FORWARD);

        leftForebar.setDirection(Servo.Direction.FORWARD);
        rightForebar.setDirection(Servo.Direction.REVERSE);

        left4bar.setDirection(Servo.Direction.FORWARD);
        right4bar.setDirection(Servo.Direction.REVERSE);

        initEncoder();

        telemetry.addData("DRAGON: ", "I am in extremely extreme agony");
        telemetry.update();

        //Wait to start code
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        //All motors are backwards except ones that are used
        while (opModeIsActive()) {

            //automatically sets fore
           /* if (beginning == 0) {
                leftForebar.setPosition(0.25);
                rightForebar.setPosition(0.25);

                /*liftOdo(0.01);
                stopOdo(1.0);

                beginning = 1;
            }*/


            //reset speed variables
            LF = 0;
            RF = 0;
            LB = 0;
            RB = 0;

            Y1 = gamepad1.right_stick_y * joyScale;
            X1 = gamepad1.right_stick_x * joyScale;
            Y2 = gamepad1.left_stick_y * joyScale;
            X2 = gamepad1.left_stick_x * joyScale2;


            //Forward/Backward
            LF += Y2;
            RF += Y2;
            LB += Y2;
            RB += Y2;

            //Strafing
            LF += X2;
            RF -= X2;
            LB -= X2;
            RB += X2;

            //Turning
            LF -= X1;
            RF += X1;
            LB -= X1;
            RB += X1;

            //Wheel power limiter
            /*LF = Math.max(-motorMax, Math.min(LF, motorMax));
            RF = Math.max(-motorMax, Math.min(RF, motorMax));
            LB = Math.max(-motorMax, Math.min(LB, motorMax));
            RB = Math.max(-motorMax, Math.min(RB, motorMax));*/


            //Set Motors
            motorfrontLeft.setPower(LF);
            motorfrontRight.setPower(RF);
            motorbackLeft.setPower(LB);
            motorbackRight.setPower(RB);

            //CHANGE THE VALUES AT THE TOP AND THEN PRESS A
            //SEND ME A MESSAGE IF SOMETHING DOESN'T WORK
            if (gamepad2.a) {
                cycle();
                //triumvirate();
                //coneDropper();
            }

            if (gamepad2.b) {
                int numRuns = 5;
                while (numRuns > 0) {
                    cycle();
                    numRuns--;
                }
            }

            if (gamepad1.y) {
                liftOdo(4.0);
                stopOdo(0.1);
            }
        }
    }


    public void linkageIn() {
        leftLinkage.setPosition(0);
        rightLinkage.setPosition(0);
    }

    public void linkageOut() {
        leftLinkage.setPosition(0.2);
        rightLinkage.setPosition(0.2);
    }

    public void move4bar(double pos) {
        left4bar.setPosition(pos-0.025);
        right4bar.setPosition(pos);
    }

    public void forebarIntake() {
        leftForebar.setPosition(FOREBAR_INTAKE);
        rightForebar.setPosition(FOREBAR_INTAKE);
    }

    public void forebarOuttake() {
        leftForebar.setPosition(FOREBAR_OUTTAKE);
        rightForebar.setPosition(FOREBAR_OUTTAKE);
    }

    public void clawOpen() {
        claw.setPosition(0);
    }

    public void clawClose() {
        claw.setPosition(0.2);
    }

    public void liftOdo(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftOdo.setPower(-1.0);
            rightOdo.setPower(-0.6);
            frontOdo.setPower(-1.0);
        }
    }

    public void stopOdo(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftOdo.setPower(0);
            rightOdo.setPower(0);
            frontOdo.setPower(0);
        }
    }

    public void stopSlides(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftSlide.setPower(0);
            rightSlide.setPower(0);

        }
    }

    public void intake(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            intake.setPower(-1);
        }
    }

    public void outtake(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            intake.setPower(1);
        }
    }

    public void stopIntake(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            intake.setPower(0);
        }
    }

    public void cycle() {
        clawOpen();
        keepClaw(1);
        linkageOut();
        keepLinkage(1);
        move4bar(CLAWBAR_GRAB_POS);
        keep4bar(1);
        clawClose();
        keepClaw(1);
        move4bar(0);
        keep4bar(1);
        linkageIn();
        keepLinkage(1);
        forebarIntake();
        keepForebar(1);
        intake(0.5);
        moveArmUp(SLIDES_HIGH);
        forebarOuttake();
        keepForebar(1);
        outtake(0.5);
        stopIntake(0.1);
        forebarIntake();
        keepForebar(1);
        moveArmDown(SLIDES_INTAKE);
    }

    public void triumvirate() {

    }

    public void coneDropper() {

    }

    public void moveArmUp(double pos) {
        while(((-1*leftSlide.getCurrentPosition()) < pos) || ((-1*rightSlide.getCurrentPosition()) < pos)) {
            leftSlide.setPower(1.0);
            rightSlide.setPower(1.0);
        }
        stopSlides(0.1);
    }

    public void moveArmDown(double pos) {
        while(((-1*leftSlide.getCurrentPosition()) > pos) || ((-1*rightSlide.getCurrentPosition()) > pos)) {
            leftSlide.setPower(-1.0);
            rightSlide.setPower(-1.0);
        }
        stopSlides(0.1);
    }

    public void keepForebar(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftForebar.setPosition(leftForebar.getPosition());
            rightForebar.setPosition(rightForebar.getPosition());
        }
    }

    public void keepClaw(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            claw.setPosition(claw.getPosition());
        }
    }

    public void keepLinkage(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftLinkage.setPosition(leftLinkage.getPosition());
            rightLinkage.setPosition(rightLinkage.getPosition());
        }
    }

    public void keep4bar(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            left4bar.setPosition(left4bar.getPosition());
            right4bar.setPosition(right4bar.getPosition());
        }
    }

    public void initEncoder() {
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        /*telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                leftSlide.getCurrentPosition(),
                rightSlide.getCurrentPosition());
        telemetry.update();*/
    }

}