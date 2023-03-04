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
public class SuperCycler extends LinearOpMode {

    //Value of slides at high junction (use ArmReader)
    double SLIDES_HIGH = 2500;

    //Value of slides at intake height (use ArmReader)
    double SLIDES_INTAKE = 325;

    double SLIDES_HOLD = SLIDES_INTAKE + 125;

    //Value of forebar at intake position (use ServoZeroer and trial and error the values)
    double FOREBAR_INTAKE = 0.8;

    //Value of forebar at outtake position (use ServoZeroer and trial and error the values)
    double FOREBAR_OUTTAKE = 0.3;

    //Value of clawbar at cone grabbing position (use LinkageTest to find trial and error the values)
    double CLAWBAR_GRAB_POS = 0.72;

    double LINKAGE_OUT_POS = 0.22;

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

    //robot status boolean values
    boolean linkageArm = false;


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
            if (beginning == 0) {
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);

                beginning++;
            }


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
            LF += X1;
            RF -= X1;
            LB += X1;
            RB -= X1;


            //Set Motors
            motorfrontLeft.setPower(LF);
            motorfrontRight.setPower(RF);
            motorbackLeft.setPower(LB);
            motorbackRight.setPower(RB);

            if (gamepad1.y) {
                liftOdo(4.0);
                stopOdo(0.1);
            }

            if (gamepad2.y){

            }
            if(gamepad2.x) {
                startCycle();
            }
            else if (gamepad2.a) {
                triumvirate();
            }

            if (gamepad2.b) {
                int numRuns = 5;
                while (numRuns > 0) {
                    triumvirate();
                    numRuns--;
                }
            }
        }
    }

    public void linkageArmOut(boolean toggle){
        if(toggle){
            clawOpen(0.01);
            linkageOut(LINKAGE_OUT_POS, 0.01);
            
        }
    }
    public void linkageIn(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftLinkage.setPosition(0.05);
            rightLinkage.setPosition(0.05);
        }
    }

    public void linkageOut(double pos, double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftLinkage.setPosition(pos);
            rightLinkage.setPosition(pos);
        }
    }

    public void move4bar(double pos, double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            left4bar.setPosition(pos-0.025);
            right4bar.setPosition(pos);;
        }
    }

    public void forebarIPos(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftForebar.setPosition(FOREBAR_INTAKE);
            rightForebar.setPosition(FOREBAR_INTAKE);
        }
    }

    public void forebarHold(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftForebar.setPosition(.7);
            rightForebar.setPosition(.675);
        }
    }

    public void forebarOPos(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftForebar.setPosition(FOREBAR_OUTTAKE+0.025);
            rightForebar.setPosition(FOREBAR_OUTTAKE);
        }
    }

    public void clawOpen(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            claw.setPosition(0);
        }
    }

    public void clawClose(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            claw.setPosition(0.25);
        }
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
        double run = (runtime.time() + time);
        while (runtime.time() < run) {
            intake.setPower(0);
        }
    }

    public void moveArmUp(double pos, double speed) {
        while(((-1*leftSlide.getCurrentPosition()) < pos) || ((-1*rightSlide.getCurrentPosition()) < pos)) {
            leftSlide.setPower(speed);
            rightSlide.setPower(speed);
        }
        holdArm();
    }

    public void moveArmDown(double pos, double speed) {
        while(((-1*leftSlide.getCurrentPosition()) > pos) || ((-1*rightSlide.getCurrentPosition()) > pos)) {
            leftSlide.setPower(-speed);
            rightSlide.setPower(-speed);
        }
        holdArm();
    }

    public void startArmDown(double speed) {
        leftSlide.setPower(-speed);
        rightSlide.setPower(-speed);
    }

    public void holdSlides(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftSlide.setPower(0.1);
            rightSlide.setPower(0.1);
        }
    }

    public void stopSlides(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftSlide.setPower(0);
            rightSlide.setPower(0);

        }
    }

    public void holdArm() {
        leftSlide.setPower(0.1);
        rightSlide.setPower(0.1);
    }

    public void startCycle(){
        forebarIPos(0.1);
        moveArmUp(SLIDES_HOLD, 0.5);
        holdArm();
        clawOpen(0.2);
        linkageOut(LINKAGE_OUT_POS-0.1, 0.2);
        move4bar(CLAWBAR_GRAB_POS, 0.5);
        linkageOut(LINKAGE_OUT_POS, 0.5);
        clawClose(0.1);
        move4bar(0, 0.5);
        linkageIn(0.35);
    }

    public void triumvirate() {
        forebarIPos(0.0000001);
        intake(0.1);
        moveArmDown(SLIDES_INTAKE, 0.5);
        holdSlides(0.5);
        clawOpen(0.01);
        //Slides go high
        moveArmUp(SLIDES_HIGH, 1.0);

        //Linkage prepares to grab
        linkageOut(LINKAGE_OUT_POS-0.1, 0.01);
        move4bar(CLAWBAR_GRAB_POS, 0.01);


        //Pull in cone and drop cone
        forebarOPos(1.0);
        /*while(!gamepad2.right_bumper) {
            if (gamepad1.left_stick_y < 0) {
                motorfrontLeft.setPower(0.1);
                motorfrontRight.setPower(0.1);
                motorbackLeft.setPower(0.1);
                motorbackRight.setPower(0.1);
            }
            //Move back
            else if (gamepad1.left_stick_y > 0) {
                motorfrontLeft.setPower(-0.1);
                motorfrontRight.setPower(-0.1);
                motorbackLeft.setPower(-0.1);
                motorbackRight.setPower(-0.1);
            }
            //Strafe right
            else if (gamepad1.dpad_right) {
                motorfrontLeft.setPower(-0.3);
                motorfrontRight.setPower(0.3);
                motorbackLeft.setPower(0.3);
                motorbackRight.setPower(-0.3);
            }
            //Strafe left
            else if (gamepad1.dpad_left) {
                motorfrontLeft.setPower(0.3);
                motorfrontRight.setPower(-0.3);
                motorbackLeft.setPower(-0.3);
                motorbackRight.setPower(0.3);
            }
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }
        }*/

        outtake(0.5);
        stopIntake(0.01);
        forebarIPos(0.01);
        linkageOut(LINKAGE_OUT_POS, 0.5);
        clawClose(0.2);
        startArmDown(0.1);
        move4bar(0, 0.01);
        linkageIn(0.01);
        intake(0.01);
        moveArmDown(SLIDES_INTAKE-25, 0.1);
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