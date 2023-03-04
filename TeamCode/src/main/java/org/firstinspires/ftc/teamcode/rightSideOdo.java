/*package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;*/

/*
 * This is an example of a more complex path to really test the tuning.
 */
/*@Autonomous(group = "drive")
@Disabled
public class rightSideOdo extends LinearOpMode {

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

    double forebarPos = 0.25;

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

    OpenCvWebcam webcam;
    SkystoneDeterminationPipeline pipeline = new SkystoneDeterminationPipeline();

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {*/
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                /*webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
*/
            /*@Override
            public void onError(int errorCode) {*/
                /*
                 * This will be called if the camera could not be opened
                 */
                /*telemetry.addLine("error");
                telemetry.update();
            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData(">", "DRAGON: FIIIIIIIIIIRE");
        telemetry.update();

        waitForStart();

        String color;

        //if (isStopRequested()) return;



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
        //Initialize Regular Servos
        leftForebar = hardwareMap.servo.get("leftForebar");
        rightForebar = hardwareMap.servo.get("rightForebar");
        leftLinkage =  hardwareMap.servo.get("leftLinkage");
        rightLinkage = hardwareMap.servo. get("rightLinkage");
        left4bar =  hardwareMap.servo.get("left4bar");
        right4bar = hardwareMap.servo. get("right4bar");
        claw = hardwareMap.servo. get("claw");

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

        if(opModeIsActive()) {

            intake(1.0);
            keepForebar(0.25);

            Trajectory traj = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-24, 0, Math.toRadians(0)))
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-24, 0, Math.toRadians(0)), Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-48, 0, Math.toRadians(-100)))
                    .build();

            Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(-48, 0, Math.toRadians(-100)), Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-62, -7.5, Math.toRadians(-100)))
                    .build();

            Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(-62, -7.5, Math.toRadians(-100)), Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-48, 0, Math.toRadians(-100)))
                    .build();

            Trajectory park2 = drive.trajectoryBuilder(new Pose2d(-48, 0, Math.toRadians(-100)), Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-36, 0, Math.toRadians(180)))
                    .build();

            Trajectory park1= drive.trajectoryBuilder(new Pose2d(-48, 0, Math.toRadians(-100)), Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-48, -24, Math.toRadians(180)))
                    .build();

            Trajectory park3= drive.trajectoryBuilder(new Pose2d(-48, 0, Math.toRadians(-100)), Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-48, 24, Math.toRadians(180)))
                    .build();

            drive.followTrajectory(traj);

            linkageOut(0.2);
            move4bar(CLAWBAR_GRAB_POS);

            color = pipeline.getAnalysis();
            telemetry.addData("Analysis", color);
            telemetry.update();

            keep4bar(1);
            linkageIn();
            move4bar(0);

            forebarOuttake();
            keepForebar(0.5);
            moveArmUp(SLIDES_HIGH);

            drive.followTrajectory(traj2);

            drive.followTrajectory(traj3);

            stopIntake(0.1);
            outtake(1.0);
            stopIntake(0.1);

            drive.followTrajectory(traj4);

            forebarIntake();
            moveArmDown(0);*/

            /*CYCLING*/
            /*cycle(clawbarHeight, linkageDistance);

            drive.followTrajectory(park1);

            /*if (color == "GREEN") {

            }
            else if (color == "PURPLE") {

              //  drive.followTrajectory(park2);

            }
            else if (color == "ORANGE") {

         */   /*}
        }

    }

    public void linkageIn() {
        leftLinkage.setPosition(0);
        rightLinkage.setPosition(0);
    }

    public void linkageOut(double distance) {
        leftLinkage.setPosition(distance);
        rightLinkage.setPosition(distance);
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

    public void cycle(double clawbarHeight, double linkageDistance) {
        clawOpen();
        keepClaw(1);
        linkageOut(linkageDistance);
        keepLinkage(1);
        move4bar(clawbarHeight);
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

    public void holdArm(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftSlide.setPower(0.1);
            rightSlide.setPower(0.1);

        }
    }

    public void moveArmUp(double pos) {
        while(((-1*leftSlide.getCurrentPosition()) < pos) || ((-1*rightSlide.getCurrentPosition()) < pos)) {
            leftSlide.setPower(1.0);
            rightSlide.setPower(1.0);
        }
        stopSlides(0.1);
        leftSlide.setPower(0.1);
        rightSlide.setPower(0.1);
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
*/
        // Send telemetry message to indicate successful Encoder reset
        /*telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                leftSlide.getCurrentPosition(),
                rightSlide.getCurrentPosition());
        telemetry.update();*/
    /*}

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {*/


        /*
         * Some color constants
         */
       /* static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);
*/
        /*
         * The core values which define the location and size of the sample regions
         */
  //      static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(140,100);
    //    static final int REGION_WIDTH = 40;
      //  static final int REGION_HEIGHT = 40;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        //Point region_pointA = new Point(
          //      REGION_TOPLEFT_ANCHOR_POINT.x,
            //    REGION_TOPLEFT_ANCHOR_POINT.y);
        //Point region_pointB = new Point(
          //      REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            //    REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        //Mat region_Cb, region_Cr;
     //   Mat YCrCb = new Mat();
       // Mat Cb = new Mat();
       // Mat Cr = new Mat();
       // private int Cb_val, Cr_val;

        //private String position;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel
         *  to the 'Cb' variable
         */
        /*void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        void inputToCr(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {*/
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
           /* inputToCb(firstFrame);
            inputToCr(firstFrame);
*/
            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
  /*          region_Cb = Cb.submat(new Rect(region_pointA, region_pointB));
            region_Cr = Cr.submat(new Rect(region_pointA, region_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {*/
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
           // inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */

            //Cb_val = (int) Core.mean(region_Cb).val[0];

            //inputToCr(input);
            //Cr_val = (int) Core.mean(region_Cr).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    region_pointA, // First point which defines the rectangle
                    region_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    5); // Thickness of the rectangle lines
*/
            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
     /*       if((Cb_val > 125)&&(Cr_val > 110)) // Was it from region 1?
            {
                position = "PURPLE"; // Record our analysis
*/
                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
  /*              Imgproc.rectangle(
                        input, // Buffer to draw on
                        region_pointA, // First point which defines the rectangle
                        region_pointB, // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if((Cb_val < 120)&&(Cr_val > 125)) // Was it from region 2?
            {
                position = "ORANGE"; // Record our analysis
*/
                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
       /*         Imgproc.rectangle(
                        input, // Buffer to draw on
                        region_pointA, // First point which defines the rectangle
                        region_pointB, // Second point which defines the rectangle
                        RED, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if((Cb_val < 130)&&(Cr_val < 130)) // Was it from region 3?
            {
                position = "GREEN"; // Record our analysis
*/
                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
  /*              Imgproc.rectangle(
                        input, // Buffer to draw on
                        region_pointA, // First point which defines the rectangle
                        region_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
*/
            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
           // return input;
        //}

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
       /* public String getAnalysis()
        {
            return position;
        }
        public int getCr()
        {
            return Cr_val;
        }
        public int getCb()
        {
            return Cb_val;
        }
    }

}*/