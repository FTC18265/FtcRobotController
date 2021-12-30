package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.ArmController;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Autonomous(name = "AutonomousPeriod")
public class freightFrenzyAuto extends LinearOpMode {
    BNO055IMU imu;

    static final double     COUNTS_PER_MOTOR_REV    = 28;
    static final double     DRIVE_GEAR_REDUCTION    = 19.2;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415) * 0.9;


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.07;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = -0.07;     // Larger is more responsive, but also less stable
    static final double     P_HORIZONTAL_COEFF      = -0.01;     // Larger is more responsive, but also less stable


    private DcMotor topright;
    private DcMotor topleft;
    private DcMotor bottomright;
    private DcMotor bottomleft;
    private DcMotorEx arm;
    private DcMotorEx susan;
    private DcMotor intake;
    private Servo door;
    private DcMotor carousel;
    private Rev2mDistanceSensor extradistancesensor;
    private Rev2mDistanceSensor distancesensor;
    private AnalogInput potentiometer;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            " ASxWKxX/////AAABmSF4eat8sUkxvdtWRwyP9DY+VoUtavSfblX3C9SsIUO3jHJ+WdvoYKEmByzo86qe/Mg4M0Xk0XIUtrYmNyErHwILvJJxOveLX2A2UAE3jO4yeYNszMt3GKQQEbv/QkiTHQLvN/uqrXBlBowX1nDOf7HeOCSIgjyZHmX5AuPIZw4TLMZ6xs+u8gup23vhSJjPROnD9Cr6+mJpwxIhpcLDtUtNL0IQdVDTPMxEoipVDIsnLuf3vCCcV/jIK5I6a2R8sPpDaGU0v4p0r+yDVAGH6TWt5pwRuV5a5GLyhjkwih7bNyIUfWCo4bNKvPXzV4wSK6DeNxtxgHquj2xLSkyG2o+4HGMdHXEsuekdjVJtYYDu ";

    private int level = 0;
    private int degree = 1;
    private double lasttime;
    private double currenttime;
    private int lastDegree;

    private String button;

    //var
    public static final double NEW_P = 1.5;
    public static final double NEW_I = 0.5;
    public static final double NEW_D = 0.0;
    public static final double NEW_F = 12.6;

    public static final double turning_NEW_P = 1.2;
    public static final double turning_NEW_I = 0.5;
    public static final double turning_NEW_D = 0.0;
    public static final double turning_NEW_F = 17;


    @Override
    public void runOpMode() {

        topright = hardwareMap.get(DcMotor.class, "topright");
        topleft = hardwareMap.get(DcMotor.class, "topleft");
        bottomright = hardwareMap.get(DcMotor.class, "bottomright");
        bottomleft = hardwareMap.get(DcMotor.class, "bottomleft");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        susan = hardwareMap.get(DcMotorEx.class, "susan");
        intake = hardwareMap.get(DcMotor.class, "intake");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        door = hardwareMap.get(Servo.class, "door");
        extradistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "extradistancesensor");
        distancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "distancesensor");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        ArmController armController = new ArmController(arm);
        armController.init();

        SusanController susanController = new SusanController(susan);
        susanController.init();

        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16/9);
        }

        topleft.setDirection(DcMotorSimple.Direction.FORWARD);
        topright.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomleft.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomright.setDirection(DcMotorSimple.Direction.REVERSE);

        door.setPosition(0);
        carousel.setPower(0);

        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients turningPidModified = susan.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        int position = 0;


        arm.setPower(0.5);
        susan.setPower(0.5);

        update();
        telemetry.addData(">", "start detecting");
        telemetry.update();
        /*
        while(!opModeIsActive()){
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.addData("position", position);
                    int i = 0;
                    String labelONe = "None";
                    String labelTwo = "None";
                    String labelThree = "None";
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if ((recognition.getLeft() > 140 && recognition.getLabel() == "Marker" && recognition.getLeft() < 300) ||
                                (recognition.getLeft() > 450 && (recognition.getLabel() == "Duck" || recognition.getLabel() == "Cube"))){
                            position = 3;
                        } else
                        if ((recognition.getLeft() > 140 && (recognition.getLabel() == "Duck" ||
                                recognition.getLabel() == "Cube") && recognition.getLeft() < 300) ||
                                (recognition.getLeft() > 450 && recognition.getLabel() == "Marker")){
                            position = 2;
                        }

                        if (i == 0){
                            labelONe = recognition.getLabel();
                        }
                        if (i == 1){
                            labelTwo = recognition.getLabel();
                        }
                        if (i == 2){
                            labelThree = recognition.getLabel();
                        }

                        if (labelONe == "Marker" && labelTwo == "Marker" || labelThree == "Marker"){
                            position = 1;
                            labelONe = "None";
                            labelTwo = "None";
                            labelThree = "None";
                        }
                        i++;
                    }
                    telemetry.update();
                }
            }
        }

         */

///////////////////////////////
        waitForStart();

        //detection
        while(distancesensor.getDistance(DistanceUnit.CM) < 15){
            forward(0.3);
        }
        stopMove();

        while(position == 0){
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.addData("position", position);
                    int i = 0;
                    String labelONe = "None";
                    String labelTwo = "None";
                    String labelThree = "None";
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if ((recognition.getLeft() > 450 && (recognition.getLabel() == "Duck" || recognition.getLabel() == "Cube"))){
                            position = 3;
                        } else
                        if ((recognition.getLeft() > 140 && (recognition.getLabel() == "Duck" ||
                                recognition.getLabel() == "Cube") && recognition.getLeft() < 300) ||
                                (recognition.getLeft() > 450 && recognition.getLabel() == "Marker")){
                            position = 2;
                        }

                        if (i == 0){
                            labelONe = recognition.getLabel();
                        }
                        if (i == 1){
                            labelTwo = recognition.getLabel();
                        }
                        if (i == 2){
                            labelThree = recognition.getLabel();
                        }

                        if (labelONe == "Marker" && labelTwo == "Marker" || labelThree == "Marker"){
                            position = 1;
                            labelONe = "None";
                            labelTwo = "None";
                            labelThree = "None";
                        }
                        i++;
                    }
                    telemetry.addData("detected position", position);
                    telemetry.update();
                }
            }
        }

        gyroTurn(0.3, 45);


/*
        //start program
        armController.autoLevel(3);
        sleep(3000);

        susanController.autoLevel(0);
        sleep(5000);

        armController.autoLevel(position);
        sleep(1000);

        update();

        if(position == 1){
            while(distancesensor.getDistance(DistanceUnit.CM) < 34.3){
                forward();
            }
            stopMove();
        }
        else if (position == 2){
            while ((distancesensor.getDistance(DistanceUnit.CM)) < 42.1){
                forward();
            }
            stopMove();
        }
        else if (position == 3){
            while ((distancesensor.getDistance(DistanceUnit.CM)) < 45.5){
                forward();
            }
            stopMove();

            door.setPosition(1);
            update();
            sleep(3000);
            susanController.autoLevel(-1);
            door.setPosition(0);
        }


 */


        sleep(10000);


    }

/////////////////methods
    public void pullBack(){
        backward(0.3);

        sleep(1000);

        stopMove();
    }

    public void forward(double power){
        topleft.setPower(power);
        topright.setPower(power);
        bottomright.setPower(power);
        bottomleft.setPower(power);
    }

    public void backward(double power){
        topleft.setPower(-power);
        topright.setPower(power);
        bottomright.setPower(-power);
        bottomleft.setPower(power);
    }

    public void left(int distance){

    }

    public void right(double power){
        topleft.setPower(power);
        topright.setPower(power);
        bottomright.setPower(power);
        bottomleft.setPower(power);
    }

    public void stopMove(){
        topleft.setPower(0);
        topright.setPower(0);
        bottomright.setPower(0);
        bottomleft.setPower(0);
    }

    public void update(){
        telemetry.addData("distance", distancesensor.getDistance(DistanceUnit.CM));
        telemetry.addData("arm", arm.getCurrentPosition());
        telemetry.addData("susan", susan.getCurrentPosition());

        telemetry.update();
    }

    public void setMode(DcMotor.RunMode mode){
        topleft.setMode(mode);
        topright.setMode(mode);
        bottomright.setMode(mode);
        bottomleft.setMode(mode);
    }

    public void setPower(double power) {
        topleft.setPower(power);
        topright.setPower(power);
        bottomright.setPower(power);
        bottomleft.setPower(power);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("heading", getAngle());
            telemetry.update();


        }
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int newTopRightTarget;
        int newBottomRightTarget;
        int newTopleftTarget;
        int newBottomLeftTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  topLeftSpeed;
        double  topRightSpeed;
        double  bottomLeftSpeed;
        double  bottomRightSpeed;

        // Determine new target position, and pass to motor controller
        moveCounts = (int)(distance * COUNTS_PER_INCH);
        newTopleftTarget = topleft.getCurrentPosition() + moveCounts;
        newTopRightTarget = topright.getCurrentPosition() + moveCounts;
        newBottomLeftTarget = bottomleft.getCurrentPosition() + moveCounts;
        newBottomRightTarget = bottomright.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        topleft.setTargetPosition(newTopleftTarget);
        topright.setTargetPosition(newTopRightTarget);
        bottomleft.setTargetPosition(newBottomLeftTarget);
        bottomright.setTargetPosition(newBottomRightTarget);

        topleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1);
        topleft.setPower(speed);
        topright.setPower(speed);
        bottomleft.setPower(speed);
        bottomright.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while (topleft.isBusy() && topright.isBusy() &&
                bottomleft.isBusy() && bottomright.isBusy()) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            topLeftSpeed = speed - steer;
            topRightSpeed = speed + steer;
            bottomLeftSpeed = speed - steer;
            bottomRightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(topLeftSpeed), Math.abs(topRightSpeed));
            if (max > 1.0)
            {
                topLeftSpeed /= max;
                topRightSpeed /= max;
            }
            max = Math.max(Math.abs(bottomLeftSpeed), Math.abs(bottomRightSpeed));
            if (max > 1.0)
            {
                bottomLeftSpeed /= max;
                bottomRightSpeed /= max;
            }

            topleft.setPower(topLeftSpeed);
            topright.setPower(topRightSpeed);
            bottomleft.setPower(bottomLeftSpeed);
            bottomright.setPower(bottomRightSpeed);

            // Display drive status for the driver.
            telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            telemetry.addData("Target",  "%7d:%7d",      newTopleftTarget,  newTopRightTarget,
                    newBottomLeftTarget, newBottomRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      topleft.getCurrentPosition(),
                    topright.getCurrentPosition(), bottomleft.getCurrentPosition(),
                    bottomright.getCurrentPosition());
            telemetry.addData("Speed",   "%5.2f:%5.2f",  topLeftSpeed, topRightSpeed,
                    bottomLeftSpeed, bottomRightSpeed);
            telemetry.update();
        }

        // Stop all motion;
        stopAllMotors();

        // Turn off RUN_TO_POSITION
        topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void gyroHorizontal( double speed,
                                double distance,
                                double angle) {
        int newTopRightTarget;
        int newBottomRightTarget;
        int newTopleftTarget;
        int newBottomLeftTarget;
        double  max;
        double  error;
        double  steer;
        double  topLeftSpeed;
        double  topRightSpeed;
        double  bottomLeftSpeed;
        double  bottomRightSpeed;
        int moveCounts;
        moveCounts = (int) (distance * COUNTS_PER_INCH * 1.4);

        if (distance > 0) {
            topleft.setDirection(DcMotorSimple.Direction.FORWARD);
            topright.setDirection(DcMotorSimple.Direction.FORWARD);
            bottomleft.setDirection(DcMotorSimple.Direction.REVERSE);
            bottomright.setDirection(DcMotorSimple.Direction.REVERSE);
//            topright.setDirection(DcMotorSimple.Direction.REVERSE);
//            bottomleft.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (distance < 0) {
            topleft.setDirection(DcMotorSimple.Direction.REVERSE);
            topright.setDirection(DcMotorSimple.Direction.REVERSE);
            bottomleft.setDirection(DcMotorSimple.Direction.FORWARD);
            bottomright.setDirection(DcMotorSimple.Direction.FORWARD);
//            topleft.setDirection(DcMotorSimple.Direction.REVERSE);
//            bottomright.setDirection(DcMotorSimple.Direction.REVERSE);
        }

//        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        if (distance > 0) {
//            newTopleftTarget = (topleft.getCurrentPosition() + moveCounts) * 0.8;
//            newTopRightTarget = (topright.getCurrentPosition() + moveCounts) * 1;
//            newBottomLeftTarget = (bottomleft.getCurrentPosition() + moveCounts) * 0.8;
//            newBottomRightTarget = (bottomright.getCurrentPosition() + moveCounts) * 1;
//        } else {
//            newTopleftTarget = (topleft.getCurrentPosition() + moveCounts) * 1;
//            newTopRightTarget = (topright.getCurrentPosition() + moveCounts) * 0.8;
//            newBottomLeftTarget = (bottomleft.getCurrentPosition() + moveCounts) * 1;
//            newBottomRightTarget = (bottomright.getCurrentPosition() + moveCounts) * 0.8;
//        }


        newTopleftTarget = (topleft.getCurrentPosition() + moveCounts);
        newTopRightTarget = (topright.getCurrentPosition() + moveCounts);
        newBottomLeftTarget = (bottomleft.getCurrentPosition() + moveCounts);
        newBottomRightTarget = (bottomright.getCurrentPosition() + moveCounts);

        topleft.setTargetPosition((int) newTopleftTarget);
        topright.setTargetPosition((int) newTopRightTarget);
        bottomleft.setTargetPosition((int) newBottomLeftTarget);
        bottomright.setTargetPosition((int) newBottomRightTarget);

        topleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        topleft.setPower(speed);
        topright.setPower(speed);
        bottomleft.setPower(speed);
        bottomright.setPower(speed);

        while (topleft.isBusy() && topright.isBusy() &&
                bottomleft.isBusy() && bottomright.isBusy()) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_HORIZONTAL_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
//            if (distance > 0) {
//                topLeftSpeed = speed - steer;
//                topRightSpeed = speed - steer;
//                bottomLeftSpeed = speed + steer;
//                bottomRightSpeed = speed + steer;
//            } else {
//                topLeftSpeed = speed + steer;
//                topRightSpeed = speed + steer;
//                bottomLeftSpeed = speed - steer;
//                bottomRightSpeed = speed - steer;
//            }
            if (distance < 0)
                steer *= -1.0;

            topLeftSpeed = speed + steer;
            topRightSpeed = speed - steer;
            bottomLeftSpeed = speed + steer;
            bottomRightSpeed = speed - steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(topLeftSpeed), Math.abs(topRightSpeed));
            if (max > 1.0)
            {
                topLeftSpeed /= max;
                topRightSpeed /= max;
            }
            max = Math.max(Math.abs(bottomLeftSpeed), Math.abs(bottomRightSpeed));
            if (max > 1.0)
            {
                bottomLeftSpeed /= max;
                bottomRightSpeed /= max;
            }

            topleft.setPower(topLeftSpeed);
            topright.setPower(topRightSpeed);
            bottomleft.setPower(bottomLeftSpeed);
            bottomright.setPower(bottomRightSpeed);

            // Display drive status for the driver.
            telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            telemetry.addData("Target",  "%7d:%7d",      newTopleftTarget,  newTopRightTarget,
                    newBottomLeftTarget, newBottomRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      topleft.getCurrentPosition(),
                    topright.getCurrentPosition(), bottomleft.getCurrentPosition(),
                    bottomright.getCurrentPosition());
            telemetry.addData("Speed",   "%5.2f:%5.2f",  topLeftSpeed, topRightSpeed,
                    bottomLeftSpeed, bottomRightSpeed);
            telemetry.update();
        }

        stopAllMotors();

        topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("targetposition", newBottomLeftTarget);
        telemetry.update();
    }

    private void stopAllMotors() {
        topleft.setPower(0);
        bottomleft.setPower(0);
        topright.setPower(0);
        bottomright.setPower(0);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            leftSpeed  = speed * steer;
            rightSpeed   = -leftSpeed;
        }

        // Send desired speeds to motors.

        topleft.setPower(leftSpeed);
        topright.setPower(rightSpeed);
        bottomleft.setPower(leftSpeed);
        bottomright.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;
        double heading;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        //return Range.clip(error * PCoeff, -1, 1);
        if ((error * PCoeff < 0.2) && (error * PCoeff > P_TURN_COEFF * HEADING_THRESHOLD)) return 0.3;
        if ((error * PCoeff > -0.2) && (error * PCoeff < -(P_TURN_COEFF * HEADING_THRESHOLD))) return -0.3;

        if (error * PCoeff < -0.75) return -0.75;
        if (error * PCoeff > 0.75) return 0.75;

        return error * PCoeff;
    }

    private double getAngle()
    {
        Orientation angles;
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }


}
