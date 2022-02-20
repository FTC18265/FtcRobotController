package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import android.util.Log;


@Autonomous (name = "Detection Auto Blue")
public class DetectionAutoBlue extends LinearOpMode {
    private static final String TAG = "Hebe";

    private DcMotor topright;
    private DcMotor topleft;
    private DcMotor bottomright;
    private DcMotor bottomleft;
    private DcMotorEx arm;
    private DcMotorEx susan;
    private DcMotor intake;
    private Servo door;
    private DcMotor carousel;
    private Rev2mDistanceSensor distancesensor;
    private AnalogInput potentiometer;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private BNO055IMU imu;
    private ColorSensor colorsensor;
    private GyroController gyroController;
    private ArmController armController;
    private SusanController susanController;

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
    private int currenttime = 0;
    private int detectionStartTime = 0;
    private int detectionResult = 1;
    List<Recognition> updatedRecognitions;

    //change based on height of color sensor
    private int color = 2000;

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
        distancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "distancesensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");

        gyroController = new GyroController(topleft, topright, bottomleft, bottomright, imu);
        gyroController.init();

        armController = new ArmController(arm);
        armController.init();

        susanController = new SusanController(susan, "red");
        susanController.init();

        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients turningPidModified = susan.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        door.setPosition(0.5);
        carousel.setPower(0);
        arm.setPower(0.5);
        susan.setPower(0.5);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16/9);
        }

        //left 100
        //middle 350
        //right 600
        while(!opModeIsActive()){
            if (tfod != null) {
                updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {

                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.addData("position", detectionResult);
                    int i = 0;

                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

//                        if ((recognition.getRight() > 500 && (recognition.getLabel() == "Duck" ||
//                                recognition.getLabel() == "Cube"))){
//                            detectionResult = 2;
//                        } else
//                        if ((recognition.getRight() > 200 && (recognition.getLabel() == "Duck" ||
//                                recognition.getLabel() == "Cube") && recognition.getRight() < 500)){
//                            detectionResult = 1;
//                        } else{
//                            detectionResult = 3;
//                        }

//                        if ((recognition.getRight() > 500 && (recognition.getLabel() == "Duck" ||
//                                recognition.getLabel() == "Cube"))){
//                            detectionResult = 3;
//                        } else
//                        if ((recognition.getRight() > 200 && (recognition.getLabel() == "Duck" ||
//                                recognition.getLabel() == "Cube") && recognition.getRight() < 500)){
//                            detectionResult = 2;
//                        }else
//                        if ((recognition.getRight() < 200 && (recognition.getLabel() == "Duck" ||
//                                recognition.getLabel() == "Cube"))){
//                            detectionResult = 1;
//                        }
                        boolean middle = false;
                        boolean left = false;
                        for(int x = 0; x < updatedRecognitions.size(); x++){

                            if(updatedRecognitions.get(x).getLabel() == "Duck" || updatedRecognitions.get(x).getLabel() == "Cube"){
                                Log.i(TAG, "object detected");

                                if ((updatedRecognitions.get(x).getRight() > 400 )){
                                    detectionResult = 2;
                                    middle = false;
                                    Log.i(TAG, "detect middle" );
                                } else if (updatedRecognitions.get(x).getRight() < 400){
                                    Log.i(TAG, "detect left" );
                                    left = false;
                                    detectionResult = 1;
                                }
                            } else if (updatedRecognitions.get(x).getLabel() == "Marker"){
                                if ((updatedRecognitions.get(x).getRight() > 400 )){
                                    middle = true;
                                } else if (updatedRecognitions.get(x).getRight() < 400){
                                    left = true;
                                }
                            }
                        }
                        if(middle == true && left == true){
                            detectionResult = 3;
                        }
                        telemetry.addData("detected position", detectionResult);
                        telemetry.update();
                        i++;
                    }

                }
            }
        }
/////////////////////////////////////////////////////////////////////
        waitForStart();
        Log.i(TAG, "start");
//        if(updatedRecognitions != null){
//            Log.i(TAG, "size" + updatedRecognitions.size());
//            for(int i = 0; i < updatedRecognitions.size(); i++){
//                Log.i(TAG, updatedRecognitions.get(i).getLabel());
//                if(updatedRecognitions.get(i).getLabel() == "Duck" || updatedRecognitions.get(i).getLabel() == "Cube"){
//                    Log.i(TAG, "object detected");
//
//                    if ((updatedRecognitions.get(i).getRight() > 400 )){
//                        detectionResult = 2;
//                        Log.i(TAG, "detect middle" );
//                    } else if (updatedRecognitions.get(i).getRight() < 400){
//                        Log.i(TAG, "detect left" );
//                        detectionResult = 1;
//                    }
//                }
//            }
//        }


        telemetry.addData("position", detectionResult);
        telemetry.update();
        sleep(10000);

        intake.setPower(0.1);

        armController.autoLevel(detectionResult);
        susanController.autoLevel(0);
        sleep(3000);

        gyroController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive() && distancesensor.getDistance(DistanceUnit.CM) < 20){
            gyroController.setPower(0.5);
        }
        gyroController.setPower(0);
        gyroController.gyroTurn(0.3, 37);

        //arm position
        if(detectionResult == 1){
            level1();
        } else if (detectionResult == 2){
            level2();
        } else if (detectionResult == 3){
            level3();
        }


        //move to carousel
        susanController.autoLevel(-1);
        armController.autoLevel(3);
        gyroController.gyroTurn(0.3, 60);

        //turn carousel
        while(distancesensor.getDistance(DistanceUnit.CM) > 30 && opModeIsActive()){
            gyroController.setPower(-0.3);
        }
        gyroController.setPower(-0.1);

        carousel.setPower(-0.5);
        sleep(3000);
        carousel.setPower(0);

        //park in storage unit
        gyroController.gyroTurn(0.3, -7);
        gyroController.forward(0.25);
        //line detection
        telemetry.update();
        while(opModeIsActive() && colorsensor.blue() < color ){
            telemetry.addData("colorsensor", colorsensor.blue());
            telemetry.update();
        }

        gyroController.stopAllMotors();
        armController.autoLevel(0);

        telemetry.addLine("finished");
        telemetry.update();

        sleep(5000);
    }

/////////////////////////////////////////////////////////methods

    private void update(){
        telemetry.addData("distance", distancesensor.getDistance(DistanceUnit.CM));
        telemetry.addData("arm", arm.getCurrentPosition());
        telemetry.addData("susan", susan.getCurrentPosition());

        telemetry.update();
    }

    private void level1(){
        //move to hub
        while(opModeIsActive() && distancesensor.getDistance(DistanceUnit.CM) < 58){
            gyroController.setPower(0.3);
        }
        gyroController.setPower(0);

        //drop freight
        intake.setPower(-0.3);
        sleep(2500);
    }

    private void level2(){
        while(opModeIsActive() && distancesensor.getDistance(DistanceUnit.CM) < 60){
            gyroController.setPower(0.3);
        }
        gyroController.setPower(0);

        //drop freight
        intake.setPower(-0.3);
        sleep(2500);

    }
    private void level3(){
        //forward
        gyroController.gyroTurn(0.3, 40);

        while(opModeIsActive() && distancesensor.getDistance(DistanceUnit.CM) < 80){
            gyroController.setPower(0.3);
        }
        gyroController.setPower(0);

        intake.setPower(-0.1);
        sleep(500);
        intake.setPower(0.1);
        door.setPosition(0);
        sleep(1000);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
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
}
