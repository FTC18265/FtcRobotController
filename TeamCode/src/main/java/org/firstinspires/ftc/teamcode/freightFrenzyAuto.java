package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.ArmController;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Autonomous(name = "AutonomousPeriod")
public class freightFrenzyAuto extends LinearOpMode {

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

        bottomright.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomleft.setDirection(DcMotorSimple.Direction.REVERSE);

        door.setPosition(0);
        carousel.setPower(0);

        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients turningPidModified = susan.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        int placement = 3;
        int position = 1;


        arm.setPower(0.5);
        susan.setPower(0.5);

        update();
        telemetry.addData(">", "start detecting");
        telemetry.update();
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
                                (recognition.getLeft() > 450 && recognition.getLabel() == "Duck")){
                            position = 3;
                        } else
                        if ((recognition.getLeft() > 140 && recognition.getLabel() == "Duck" && recognition.getLeft() < 300) ||
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

///////////////////////////////
        waitForStart();

        //start program
        armController.autoLevel(3);
        sleep(3000);



        susanController.autoLevel(0);
        sleep(5000);

        armController.autoLevel(placement);
        sleep(1000);

        update();

        if(placement == 1){
            while(distancesensor.getDistance(DistanceUnit.CM) < 34.3){
                forward();
            }
            stopMove();
        }
        else if (placement == 2){
            while ((distancesensor.getDistance(DistanceUnit.CM)) < 42.1){
                forward();
            }
            stopMove();
        }
        else if (placement == 3){
            while ((distancesensor.getDistance(DistanceUnit.CM)) < 45.5){
                forward();
            }
            stopMove();
        }

        door.setPosition(1);
        update();
        sleep(3000);
        susanController.autoLevel(-1);
        door.setPosition(0);

        sleep(10000);


    }

/////////////////methods
    public void pullBack(){
        backward();

        sleep(1000);

        stopMove();
    }

    public void forward(){
        topleft.setPower(0.3);
        topright.setPower(-0.3);
        bottomright.setPower(0.3);
        bottomleft.setPower(-0.3);
    }

    public void backward(){
        topleft.setPower(-0.3);
        topright.setPower(0.3);
        bottomright.setPower(-0.3);
        bottomleft.setPower(0.3);
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
}
