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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


@Autonomous (name = "Detection Auto Blue")
public class DetectionAutoBlue extends LinearOpMode {
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
    private BNO055IMU imu;
    private ColorSensor colorsensor;
    private GyroController gyroController;
    private ArmController armController;
    private SusanController susanController;

    private int level = 0;
    private int degree = 1;
    private int currenttime = 0;
    private int detectionStartTime = 0;
    private int detectionResult = 2;

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
        extradistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "extradistancesensor");
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


        update();
/////////////////////////////////////////////////////////////////////
        waitForStart();
        intake.setPower(0.1);

        armController.autoLevel(detectionResult);
        susanController.autoLevel(0);
        sleep(3000);

        gyroController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(opModeIsActive() && distancesensor.getDistance(DistanceUnit.CM) < 20){
            gyroController.setPower(0.5);
        }
        gyroController.setPower(0);
        gyroController.gyroTurn(0.3, 35);

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
        gyroController.gyroTurn(0.3, 50);

        //turn carousel
        while(distancesensor.getDistance(DistanceUnit.CM) > 30 && opModeIsActive()){
            gyroController.setPower(-0.3);
        }
        gyroController.setPower(-0.1);

        carousel.setPower(-0.5);
        sleep(2500);
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
        while(opModeIsActive() && distancesensor.getDistance(DistanceUnit.CM) < 55){
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
        while(opModeIsActive() && distancesensor.getDistance(DistanceUnit.CM) < 75){
            gyroController.setPower(0.3);
        }
        gyroController.setPower(0);

        intake.setPower(-0.1);
        sleep(500);
        intake.setPower(0.1);
        door.setPosition(0);
    }
}
