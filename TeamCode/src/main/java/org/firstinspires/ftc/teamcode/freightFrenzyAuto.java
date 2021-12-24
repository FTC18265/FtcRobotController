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
import org.firstinspires.ftc.teamcode.ArmController;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

        int placement = 2;


        arm.setPower(0.5);
        susan.setPower(0.5);

        waitForStart();

        //start program
        armController.autoLevel(3);
        sleep(1000);
        susanController.autoLevel(0);
        sleep(5000);
        armController.autoLevel(placement);
        sleep(3000);

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
            while ((distancesensor.getDistance(DistanceUnit.CM)) < 45){
                forward();
            }
            stopMove();
        }

        door.setPosition(1);
        update();
        sleep(3000);
        door.setPosition(0);
        sleep(3000);
        susanController.autoLevel(-1);

        sleep(10000);
    }


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
        telemetry.update();
    }

}
