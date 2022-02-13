package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class DriverWrap {

    private DcMotor topright;
    private DcMotor topleft;
    private DcMotor bottomright;
    private DcMotor bottomleft;
    private DcMotorEx arm;
    private DcMotorEx susan;
    private DcMotor intake;
    private Servo door;
    private Servo shippingArm;
    private DcMotor carousel;
    private Rev2mDistanceSensor extradistancesensor;
    private Rev2mDistanceSensor distancesensor;
    private MecanumController mecanumController;
    private ArmController armController;
    private SusanController susanController;

    private AnalogInput potentiometer;

    private double lasttime = 0;
    private double currenttime = 0;
    private int lastDegree = 0;
    private boolean secureFreight = false;
    private double secureFreightStartTime = 0;
    private boolean releaseFreight = false;
    private double releaseFreightStartTime = 0;
    private double slowValue = 1;
    private String alliance;
    private boolean shippingMode = true;

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

    public DriverWrap (String side){
        alliance = side;
    }


    public void init(HardwareMap hardwareMap, String alliance){
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
        shippingArm = hardwareMap.get(Servo.class, "shippingArm");
        extradistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "extradistancesensor");
        distancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "distancesensor");

        mecanumController = new MecanumController(topleft, topright, bottomleft, bottomright);
        mecanumController.init();

        armController = new ArmController(arm);
        armController.init();

        susanController = new SusanController(susan, alliance);
        susanController.init();

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        door.setPosition(0.5);
        shippingArm.setPosition(1);
        carousel.setPower(0);

        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients turningPidModified = susan.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setPower(0.5);
        susan.setPower(0.5);
    }

    public void run(Gamepad gamepad1, Gamepad gamepad2, double currenttime){
        //slow mode
        slowValue = 1 - gamepad1.left_trigger * 0.65;

        //TFE scoring
        if(gamepad2.right_trigger != 0){
            shippingMode = true;
        }else{
            shippingMode = false;
        }
        shipping(armController.getLevel());

        //drive train
        bottomright.setPower
                (0.7 * (gamepad1.left_stick_x + (gamepad1.right_stick_x - gamepad1.right_stick_y)) * slowValue);
        topright.setPower
                (0.7 * (gamepad1.left_stick_x + (-gamepad1.right_stick_x - gamepad1.right_stick_y)) * slowValue);
        bottomleft.setPower
                (0.7 * (-gamepad1.left_stick_x + (-gamepad1.right_stick_x - gamepad1.right_stick_y)) * slowValue);
        topleft.setPower
                (0.7 * (-gamepad1.left_stick_x + (gamepad1.right_stick_x - gamepad1.right_stick_y)) * slowValue);

        //carousel
        if (gamepad1.b){
            button = "gamepad1.b";
            carousel.setPower(0.575);
        }
        if (gamepad1.x) {
            button = "gamepad1.x";
            carousel.setPower(-0.575);
        }
        if (gamepad1.y) {
            button = "gamepad1.y";
            carousel.setPower(0);
        }

        if(gamepad2.a && currenttime - lasttime > 0.5){
            button = "gamepad2.a";
            lasttime = currenttime;
            armController.adjustLevel(-1);

            armController.setArm(armController.getLevel(), shippingMode);
        }
        if(gamepad2.y && currenttime - lasttime > 0.5){
            lasttime = currenttime;
            button = "gamepad2.y";
            armController.adjustLevel(1);
            //secure freight
//            secureFreight = true;
//            secureFreightStartTime = currenttime;
//            intake.setPower(0.1);
            armController.setArm(armController.getLevel(), shippingMode);
        }

//        if(secureFreight == true && currenttime - secureFreightStartTime > 0.5){
//            intake.setPower(0);
//            secureFreight = false;
//        }


        if (gamepad2.dpad_down && currenttime - lasttime > 0.1) {
            button = "gamepad2.dpad_down";
            armController.microAjust(35);
            lasttime = currenttime;
        }
        if (gamepad2.dpad_up && currenttime - lasttime > 0.1) {
            button = "gamepad2.dpad_up";
            armController.microAjust(-35);
            lasttime = currenttime;
        }

        if(gamepad2.x && currenttime - lasttime > 0.5){
            button = "gamepad2.x";
            lasttime = currenttime;
            susanController.adjustLevel(-1);
        }
        if(gamepad2.b && currenttime - lasttime > 0.5){
            button = "gamepad2.b";
            lasttime = currenttime;
            susanController.adjustLevel(1);
        }


        if (gamepad2.left_bumper && currenttime - lasttime > 0.1) {
            button = "gamepad2.dpad_down";
            susanController.microAjust(35);
            lasttime = currenttime;
        }
        if (gamepad2.right_bumper && currenttime - lasttime > 0.1) {
            button = "gamepad2.dpad_up";
            susanController.microAjust(-35);
            lasttime = currenttime;
        }


        //intake
        if (gamepad1.right_bumper) {
            button = "gamepad1.right_bumper";
            intake.setPower(0.5);
        }
        else if (secureFreight == false && releaseFreight == false){
            intake.setPower(0);
        }

        if(gamepad1.a){
            intake.setPower(-0.5);
        }

        //output
        if (gamepad1.left_bumper) {
            button = "gamepad1.left_bumper";
            if(alliance == "red"){
                door.setPosition(1);
            }
            if(alliance == "blue"){
                door.setPosition(0);
            }

            releaseFreight = true;
            intake.setPower(0.1);
            releaseFreightStartTime = currenttime;
        }
        else{
            door.setPosition(0.5);
//                setArm(0);
        }

        if(releaseFreight == true && currenttime - releaseFreightStartTime > 0.5){
            intake.setPower(0);
            releaseFreight = false;
        }

        /*
            //distance sensor test
            if(extradistancesensor.getDistance(DistanceUnit.CM) < 12.5){
                telemetry.addLine("object detected");
            }else{
                telemetry.addLine("no object");
            }

            if(gamepad1.right_bumper && extradistancesensor.getDistance(DistanceUnit.CM) < 12.5){
                 pullBack();

                 level = 1;
                 setArm(level);

                 intake.setPower(0);
            }

        */
    }

    //TSE arm
    //orgi position - 1
    //level 1 0.5
    //level 2 0.25
    //level 3 0
    public void shipping(int level){
        if(shippingMode == true){
            if(level == 0 ){
                shippingArm.setPosition(0.55);
            }
            if(level == 1 && arm.getCurrentPosition() < -600){
//                arm.setTargetPosition(-775);
                shippingArm.setPosition(0.25);
            }
            if(level == 2 && arm.getCurrentPosition() < -1200){
                shippingArm.setPosition(0);
            }
        }else{
            shippingArm.setPosition(1);
        }
    }

    public int getArmPosition(){
        return armController.getCurrentPosition();
    }

}
