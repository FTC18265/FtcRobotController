package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "encoderTest")
public class encoderTest extends LinearOpMode {

    private DcMotor topright;
    private DcMotor topleft;
    private DcMotor bottomright;
    private DcMotor bottomleft;
    private DcMotorEx arm;
    private DcMotor susan;
    private DcMotor intake;
    private Servo door;
    private DcMotor carousel;
    private Rev2mDistanceSensor distancesensor;

    private AnalogInput potentiometer;

    private int level = 0;

    private double lasttime;
    private double currenttime;

    private String button;

    public static final double NEW_P = 1.5;
    public static final double NEW_I = 0.5;
    public static final double NEW_D = 0.0;
    public static final double NEW_F = 12.6;

    @Override
    public void runOpMode() {

        topright = hardwareMap.get(DcMotor.class, "topright");
        topleft = hardwareMap.get(DcMotor.class, "topleft");
        bottomright = hardwareMap.get(DcMotor.class, "bottomright");
        bottomleft = hardwareMap.get(DcMotor.class, "bottomleft");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        susan = hardwareMap.get(DcMotor.class, "susan");
        intake = hardwareMap.get(DcMotor.class, "intake");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        door = hardwareMap.get(Servo.class, "door");
        distancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "distancesensor");

        telemetry.update();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(0);

        bottomright.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomleft.setDirection(DcMotorSimple.Direction.REVERSE);

        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        topleft.setPower(0.3);
        topright.setPower(0.3);
        bottomleft.setPower(0.3);
        bottomright.setPower(0.3);

        sleep(10000);

//        while (opModeIsActive()) {
//            telemetry.addData("distance", distancesensor.getDistance(DistanceUnit.CM));
//            telemetry.addData("arm", arm.getCurrentPosition());
//            telemetry.addData("susan", susan.getCurrentPosition());
//
//
//            telemetry.update();
///*
//            if(gamepad1.x){
//                while(distancesensor.getDistance(DistanceUnit.CM) > 27){
//                    topleft.setPower(-0.25);
//                    topright.setPower(0.25);
//                    bottomright.setPower(-0.25);
//                    bottomleft.setPower(0.25);
//                }
//                topleft.setPower(0.3);
//                topright.setPower(0.25);
//                bottomright.setPower(-0.25);
//                bottomleft.setPower(-0.3);
//                sleep(100);
//
//                topleft.setPower(0);
//                topright.setPower(0.25);
//                bottomright.setPower(-0.25);
//                bottomleft.setPower(0);
//
//                carousel.setPower(-0.5);
//            }
//
// */
//
//        }

    }

}
