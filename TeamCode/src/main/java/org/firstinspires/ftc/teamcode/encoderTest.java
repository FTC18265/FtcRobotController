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
    private ColorSensor colorsensor;

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
        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");

        telemetry.update();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(0);

        topleft.setDirection(DcMotorSimple.Direction.FORWARD);
        topright.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomleft.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomright.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        int rotation = 0;
        int remaining = 0;


        waitForStart();

//        while(opModeIsActive()){
//            telemetry.addData("distance sensor", distancesensor.getDistance(DistanceUnit.CM));
//            telemetry.update();
//        }
while(opModeIsActive()) {
    telemetry.addData("colorsensor", colorsensor.red());
    telemetry.update();
}

//        topleft.setTargetPosition(1000);
//        topright.setTargetPosition(-1000);
//        bottomleft.setTargetPosition(-1000);
//        bottomright.setTargetPosition(1000);
//
//        topleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        topright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bottomleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bottomright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        topleft.setPower(0.3);
//        topright.setPower(-0.3);
//        bottomleft.setPower(-0.3);
//        bottomright.setPower(0.3);
//
//        while (topleft.isBusy() && topright.isBusy() &&
//                bottomleft.isBusy() && bottomright.isBusy()) {
//            telemetry.addData("topright", topright.getCurrentPosition());
//            telemetry.addData("topleft", topleft.getCurrentPosition());
//            telemetry.addData("bottomright", bottomright.getCurrentPosition());
//            telemetry.addData("bottomleft", bottomleft.getCurrentPosition());
//
//            telemetry.update();
//        }
//
//        sleep(2000);
//
//        topleft.setPower(0);
//        topright.setPower(0);
//        bottomleft.setPower(0);
//        bottomright.setPower(0);

//        while(opModeIsActive()){
//            bottomright.setPower
//                    (0.5 * (gamepad1.left_stick_x + (gamepad1.right_stick_x - gamepad1.right_stick_y)));
//            topright.setPower
//                    (0.5 * (gamepad1.left_stick_x + (-gamepad1.right_stick_x - gamepad1.right_stick_y)));
//            bottomleft.setPower
//                    (0.5 * (-gamepad1.left_stick_x + (-gamepad1.right_stick_x - gamepad1.right_stick_y)));
//            topleft.setPower
//                    (0.5 * (-gamepad1.left_stick_x + (gamepad1.right_stick_x - gamepad1.right_stick_y)));
//
//            telemetry.addData("topright", topright.getCurrentPosition());
//            telemetry.addData("topleft", topleft.getCurrentPosition());
//            telemetry.addData("bottomright", bottomright.getCurrentPosition());
//            telemetry.addData("bottomleft", bottomleft.getCurrentPosition());
//
//            telemetry.update();

//        }


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
