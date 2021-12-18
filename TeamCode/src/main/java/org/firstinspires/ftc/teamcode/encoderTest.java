package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

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

    private AnalogInput potentiometer;

    private ColorSensor colorsensor;
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
        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");

        telemetry.addData("colorsensor", colorsensor.blue());
        telemetry.update();

        susan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


        while (opModeIsActive()) {
            if(gamepad1.a) {
                susan.setPower(0.15);
            }

            if(gamepad1.b){
                susan.setPower(-0.15);
            }

            if(gamepad1.x){
                susan.setPower(0);
            }
            telemetry.addData("colorsensor", colorsensor.blue());
            telemetry.update();

        }

    }

}
