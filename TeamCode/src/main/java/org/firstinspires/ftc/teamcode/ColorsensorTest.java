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


@Autonomous (name = "Colorsensor Test")
public class ColorsensorTest extends LinearOpMode {
    private DcMotor topright;
    private DcMotor topleft;
    private DcMotor bottomright;
    private DcMotor bottomleft;
    private ColorSensor colorsensor;

    //change based on height of color sensor
    private int color = 2000;

    @Override
    public void runOpMode() {
        topright = hardwareMap.get(DcMotor.class, "topright");
        topleft = hardwareMap.get(DcMotor.class, "topleft");
        bottomright = hardwareMap.get(DcMotor.class, "bottomright");
        bottomleft = hardwareMap.get(DcMotor.class, "bottomleft");

        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");

        MecanumController mecanumController = new MecanumController(topleft, topright, bottomleft, bottomright);
        mecanumController.init();

///////////////////////////////
        waitForStart();

        mecanumController.forward(0.25);
        while(opModeIsActive() && colorsensor.blue() < color ){
            telemetry.addData("colorsensor", colorsensor.blue());
            telemetry.update();
        }

        mecanumController.stopAllMotors();

        telemetry.addLine("finished");
        telemetry.update();

        sleep(5000);
    }

}
