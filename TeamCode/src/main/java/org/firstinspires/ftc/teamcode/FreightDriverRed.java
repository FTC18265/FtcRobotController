package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (group = "red", name = "Driver Red")
public class FreightDriverRed extends LinearOpMode {
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
    private double lasttime = 0;
    private double currenttime = 0;
    private int lastDegree = 0;
    private boolean secureFreight = false;
    private double secureFreightStartTime = 0;
    private boolean releaseFreight = false;
    private double releaseFreightStartTime = 0;

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

    DriverWrap driverWrap = new DriverWrap("red");

    @Override
    public void runOpMode(){
        driverWrap.init(hardwareMap, "red");


        waitForStart();

        while(opModeIsActive()){
            currenttime = getRuntime();
            driverWrap.run(gamepad1, gamepad2, currenttime);
        }
    }

}
