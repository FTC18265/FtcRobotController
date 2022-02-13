package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "armTest")
public class ArmTest extends LinearOpMode {
    private DcMotorEx arm;
    private DcMotorEx susan;
    private DcMotor intake;
    private Servo door;
    private DistanceSensor extradistancesensor;

    @Override
    public void runOpMode(){
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        susan = hardwareMap.get(DcMotorEx.class, "susan");
        intake = hardwareMap.get(DcMotor.class, "intake");
        door = hardwareMap.get(Servo.class, "door");
        extradistancesensor = hardwareMap.get(DistanceSensor.class, "extradistancesensor");

        String button;
        double currenttime = 0;
        double lasttime = 0;

        ArmController armController = new ArmController(arm);
        armController.init();

        SusanController susanController = new SusanController(susan, "blue");
        susanController.init();

        arm.setPower(0.5);
        susan.setPower(0.5);

        door.setPosition(0.5);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){

        }
    }

}
