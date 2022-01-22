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

    public static final double NEW_P = 1.5;
    public static final double NEW_I = 0.5;
    public static final double NEW_D = 0.0;
    public static final double NEW_F = 12.6;

    public static final double turning_NEW_P = 1.2;
    public static final double turning_NEW_I = 0.5;
    public static final double turning_NEW_D = 0.0;
    public static final double turning_NEW_F = 17;

    @Override
    public void runOpMode(){
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        susan = hardwareMap.get(DcMotorEx.class, "susan");
        intake = hardwareMap.get(DcMotor.class, "intake");
        door = hardwareMap.get(Servo.class, "door");
        extradistancesensor = hardwareMap.get(DistanceSensor.class, "extradistancesensor");

        ArmController armController = new ArmController(arm);
        armController.init();

        SusanController susanController = new SusanController(susan, "red");
        susanController.init();

        arm.setPower(0.5);
        susan.setPower(0.5);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        armController.autoLevel(1);
        while(arm.isBusy()){
            telemetry.addData("arm", armController.getCurrentPosition());
            telemetry.update();
            intake.setPower(-0.1);
        }
        intake.setPower(-0.45);
        while( extradistancesensor.getDistance(DistanceUnit.CM) < 12.5){
            telemetry.addLine("object detected");
            telemetry.update();
        }
        sleep(500);
        telemetry.addLine("object not detected");
        telemetry.update();

        intake.setPower(0);
        sleep(5000);
//        intake.setPower(0);
//        if(extradistancesensor.getDistance(DistanceUnit.CM) < 12.5){
//            telemetry.addLine("object detected");
//        }else{
//            telemetry.addLine("no object");
//        }
    }

}
