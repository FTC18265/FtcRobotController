package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "armTest")
public class armTest extends LinearOpMode {
    private DcMotorEx arm;
    private DcMotorEx susan;
    private DcMotor intake;
    private Servo door;

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

        ArmController armController = new ArmController(arm);
        armController.init();

        SusanController susanController = new SusanController(susan, "red");
        susanController.init();

        arm.setPower(0.5);
        susan.setPower(0.5);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        armController.autoLevel(3);
        while(arm.isBusy()){
            telemetry.addData("arm", armController.getCurrentPosition());
            telemetry.update();
            intake.setPower(-0.1);
        }
        intake.setPower(0);
        sleep(1000);
        susanController.autoLevel(0);
        sleep(2500);


        door.setPosition(1);
        sleep(1500);
            intake.setPower(0.1);

        sleep(2000);
        intake.setPower(0);

        susanController.autoLevel(-1);
        door.setPosition(0);

        sleep(10000);
    }

}
