package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class mechanumDrive extends LinearOpMode {

    private DcMotor topright;
    private DcMotor topleft;
    private DcMotor conveyor;
    private Servo pusher;
    private Servo claw;
    private Servo arm;
    private DcMotor shooterleft;
    private DcMotor shooterright;
    private DcMotor intake;
    private DcMotor bottomright;
    private DcMotor bottomleft;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        boolean intake2;

        topright = hardwareMap.get(DcMotor.class, "topright");
        topleft = hardwareMap.get(DcMotor.class, "topleft");
        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        pusher = hardwareMap.get(Servo.class, "pusher");
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");
        shooterleft = hardwareMap.get(DcMotor.class, "shooter-left");
        shooterright = hardwareMap.get(DcMotor.class, "shooter-right");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bottomright = hardwareMap.get(DcMotor.class, "bottomright");
        bottomleft = hardwareMap.get(DcMotor.class, "bottomleft");

        // Put initialization blocks here.
        topright.setDirection(DcMotorSimple.Direction.REVERSE);
        topleft.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterleft.setDirection(DcMotorSimple.Direction.REVERSE);
        pusher.setPosition(0);
        claw.setPosition(0);
        arm.setPosition(0);

        shooterleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterleft.setPower(0.42);
        shooterright.setPower(0.42);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake2 = false;

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.dpad_up){

                }
                if (gamepad1.dpad_down){

                }
                if (gamepad1.y) {
                    intake2 = true;
                }
                if (gamepad1.x) {
                    intake2 = false;
                }
                if (intake2 == true) {
                    intake.setPower(1);
                    conveyor.setPower(0.8);
                } else {
                    intake.setPower(0);
                    conveyor.setPower(0);
                }
                if (gamepad1.left_bumper) {
                    claw.setPosition(1);
                } else {
                    claw.setPosition(0);
                }
                if (gamepad1.right_bumper) {
                    arm.setPosition(1);
                } else {
                    arm.setPosition(0);
                }
                if (gamepad1.a) {
                    pusher.setPosition(1);
                } else {
                    pusher.setPosition(0);
                }
                topright.setPower(0.5 * (gamepad1.left_stick_x + gamepad1.right_stick_x + gamepad1.right_stick_y));
                bottomright.setPower(0.5 * (-gamepad1.left_stick_x + (gamepad1.right_stick_x - gamepad1.right_stick_y)));
                topleft.setPower(0.5 * (gamepad1.left_stick_x + (gamepad1.right_stick_x - gamepad1.right_stick_y)));
                bottomleft.setPower(0.5 * (-gamepad1.left_stick_x + gamepad1.right_stick_x + gamepad1.right_stick_y));
                telemetry.update();
            }
        }
    }
}
