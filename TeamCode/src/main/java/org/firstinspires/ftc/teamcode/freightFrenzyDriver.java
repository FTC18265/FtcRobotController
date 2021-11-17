package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DriverControlPeriod")
public class freightFrenzyDriver extends LinearOpMode {

    private DcMotor topright;
    private DcMotor topleft;
    private DcMotor bottomright;
    private DcMotor bottomleft;
    private DcMotor arm;
    private DcMotor intake;
    private Servo pusher;
    private DcMotor carousel;

    private AnalogInput potentiometer;

    int level = 3;


    @Override
    public void runOpMode() {

        topright = hardwareMap.get(DcMotor.class, "topright");
        topleft = hardwareMap.get(DcMotor.class, "topleft");
        bottomright = hardwareMap.get(DcMotor.class, "bottomright");
        bottomleft = hardwareMap.get(DcMotor.class, "bottomleft");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        pusher = hardwareMap.get(Servo.class, "pusher");


        topright.setDirection(DcMotorSimple.Direction.REVERSE);
        topleft.setDirection(DcMotorSimple.Direction.REVERSE);

        pusher.setPosition(1);
        carousel.setPower(0);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //drive train
                topright.setPower
                    (0.5 * (gamepad1.left_stick_x + gamepad1.right_stick_x + gamepad1.right_stick_y));
                bottomright.setPower
                    (0.5 * (-gamepad1.left_stick_x + (gamepad1.right_stick_x - gamepad1.right_stick_y)));
                topleft.setPower
                    (0.5 * (gamepad1.left_stick_x + (gamepad1.right_stick_x - gamepad1.right_stick_y)));
                bottomleft.setPower
                    (0.5 * (-gamepad1.left_stick_x + gamepad1.right_stick_x + gamepad1.right_stick_y));
                telemetry.update();

                //carousel
                /*
                if (gamepad1.y) {
                    carousel.setPower(1);
                }
                else{
                    carousel.setPower(0);
                }

                //potentiometer test
                telemetry.addData("voltage", potentiometer.getVoltage());
                telemetry.update();

                 */

                //arm
                setArm(level);

                if(gamepad1.left_bumper){
                    level -= level;
                    if (level < 0){
                        level = 0;
                    }
                }
                if(gamepad1.right_bumper){
                    level += level;
                    if (level > 3){
                        level = 3;
                    }
                }

                //intake
                if (gamepad1.a) {
                    intake.setPower(1);
                }
                else{
                    intake.setPower(0);
                }

                //output
                if (gamepad1.x) {
                    intake.setPower(-1);
                    pusher.setPosition(0);
                }
                else{
                    pusher.setPosition(1);
                    setArm(0);
                }

            }


        }

    }

    public void setArm (int level){
        if (level == 1){
            while (potentiometer.getVoltage() < 1.2165){
                arm.setPower(0.3);
            }
        }
        else if (level == 2){
            while (potentiometer.getVoltage() < 1.4214){
                arm.setPower(0.3);
            }
        }
        else if (level == 3){
            while (potentiometer.getVoltage() < 1.7637){
                arm.setPower(0.3);

            }
        }
        else if (level == 0){
            while (potentiometer.getVoltage() > 1.076){
                arm.setPower(0.3);
            }
        }
        arm.setPower(0);

    }

}
