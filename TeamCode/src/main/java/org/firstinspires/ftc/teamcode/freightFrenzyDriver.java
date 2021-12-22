package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DriverControlPeriod")
public class freightFrenzyDriver extends LinearOpMode {

    private DcMotor topright;
    private DcMotor topleft;
    private DcMotor bottomright;
    private DcMotor bottomleft;
    private DcMotorEx arm;
    private DcMotorEx susan;
    //private DcMotor susan;

    private DcMotor intake;
    private Servo door;
    private DcMotor carousel;
    private Rev2mDistanceSensor extradistancesensor;

    private AnalogInput potentiometer;

    private int level = 0;
    private int degree = 1;
    private double lasttime;
    private double currenttime;
    private int lastDegree;

    private String button;

    public static final double NEW_P = 1.5;
    public static final double NEW_I = 0.5;
    public static final double NEW_D = 0.0;
    public static final double NEW_F = 12.6;

    public static final double turning_NEW_P = 1.2;
    public static final double turning_NEW_I = 0.5;
    public static final double turning_NEW_D = 0.0;
    public static final double turning_NEW_F = 17;

    @Override
    public void runOpMode() {

        topright = hardwareMap.get(DcMotor.class, "topright");
        topleft = hardwareMap.get(DcMotor.class, "topleft");
        bottomright = hardwareMap.get(DcMotor.class, "bottomright");
        bottomleft = hardwareMap.get(DcMotor.class, "bottomleft");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        susan = hardwareMap.get(DcMotorEx.class, "susan");
        //susan = hardwareMap.get(DcMotor.class, "susan");

        intake = hardwareMap.get(DcMotor.class, "intake");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        door = hardwareMap.get(Servo.class, "door");
        extradistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "extradistancesensor");


        bottomright.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomleft.setDirection(DcMotorSimple.Direction.REVERSE);
        susan.setDirection(DcMotorSimple.Direction.REVERSE);

        door.setPosition(0);
        carousel.setPower(0);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        susan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int position = arm.getCurrentPosition();
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(0);


        int turningPosition = susan.getCurrentPosition();
        susan.setTargetPosition(turningPosition);
        susan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        susan.setPower(0);

        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        //PIDFCoefficients turningPidOrig = susan.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        // change coefficients.
        arm.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        arm.setPositionPIDFCoefficients(5.0);

        susan.setVelocityPIDFCoefficients(turning_NEW_P, turning_NEW_I, turning_NEW_D, turning_NEW_F);
        susan.setPositionPIDFCoefficients(3.0);
        susan.setTargetPositionTolerance(3);



        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients turningPidModified = susan.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        arm.setPower(0.5);
        susan.setPower(0.5);


//        arm.setTargetPosition(300);

        while (opModeIsActive()) {
            //drive train
            //susan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //susan.setPower(0.75);

            bottomright.setPower
                (0.5 * (gamepad1.left_stick_x + gamepad1.right_stick_x - gamepad1.right_stick_y));
            topright.setPower
                (0.5 * (-gamepad1.left_stick_x + (gamepad1.right_stick_x + gamepad1.right_stick_y)));
            bottomleft.setPower
                (0.5 * (gamepad1.left_stick_x + (gamepad1.right_stick_x + gamepad1.right_stick_y)));
            topleft.setPower
                (0.5 * (-gamepad1.left_stick_x + gamepad1.right_stick_x - gamepad1.right_stick_y));
            telemetry.update();

            //susan.setPower(gamepad2.right_stick_x * 0.3);

            //carousel
            if (gamepad1.b){
                button = "gamepad1.b";
                carousel.setPower(0.5);
            }
            if (gamepad1.x) {
                button = "gamepad1.x";
                carousel.setPower(-0.5);
            }
            if (gamepad1.y) {
                button = "gamepad1.y";
                carousel.setPower(0);
            }

            /*
            //distance sensor test
            if(extradistancesensor.getDistance(DistanceUnit.CM) < 12.5){
                telemetry.addLine("object detected");
            }else{
                telemetry.addLine("no object");
            }

            if(gamepad1.right_bumper && extradistancesensor.getDistance(DistanceUnit.CM) < 12.5){
                 pullBack();

                 level = 1;
                 setArm(level);

                 intake.setPower(0);
            }

             */

            //potentiometer test
            telemetry.addData("voltage", potentiometer.getVoltage());
//            telemetry.addData("Runtime", "%.03f", getRuntime());
//            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.04f, %.04f",
//                    pidOrig.p, pidOrig.i, pidOrig.d, pidOrig.f);
//            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f, %.04f",
//                    pidModified.p, pidModified.i, pidModified.d, pidModified.d);
            telemetry.addData("Arm", "Level: %d, Target: %d, Current: %d",
                    level, arm.getTargetPosition(), arm.getCurrentPosition());
            telemetry.addData("susan", "Degree: %d, Target: %d, Current: %d",
                    degree, susan.getTargetPosition(), susan.getCurrentPosition());
            telemetry.addData("Gamepad", button);
            telemetry.update();


            telemetry.update();

            //arm
//            setArmLevel(level);
//            arm.setPower(0.5);
            currenttime = getRuntime();
            if(gamepad2.a && currenttime - lasttime > 1){
                button = "gamepad2.a";
                level--;
                lasttime = currenttime;
                if (level < 0){
                    level = 0;
                }
                setArm(level);
            }
            if(gamepad2.y && currenttime - lasttime > 1){
                button = "gamepad2.y";
                level++;
                lasttime = currenttime;
                if (level > 2){
                    level = 2;
                }
                setArm(level);
            }

            position = arm.getCurrentPosition();
            if (gamepad2.dpad_down && currenttime - lasttime > 0.1) {
                button = "gamepad2.dpad_down";
                position += 35;
                arm.setTargetPosition(position);
                lasttime = currenttime;
            }
            if (gamepad2.dpad_up && currenttime - lasttime > 0.1) {
                button = "gamepad2.dpad_up";
                position -= 35;
                arm.setTargetPosition(position);
                lasttime = currenttime;
            }



            if(gamepad2.x && currenttime - lasttime > 1){
                button = "gamepad2.x";
                lastDegree = degree;
                degree--;
                lasttime = currenttime;
                if (degree < -1){
                    degree = -1;
                }
                setTurn();
            }
            if(gamepad2.b && currenttime - lasttime > 1){
                button = "gamepad2.b";
                lastDegree = degree;
                degree++;
                lasttime = currenttime;
                if (degree > 1){
                    degree = 1;
                }
                setTurn();
            }

            turningPosition = susan.getCurrentPosition();
            if (gamepad2.left_bumper && currenttime - lasttime > 0.1) {
                button = "gamepad2.left_bumper";
                turningPosition -= 35;
                if (turningPosition > 0){
                    turningPosition = 0;
                }
                susan.setTargetPosition(turningPosition);
                lasttime = currenttime;

            }
            if (gamepad2.right_bumper && currenttime - lasttime > 0.1) {
                button = "gamepad2.right_bumper";
                turningPosition += 35;
                if (turningPosition < -1500){
                    turningPosition = -1500;
                }
                susan.setTargetPosition(turningPosition);
                lasttime = currenttime;
            }




            //intake
            if (gamepad1.right_bumper) {
                button = "gamepad1.right_bumper";
                intake.setPower(-0.3);
            }
            else{
                intake.setPower(0);
            }

            if(gamepad1.a){
                intake.setPower(0.3);
            }

            //output
            if (gamepad1.left_bumper) {
                button = "gamepad1.left_bumper";
                door.setPosition(1);
            }
            else{
                door.setPosition(0);
//                setArm(0);
            }

            //susan.setTargetPosition(keepPosition);

        }

    }

    public void setArm (int level){
        if (level == 1){
            arm.setPower(0.5);
//            while (potentiometer.getVoltage() < 1.2165){
//                arm.setPower(0.3);
//            }
            arm.setTargetPosition(-700);
        }
        else if (level == 2){
            arm.setPower(0.5);

//            while (potentiometer.getVoltage() < 1.4214){
//                arm.setPower(0.3);
//            }
            arm.setTargetPosition(-1200);
        }

        else if (level == 0){
//            while (potentiometer.getVoltage() > 1.076){
//                arm.setPower(0.3);
//            }
            arm.setPower(0.5);
            arm.setTargetPosition(0);

        }
//        arm.setPower(0);

    }

    public void setTurn (){
        if (degree == 1){
            susan.setPower(0.5);
//            while (potentiometer.getVoltage() < 1.2165){
//                arm.setPower(0.3);
//            }
            susan.setTargetPosition(0);
        }
        else if (degree == -1){
            susan.setPower(0.5);

//            while (potentiometer.getVoltage() < 1.4214){
//                arm.setPower(0.3);
//            }
            susan.setTargetPosition(-1500);
        }

        else if (degree == 0){
//            while (potentiometer.getVoltage() > 1.076){
//                arm.setPower(0.3);
//            }
            susan.setPower(0.5);
            susan.setTargetPosition(-750);

        }
    }

    public void pullBack(){
        topleft.setPower(-0.3);
        topright.setPower(-0.3);
        bottomright.setPower(-0.3);
        bottomleft.setPower(-0.3);

        sleep(1000);

        topleft.setPower(0);
        topright.setPower(0);
        bottomright.setPower(0);
        bottomleft.setPower(0);
    }
/*
    public void setTurn (){
        if (degree > lastDegree){

            if(degree == 0){
                susan.setPower(-0.3);

                while(colorsensor.blue() > 500){

                }
                while(colorsensor.blue() < 500){

                }
                susan.setPower(0);

            }
            if(degree == 1){
                susan.setPower(-0.3);

                while(colorsensor.blue() < 1000){

                }
                susan.setPower(0);
            }
        }
        if (degree < lastDegree){
            if(degree == 0){
                susan.setPower(0.3);

                while(colorsensor.blue() > 500){

                }
                while(colorsensor.blue() < 500){

                }

                susan.setPower(0);
            }
            if(degree == -1){
                susan.setPower(0.2);

                while(colorsensor.blue() < 1000){

                }
                susan.setPower(0);
            }
        }//keepPosition = susan.getCurrentPosition();
    }

 */



}
