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
import org.firstinspires.ftc.teamcode.ArmController;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DriverControlPeriod")
public class freightFrenzyDriver extends LinearOpMode {

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


    @Override
    public void runOpMode() {
        topright = hardwareMap.get(DcMotor.class, "topright");
        topleft = hardwareMap.get(DcMotor.class, "topleft");
        bottomright = hardwareMap.get(DcMotor.class, "bottomright");
        bottomleft = hardwareMap.get(DcMotor.class, "bottomleft");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        susan = hardwareMap.get(DcMotorEx.class, "susan");
        intake = hardwareMap.get(DcMotor.class, "intake");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        door = hardwareMap.get(Servo.class, "door");
        extradistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "extradistancesensor");
        distancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "distancesensor");

        MecanumController mecanumController = new MecanumController(topleft, topright, bottomleft, bottomright);
        mecanumController.init();

        ArmController armController = new ArmController(arm);
        armController.init();

        SusanController susanController = new SusanController(susan);
        susanController.init();

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        door.setPosition(0);
        carousel.setPower(0);

        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("1234567810");
        telemetry.update();

        waitForStart();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients turningPidModified = susan.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        arm.setPower(0.5);
        susan.setPower(0.5);


        while (opModeIsActive()) {
            //drive train
            bottomright.setPower
                    (0.5 * (gamepad1.left_stick_x + (gamepad1.right_stick_x - gamepad1.right_stick_y)));
            topright.setPower
                    (0.5 * (gamepad1.left_stick_x + (-gamepad1.right_stick_x - gamepad1.right_stick_y)));
            bottomleft.setPower
                    (0.5 * (-gamepad1.left_stick_x + (-gamepad1.right_stick_x - gamepad1.right_stick_y)));
            topleft.setPower
                    (0.5 * (-gamepad1.left_stick_x + (gamepad1.right_stick_x - gamepad1.right_stick_y)));

            telemetry.addData("topright", topright.getCurrentPosition());
            telemetry.addData("topleft", topleft.getCurrentPosition());
            telemetry.addData("bottomright", bottomright.getCurrentPosition());
            telemetry.addData("bottomleft", bottomleft.getCurrentPosition());

            telemetry.update();

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
            telemetry.addData("Arm", "Level: %d, Target: %d, Current: %d",
                    level, arm.getTargetPosition(), arm.getCurrentPosition());
            telemetry.addData("susan", "Degree: %d, Target: %d, Current: %d",
                    degree, susan.getTargetPosition(), susan.getCurrentPosition());
            telemetry.addData("Gamepad", button);
            telemetry.update();


            currenttime = getRuntime();
            if(gamepad2.a && currenttime - lasttime > 1){
                button = "gamepad2.a";
                lasttime = currenttime;
                armController.adjustLevel(-1);
            }
            if(gamepad2.y && currenttime - lasttime > 1){
                lasttime = currenttime;
                button = "gamepad2.y";
                armController.adjustLevel(1);

                //secure freight
                secureFreight = true;
                secureFreightStartTime = currenttime;
                intake.setPower(-0.1);
            }

            if(secureFreight == true && currenttime - secureFreightStartTime > 0.5){
                intake.setPower(0);
                secureFreight = false;
            }


            if (gamepad2.dpad_down && currenttime - lasttime > 0.1) {
                button = "gamepad2.dpad_down";
                armController.microAjust(35);
                lasttime = currenttime;
            }
            if (gamepad2.dpad_up && currenttime - lasttime > 0.1) {
                button = "gamepad2.dpad_up";
                armController.microAjust(-35);
                lasttime = currenttime;
            }



            currenttime = getRuntime();
            if(gamepad2.x && currenttime - lasttime > 1){
                button = "gamepad2.x";
                lasttime = currenttime;
                susanController.adjustLevel(-1);
            }
            if(gamepad2.b && currenttime - lasttime > 1){
                button = "gamepad2.b";
                lasttime = currenttime;
                susanController.adjustLevel(1);
            }


            if (gamepad2.left_bumper && currenttime - lasttime > 0.1) {
                button = "gamepad2.dpad_down";
                susanController.microAjust(35);
                lasttime = currenttime;
            }
            if (gamepad2.right_bumper && currenttime - lasttime > 0.1) {
                button = "gamepad2.dpad_up";
                susanController.microAjust(-35);
                lasttime = currenttime;
            }


            //intake
            if (gamepad1.right_bumper) {
                button = "gamepad1.right_bumper";
                intake.setPower(0.5);
            }
            else if (secureFreight == false && releaseFreight == false){
                intake.setPower(0);
            }

            if(gamepad1.a){
                intake.setPower(-0.5);
            }

            //output
            if (gamepad1.left_bumper) {
                button = "gamepad1.left_bumper";
                door.setPosition(1);

                releaseFreight = true;
                intake.setPower(0.1);
                releaseFreightStartTime = currenttime;
            }
            else{
                door.setPosition(0);
//                setArm(0);
            }

            if(releaseFreight == true && currenttime - releaseFreightStartTime > 0.5){
                intake.setPower(0);
                releaseFreight = false;
            }

            //susan.setTargetPosition(keepPosition);

        }

    }

    public void pullBack(){
        topleft.setPower(-0.3);
        topright.setPower(0.3);
        bottomright.setPower(-0.3);
        bottomleft.setPower(0.3);

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
