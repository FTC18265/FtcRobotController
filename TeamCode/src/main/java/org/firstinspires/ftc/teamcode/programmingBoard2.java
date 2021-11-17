package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

public class programmingBoard2 {
    private DigitalChannel touchSensor;
    private DcMotor motor;
    private double ticksPerRotation;
    private Servo servo;
    private AnalogInput pot;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private BNO055IMU imu;

    //initialize
    public void init(HardwareMap hwMap) {

        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        motor = hwMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = hwMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.FORWARD);
        servo.scaleRange(0, 0.5);

        pot = hwMap.get(AnalogInput.class, "pot");

        colorSensor = hwMap.get(ColorSensor.class, "sensor_color_distance");
        distanceSensor = hwMap.get(DistanceSensor.class, "sensor_color_distance");

        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        // change to default set of parameters go here
        imu.initialize(params);
    }

    //methodss
    public boolean isTouchSensorPressed() {
        if(!touchSensor.getState()){
            return true;
        }
        return false;
    }

    /*
    public String isTouchSensorPressed() {
        if(touchSensor.getState() == true){
            return "pressed";
        }
        else{
            return "not pressed";
        }
    }
    */

    public void setMotorSpeed(double speed){
        motor.setPower(speed);
    }

    public double getMotorRotations(){
        return motor.getCurrentPosition() / ticksPerRotation;
    }

    public int setMode(boolean a, boolean b){
        int result = 0;
        if(a == true){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            result = 1;
        }

        if(b == true){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            result = 2;
        }
        return result;
    }

    public void setServoPosition(double position){
        servo.setPosition(position);
    }

    public double getPotAngle(){
        return Range.scale(pot.getVoltage(), 0, pot.getMaxVoltage(), 0, 270);
    }

    public double getPotAngleZeroToOne(){
        return Range.scale(pot.getVoltage(), 0, pot.getMaxVoltage(), 0, 1);
    }

    public int getAmountRed(){
        return colorSensor.red();
    }

    public double getDistance(DistanceUnit du){
        return distanceSensor.getDistance(du);
    }

    public int getAmountBlue(){
        return colorSensor.blue();
    }

    public double getHeading(AngleUnit angleUnit) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit);
        return angles.firstAngle;
    }

    public ArrayList<testItem> getTests() {
        ArrayList<testItem> tests = new ArrayList<>();
        tests.add(new testMotor("PB Motor", 0.5, motor));

        tests.add(new testAnalogInput("PB Pot", pot, 0, 270));
        return tests;
    }
}