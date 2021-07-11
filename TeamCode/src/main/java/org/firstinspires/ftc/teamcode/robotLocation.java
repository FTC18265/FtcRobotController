package org.firstinspires.ftc.teamcode;

public class robotLocation {
    double angle;
    double x;
    double y;

    public robotLocation(double angle) {
        this.angle = angle;
    }

    public double getHeading() {
        double angle = this.angle;
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    @Override
    public String toString() {
        return "RobotLocation: angle (" + angle + ")";
    }

    public void turn(double angleChange) {
        angle += angleChange;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public double doubleGetAngle(){
        return angle *= 2;
    }

    public double getX(){
        double x = this.x;
        x = 5;
        return x;
    }

    public void changX(double change){
        x += change;
    }

    public void setX(double x){
        this.x = x;
    }

    public double getY(){
        double y = this.y;
        y = 5;
        return y;
    }
    public void changY(double change){
        y += change;
    }

    public void setY(double y){
        this.y = y;
    }

}