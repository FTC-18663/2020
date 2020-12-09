package org.firstinspires.ftc.teamcode;


public class Carousel {


    public Carousel() {

    }

    public void init() {

    }


    public void setCarousel(boolean x, double Speed){
        if(x) {
            Robot.robotMap.carousel.setPower(Speed);
        }
        else{
            Robot.robotMap.carousel.setPower(0.00);
        }

        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    public void stop() {
        Robot.robotMap.carousel.setPower(0.00);
    }
}