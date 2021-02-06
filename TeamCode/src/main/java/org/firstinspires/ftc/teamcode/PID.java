package org.firstinspires.ftc.teamcode;

public class PID {

    private double kP;
    private double kI;
    private double kD;
    private double lastError;
    private double sumError;
    private long lastTime;

    public PID(double kP, double kI, double kD) {
        setkP(kP);
        setkI(kI);
        setkD(kD);
        setLastError(0.00);
    }

    public PID() {
        setkP(0.00);
        setkI(0.00);
        setkD(0.00);
        setLastError(0.00);
    }

    public double getkP() {
        return this.kP;
    }

    public double getkI() {
        return this.kI;
    }

    public double getkD() {
        return this.kD;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    private void setLastError(double error) {
        this.lastError = error;
    }

    public double getLastError() {
        return this.lastError;
    }

    private void setSumError(double sum) {
        this.sumError = sum;
    }

    public double getSumError() {
        return sumError;
    }


    public double calculatePID(double target, double position, double currentTime) {
        double currentError = target - position;
        double dTime = currentTime - lastTime;

        double p = calculateP(kP, currentError);
        double i = calculateI(currentError, dTime);
        double d = calculateD(currentError, dTime);

        setLastError(currentError);

        return p + i + d;
    }

    private double calculateP(double p, double newError) {
        return p * newError;
    }

    private double calculateI(double newError, double dTime) {

        if(lastError > 0 && newError < 0) {
            setSumError(0.00); //set to zero when overshoot
        } else if(lastError < 0 && newError > 0){
            setSumError(0.00); //set to zero when overshoot
        } else {
            setSumError(getSumError() + (newError * dTime));
        }

        return getSumError() * kI;
    }

    private double calculateD(double newError, double dTime) {
        return (newError - lastError) * dTime;
    }
}
