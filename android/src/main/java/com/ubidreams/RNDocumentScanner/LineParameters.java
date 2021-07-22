package com.ubidreams.RNDocumentScanner;

public class LineParameters {
    private double rho;
    private double theta;

    public LineParameters(double rho, double theta) {
        this.rho = rho;
        this.theta = theta;
    }

    public double getRho() {
        return rho;
    }

    public double getTheta() {
        return theta;
    }

    @Override
    public String toString() {
        double thetaDeg = theta * 180 / Math.PI;
        return "theta = " + thetaDeg + "deg, rho = " + rho;
    }
}
