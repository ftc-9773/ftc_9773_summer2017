/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.drivesys;

public class Wheel {

    String type;
    double diameter;
    double circumference;

    public Wheel(String type, double diameter)
    {
        this.type = type;
        this.diameter = diameter;
        this.circumference = diameter * Math.PI;
    }

    public double getCircumference() { return (this.circumference); }
    public double getRadius() { return (this.diameter); }


}
