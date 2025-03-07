// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class PVector {
    public double x, y, z;

    public PVector (double a, double b, double c){
        x=a;
        y=b;
        z=c;
    }

    public double dist2D(PVector t){

    return Math.sqrt(Math.pow(t.x - this.x, 2) + Math.pow((t.y - this.y), 2) );
    }

    public String toString(){
        return "(" + x + ','+ y + ')'; //I don't car about Z right now, maybe never who knows.
    }
}
