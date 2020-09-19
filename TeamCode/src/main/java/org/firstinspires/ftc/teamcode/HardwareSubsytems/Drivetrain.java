package org.firstinspires.ftc.teamcode.HardwareSubsytems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.Drive.SampleMecanumDrive;

public class Drivetrain extends SampleMecanumDrive implements Subsytem {
//    private ExpansionHubMotor frontLeft, frontRight, backLeft, backRight;

    private OpMode opMode;
    private DeviceManager deviceManager;

    public Drivetrain(HardwareMap hwmap, OpMode opMode, DeviceManager deviceManager){
        super(hwmap);
        this.opMode = opMode;
        this.deviceManager = deviceManager;

//        frontLeft = deviceManager.leftFrontMotor;
//        frontRight = deviceManager.rightFrontMotor;
//        backLeft = deviceManager.leftBackMotor;
//        backRight = deviceManager.rightBackMotor;
//
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

//    public void setVectorRobot(double x, double y, double w){
//        double fl = (-x+y-w), fr = (x+y+w), bl = (x+y-w), br = (-x+y+w);
//        double max = Math.max(Math.abs(fl),Math.max(Math.abs(fr), Math.max(Math.abs(bl),Math.abs(br))));
//        if(max > 1){ fl /= max; fr /= max; bl /= max; br /= max; }
//        setPowers(fl, fr, bl, br);
//    }
//
//    public void setVectorField(double x, double y, double w, double currentHeading){
//        double xdot = x*Math.cos(-currentHeading) - y*Math.sin(-currentHeading);
//        double ydot =  y*Math.cos(-currentHeading) + x*Math.sin(-currentHeading);
//        setVectorRobot(xdot, ydot, w);
//    }
//
//    private void setPowers(double fl, double fr, double bl, double br){
//        frontLeft.setPower(fl);
//        frontRight.setPower(fr);
//        backLeft.setPower(bl);
//        backRight.setPower(br);
//    }
}
