package org.firstinspires.ftc.teamcode.HardwareSubsytems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Robot {

    public Drivetrain drivetrain;
//    public DeadWheelLocalizer deadWheelLocalizer;
//    public IMULocalizer imuLocalizer;
//    public Intake intake;
//    public Virtual4Bar virtual4Bar;
//    public Depositor depositor;
//    public Lift lift;
//    //public FoundationGrabber foundationGrabber;
//    public FoundationGrabberV2 foundationGrabber;
//    public AutoClawTele autoClaw;
//    public Lights lights;
//    public Scissor scissor;

    OpMode opMode;
    DeviceManager deviceManager;

    public Robot(OpMode opMode){
        this.opMode = opMode;
        deviceManager = new DeviceManager(opMode.hardwareMap);
    }

    public void init(){
        deviceManager.init();
        drivetrain = new Drivetrain(opMode.hardwareMap, opMode, deviceManager);
//        deadWheelLocalizer = new DeadWheelLocalizer(opMode, deviceManager);
//        imuLocalizer = new IMULocalizer(opMode, deviceManager);
//        intake = new Intake(opMode, deviceManager);
//        virtual4Bar = new Virtual4Bar(opMode, deviceManager);
//        depositor = new Depositor(opMode, deviceManager);
//        lift = new Lift(opMode, deviceManager);
//        foundationGrabber = new FoundationGrabberV2(opMode, deviceManager);
//        autoClaw = new AutoClawTele(opMode, deviceManager);
//        lights = new Lights(opMode, deviceManager);
//        scissor = new Scissor(opMode, deviceManager);
    }

    public void setBulkReadManual(){
        deviceManager.setBulkReadManual();
    }

    public void clearBulkCache(){
        deviceManager.clearBulkCache();
    }
}
