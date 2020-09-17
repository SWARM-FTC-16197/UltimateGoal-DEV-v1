package org.firstinspires.ftc.teamcode.HardwareSubsytems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class DeviceManager{
    HardwareMap hardwareMap;
    public RevBulkData bulkData1, bulkData2;
    ExpansionHubEx expansionHub1, expansionHub2;

    public ExpansionHubMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor,
                             leftIntakeMotor, rightIntakeMotor,
                             leftLiftMotor,
                             lights;

    public ServoImplEx leftV4BServo, rightV4BServo,
                       leftFoundationServo, rightFoundationServo,
                       cheeseClawServo, cheeseVerticalServo,
                       frontDepositServo, backDepositServo,
                        scissor;

    BNO055IMU imu;

    public DistanceSensor distance;

    public DeviceManager(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void init(){
        //distance = hardwareMap.get(DistanceSensor.class, "distance");

        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        leftFrontMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftBack");
        rightBackMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightBack");

        leftIntakeMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftIntake");
        rightIntakeMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightIntake");

        leftLiftMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftLift");

        lights = (ExpansionHubMotor) hardwareMap.dcMotor.get("lights");

        leftV4BServo = (ServoImplEx) hardwareMap.servo.get("leftV4B");
        rightV4BServo = (ServoImplEx) hardwareMap.servo.get("rightV4B");

        leftFoundationServo = (ServoImplEx) hardwareMap.servo.get("leftFoundation");
        rightFoundationServo = (ServoImplEx) hardwareMap.servo.get("rightFoundation");

        cheeseClawServo = (ServoImplEx) hardwareMap.servo.get("autoClawClamp");
        cheeseVerticalServo = (ServoImplEx) hardwareMap.servo.get("autoClawVertical");

        frontDepositServo = (ServoImplEx) hardwareMap.servo.get("frontDeposit");
        backDepositServo = (ServoImplEx) hardwareMap.servo.get("backDeposit");

        scissor = (ServoImplEx) hardwareMap.servo.get("scissor");
    }

    public void updateAll(){
        bulkData1 = expansionHub1.getBulkInputData();
        bulkData2 = expansionHub1.getBulkInputData();
    }

    public void updateOnlyHub1(){
        bulkData1 = expansionHub1.getBulkInputData();
    }

    public void updateOnlyHub2(){
        bulkData2 = expansionHub1.getBulkInputData();
    }

}