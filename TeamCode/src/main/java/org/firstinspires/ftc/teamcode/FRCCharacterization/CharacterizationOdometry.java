/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.FRCCharacterization;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.Util.RevEncoder;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


@TeleOp(name="CharacterizationRearWheels", group="Linear Opmode")
//@Disabled
public class CharacterizationOdometry extends LinearOpMode {

    static private double WHEEL_DIAMETER = 0.333;
    static private double ENCODER_EDGES_PER_REV = 512 / 4.;
    double encoderConstant = (1 / ENCODER_EDGES_PER_REV) * WHEEL_DIAMETER * Math.PI;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

//    RevBulkData bulkData;
//    AnalogInput a0, a1, a2, a3;
//    DigitalChannel d0, d1, d2, d3, d4, d5, d6, d7;
    ExpansionHubMotor leftFrontDrive, rightFrontDrive, leftRearDrive, rightRearDrive;
    ExpansionHubEx expansionHub;

    RevEncoder leftOdo, rightOdo;

    Number[] numberArray = new Number[10];

    String filename = "AdafruitIMUCalibration.json";
    File file = new File(filename);
    JSONObject data = new JSONObject();

    double power = 0, previousPower = 0;;

    final double POWER_RAMP_FACTOR = 0.01;

    double now, leftPosition, leftRate, rightPosition, rightRate, battery, motorVolts, leftMotorVolts, rightMotorVolts;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        leftFrontDrive = (ExpansionHubMotor) hardwareMap.dcMotor.get("left_front_drive");
        rightFrontDrive = (ExpansionHubMotor) hardwareMap.dcMotor.get("right_front_drive");
        leftRearDrive = (ExpansionHubMotor) hardwareMap.dcMotor.get("left_rear_drive");
        rightRearDrive = (ExpansionHubMotor) hardwareMap.dcMotor.get("right_rear_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        leftOdo = new RevEncoder(leftRearDrive);
        rightOdo = new RevEncoder(rightRearDrive);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            reset("Please press 'A' to ramp up");

            while (power < 1){
                now = runtime.nanoseconds();
                power = POWER_RAMP_FACTOR * now;
                if (power >= 1) break;
                recordData(power, "slow-forward");
            }

            reset("Please press 'A' to ramp down");

            while (power > 0){
                now = runtime.nanoseconds();
                power = 1 - (now * POWER_RAMP_FACTOR);
                if (power <= 0) break;
                recordData(power, "slow-backward");
            }

            reset("Please press 'A' to set half speed forwards");

            setMotorPowers(0.5);
            now = runtime.nanoseconds();
            while (now != ElapsedTime.SECOND_IN_NANO * 0.75){
                now = runtime.nanoseconds();
                recordData(0.5, "fast-forward");
            }
            setMotorPowers(0);

            reset("Please press 'A' to set half speed backwards");

            setMotorPowers(-0.5);
            now = runtime.nanoseconds();
            while (now != ElapsedTime.SECOND_IN_NANO * 0.75){
                now = runtime.nanoseconds();
                recordData(-0.5, "fast-backward");
            }
            setMotorPowers(0);
        }
        JSONArray finalData = new JSONArray();
        finalData.add(data);

        try {
            FileWriter fw = new FileWriter(file);
            fw.write(finalData.toJSONString());
            fw.flush();
        } catch (IOException e){
            e.printStackTrace();
        }
    }

    private void setMotorPowers(double newPower){
        leftFrontDrive.setPower(newPower);
        rightFrontDrive.setPower(newPower);
        leftRearDrive.setPower(newPower);
        rightRearDrive.setPower(newPower);
    }

    private void recordData(double newPower, String loopType){
        leftPosition = leftOdo.getCurrentPosition() * encoderConstant;
        leftRate = leftOdo.getCorrectedVelocity() * encoderConstant;

        rightPosition = rightOdo.getCurrentPosition() * encoderConstant;
        rightRate = rightOdo.getCorrectedVelocity() * encoderConstant;

        battery = expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
        motorVolts = battery * Math.abs(previousPower);

        leftMotorVolts = motorVolts;
        rightMotorVolts = motorVolts;

        previousPower = newPower;

        setMotorPowers(newPower);

        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = newPower;
        numberArray[3] = leftMotorVolts;
        numberArray[4] = rightMotorVolts;
        numberArray[5] = leftPosition;
        numberArray[6] = rightPosition;
        numberArray[7] = leftRate;
        numberArray[8] = rightRate;
        numberArray[9] = 0;

        for (Number number : numberArray) {
            data.put(loopType, number.toString());
        }
    }

    private void reset(String telemetryData){
        telemetry.addData("Status: ", telemetryData);
        telemetry.update();

        while  (!gamepad1.a) {}

        runtime.reset();
    }
}
