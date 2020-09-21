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

package org.firstinspires.ftc.teamcode.frc_characterization;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;


@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class FRC_Characterization_RR2_Tests extends LinearOpMode {

    static private double WHEEL_DIAMETER = 0.333;
    static private double ENCODER_EDGES_PER_REV = 512 / 4.;
    double encoderConstant = (1 / ENCODER_EDGES_PER_REV) * WHEEL_DIAMETER * Math.PI;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

//    RevBulkData bulkData;
//    AnalogInput a0, a1, a2, a3;
//    DigitalChannel d0, d1, d2, d3, d4, d5, d6, d7;
    ExpansionHubMotor leftFrontDrive, rightFrontDrive;
    ExpansionHubEx expansionHub;

    Number[] numberArray = new Number[10];

    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftRearPower = 0;
    double rightRearPower = 0;

    final double POWER_RAMP_FACTOR = 0.01;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        leftFrontDrive = (ExpansionHubMotor) hardwareMap.dcMotor.get("left_front_drive");
        rightFrontDrive = (ExpansionHubMotor) hardwareMap.dcMotor.get("right_front_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        double previousPower = 0;

        waitForStart();
        runtime.reset();

        double now, leftFrontPosition, leftFrontRate, rightFrontPosition, rightFrontRate, battery, motorVolts, leftFrontMotorVolts, rightFrontMotorVolts;

        while (opModeIsActive()) {

            telemetry.addData("Status: ", "Please press 'A' to ramp up");
            telemetry.update();

            while (!gamepad1.a) {}

            runtime.reset();

            while (leftFrontPower != 1){

                now = runtime.nanoseconds();
                setMotorPowers(now);

                leftFrontPosition = leftFrontDrive.getCurrentPosition() * encoderConstant;
                leftFrontRate = leftFrontDrive.getVelocity() * encoderConstant;

                rightFrontPosition = rightFrontDrive.getCurrentPosition() * encoderConstant;
                rightFrontRate = rightFrontDrive.getVelocity() * encoderConstant;

                battery = expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
                motorVolts = battery * Math.abs(previousPower);

                leftFrontMotorVolts = motorVolts;
                rightFrontMotorVolts = motorVolts;

                previousPower = (0.5 * (leftFrontPower + rightFrontPower));

                numberArray[0] = now;
                numberArray[1] = battery;
                numberArray[2] = (0.5 * (leftFrontPower + rightFrontPower));
                numberArray[3] = leftFrontMotorVolts;
                numberArray[4] = rightFrontMotorVolts;
                numberArray[5] = leftFrontPosition;
                numberArray[6] = rightFrontPosition;
                numberArray[7] = leftFrontRate;
                numberArray[8] = rightFrontRate;
                numberArray[9] = 0;
            }

            telemetry.addData("Status: ", "Please press 'A' to ramp down");
            telemetry.update();

            while  (!gamepad1.a) {}

            runtime.reset();

            while (leftFrontPower != 0){
                now = runtime.nanoseconds();
                setMotorPowers(1 - now);

                leftFrontPosition = leftFrontDrive.getCurrentPosition() * encoderConstant;
                leftFrontRate = leftFrontDrive.getVelocity() * encoderConstant;

                rightFrontPosition = rightFrontDrive.getCurrentPosition() * encoderConstant;
                rightFrontRate = rightFrontDrive.getVelocity() * encoderConstant;

                battery = expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
                motorVolts = battery * Math.abs(previousPower);

                leftFrontMotorVolts = motorVolts;
                rightFrontMotorVolts = motorVolts;

                previousPower = (0.5 * (leftFrontPower + rightFrontPower));

                numberArray[0] = now;
                numberArray[1] = battery;
                numberArray[2] = (0.5 * (leftFrontPower + rightFrontPower));
                numberArray[3] = leftFrontMotorVolts;
                numberArray[4] = rightFrontMotorVolts;
                numberArray[5] = leftFrontPosition;
                numberArray[6] = rightFrontPosition;
                numberArray[7] = leftFrontRate;
                numberArray[8] = rightFrontRate;
                numberArray[9] = 0;
            }
        }
    }

    private void setMotorPowers(double time){
        leftFrontPower = POWER_RAMP_FACTOR * time;
        rightFrontPower = POWER_RAMP_FACTOR * time;
        leftRearPower = POWER_RAMP_FACTOR * time;
        rightRearPower = POWER_RAMP_FACTOR * time;
    }
}
