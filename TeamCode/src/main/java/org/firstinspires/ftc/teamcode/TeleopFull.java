package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareSubsytems.Robot;
import org.firstinspires.ftc.teamcode.Miscellaneous.ButtonPress;
import org.firstinspires.ftc.teamcode.Miscellaneous.InputScalar;

import static org.firstinspires.ftc.teamcode.Miscellaneous.InputScalar.*;
import static org.firstinspires.ftc.teamcode.Miscellaneous.InputScalar.scale;

@TeleOp(name="Teleop Full", group="A")
public class TeleopFull extends LinearOpMode {
    private final double STANDARD_DT_MULTIPLIER = 1;
    private final double RIGHT_DT_MULTIPLIER = 0.4;
    private final double LEFT_DT_MULTIPLIER = 0.3;
    private final double FINE_ADJUST_AMOUNT = 10.0;

    Robot robot;

    private boolean fieldOriented = false, intakeOn = false;
    private int layer = 0;

    @Override
    public void runOpMode() {
        AutoClawAutoV2 autoClawL = new AutoClawAutoV2(hardwareMap);
        AutoClawAutoV2R autoClawR = new AutoClawAutoV2R(hardwareMap);
        autoClawL.goToInit();
        autoClawR.goToInit();

        InputScalar.setScaleType(ScaleType.CUBEOVERABS);

        waitForStart();

        robot = new Robot(this);

        robot.init();

        //robot.intake.spinOut();

        while (opModeIsActive()) {
            ButtonPress.giveMeInputs(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper, gamepad1.left_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button, gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_bumper, gamepad2.left_bumper, gamepad2.left_stick_button, gamepad2.right_stick_button);

            if(ButtonPress.isGamepad2_dpad_up_pressed()){
                layer++;
                robot.lift.goToLayer(layer);
            }
            if(ButtonPress.isGamepad2_dpad_down_pressed()){
                layer--;
                robot.lift.goToLayer(layer);
            }
            if(ButtonPress.isGamepad2_dpad_right_pressed()){
                //layer = 0;
                robot.lift.goToLayer(0);
                robot.depositor.goToIntaking();
                robot.virtual4Bar.goToIntaking();
            }
            if(ButtonPress.isGamepad2_x_pressed()) layer = 0;
            robot.lift.fineAdjust((int)(-gamepad2.left_stick_y * FINE_ADJUST_AMOUNT));
            robot.lift.fineAdjust((int)(-gamepad2.right_stick_y * FINE_ADJUST_AMOUNT));//reee
            if(robot.lift.leftLift.getTargetPosition() == 0 && robot.lift.leftLift.getCurrentPosition() < 25){
                robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.leftLift.setPower(0);
            }else{
                robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.leftLift.setPower(1);
                robot.intake.stop();
            }

            /*
            if(robot.intake.intakeState == Intake.IntakeState.IN){
                robot.lights.setPower(.5);
            }else if (robot.intake.intakeState == Intake.IntakeState.OUT) {
                robot.lights.breath();
            }
            else {
                robot.lights.setPower(0);
            }
            */

            if(ButtonPress.isGamepad1_a_pressed()) robot.intake.goIn();
            if(ButtonPress.isGamepad1_b_pressed()) robot.intake.goOut();

            if(ButtonPress.isGamepad2_right_bumper_pressed()) robot.depositor.cycleDepositor();
            if(ButtonPress.isGamepad2_b_pressed()) robot.depositor.clampToggle();

            if(ButtonPress.isGamepad2_left_bumper_pressed()) robot.virtual4Bar.cycleV4B();

            if(ButtonPress.isGamepad1_left_bumper_pressed())robot.foundationGrabber.toggle();

            if(gamepad2.left_trigger > 0.4 && gamepad2.right_trigger > 0.4) robot.depositor.cap();

            if(ButtonPress.isGamepad1_right_bumper_pressed()) robot.imuLocalizer.resetHeading();
            if(ButtonPress.isGamepad1_y_pressed()) fieldOriented = !fieldOriented;
            if(gamepad1.right_trigger > 0)InputScalar.setMultiplier(RIGHT_DT_MULTIPLIER);
            if(gamepad1.left_trigger > 0)InputScalar.setMultiplier(LEFT_DT_MULTIPLIER);
            if(gamepad1.right_trigger <= 0 && gamepad1.left_trigger <= 0)InputScalar.setMultiplier(STANDARD_DT_MULTIPLIER);
            if(fieldOriented) {
                robot.drivetrain.setVectorField(scale(-gamepad1.left_stick_x), scale(-gamepad1.left_stick_y), 0.90 * scale(-gamepad1.right_stick_x), robot.imuLocalizer.getHeadingRadian());
            }else{
                robot.drivetrain.setVectorRobot(scale(-gamepad1.left_stick_x), scale(-gamepad1.left_stick_y), 0.90 * scale(-gamepad1.right_stick_x));
            }

            if(ButtonPress.isGamepad2_y_pressed()) robot.lift.resetEncoderTicks();

            if(ButtonPress.isGamepad2_a_pressed()) robot.scissor.goToOut();

            telemetry.addData("threshold", robot.lift.leftLift.getTargetPositionTolerance());
            telemetry.addData("leftPower:", robot.lift.leftLift.getPower());
            telemetry.addData("ticks", robot.lift.leftLift.getCurrentPosition());
            telemetry.addData("targerPos", robot.lift.leftLift.getTargetPosition());
            telemetry.update();
        }
        robot.virtual4Bar.leftV4B.getController().pwmDisable();
    }
}
