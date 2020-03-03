package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;

@TeleOp(name = "yeahhhhh", group = "TeleOp")
public class TeleOpV3 extends OpMode {
    private HardwareMain robot = new HardwareMain(this);

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        //drivetrain
        robot.drivetrain.robotCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, (gamepad1.right_trigger - gamepad1.left_trigger));
        if (gamepad1.left_stick_button) robot.drivetrain.resetAngle();

        //intake
        if (gamepad2.a) robot.intake.setIntakePower(1);
        else if (gamepad2.b) robot.intake.setIntakePower(-1);
        else robot.intake.setIntakePower(0);

        //lift
        robot.lift.raiseLift(gamepad2.dpad_up);
        robot.lift.lowerLift(gamepad2.dpad_down);
        robot.lift.adjustLevel(gamepad2.right_stick_y);
        robot.lift.resetLift(gamepad2.right_stick_button);

        //send telemetries from individual subsystems
        robot.drivetrain.sendTelemetry();
        robot.intake.sendTelemetry();
        robot.lift.sendTelemetry();
    }
}