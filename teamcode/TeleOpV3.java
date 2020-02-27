package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;

public class TeleOpV3 extends OpMode {
    public HardwareMain robot = new HardwareMain(this);

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.drivetrain.robotCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, (gamepad1.left_trigger + gamepad1.right_trigger));
        if (gamepad2.a) {
            robot.intake.setIntakePower(1);
        } else if (gamepad2.b) {
            robot.intake.setIntakePower(1);
        }

        //send telemetries from individual subsystems
        robot.drivetrain.sendTelemetry();
    }
}