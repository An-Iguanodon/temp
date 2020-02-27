package org.firstinspires.ftc.teamcode.old;

//imports

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.HardwareInit;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

//COMMENT IN EXPLANATIONS!!!!!!!

/*debouncer code

        if (gamepad2.x && !x_past) {
                test++;
                x_past = true;
                } else if (!gamepad2.x) {
                x_past = false;
                }
                telemetry.addData("test", test);
*/

@TeleOp(name = "yeahhhhh", group = "TeleOp")
public class TeleOpV2 extends OpMode {

    HardwareInit robot = new HardwareInit();
    //declare motor power variables and set to 0
    //driver 1
    private double LFMP = 0;
    private double RFMP = 0;
    private double LBMP = 0;
    private double RBMP = 0;

    //driver 2
    private double LIP = 0;
    private double RIP = 0;
    private double BP = 0;
    double MP = 0;

    //declare servo position variables and set to 0
    double capPos = 0;
    double clampPos = 0;
    double rotPos = 0;

    private double lx, ly, rx, ry, lt, rt, gyro, turn; //declare variables required for getGamepad1()

    private double lx2, ly2, rx2, ry2; //declare variables required for getGamepad2()


    boolean x_past = false;

    private String drive = "tank drive"; //declare drive mode variable (default = tank drive)

    private double margin = 0.1; //declare double for a dead-zone margin on triggers and joysticks

    private double test = 0;

    @Override
    public void init() {
        //code to be run after hitting init
        //initializes the drivetrain, intake, output, imu, and servos
        robot.initRobot(hardwareMap);
    }

    @Override
    public void loop() {
        //code to be run repeatedly after hitting play

        //driver 1
        getImu();
        getGamepad1();
        driveMode();

        //driver 2
        getGamepad2();
        intake();
        output(rx2, ly2);

    }

    private void getGamepad1() {
        //define gamepad 1 variables with gamepad 1 input
        rx = gamepad1.right_stick_x;
        ry = -gamepad1.right_stick_y;
        lx = gamepad1.left_stick_x;
        ly = -gamepad1.left_stick_y;
        gyro = robot.angles.firstAngle;
        lt = gamepad1.left_trigger;
        rt = gamepad1.right_trigger;
        turn = Math.cbrt(rt) - Math.cbrt(lt);
        TelemetryD("rx", rx);
        TelemetryD("ry", ry);
        TelemetryD("lx", lx);
        TelemetryD("ly", ly);
        TelemetryD("gyro", gyro);
        TelemetryD("lt", lt);
        TelemetryD("rt", rt);
        TelemetryD("turn", turn);
    }

    private void getGamepad2() {
        //define gamepad 2 variables with gamepad 2 input
        rx2 = gamepad2.right_stick_x;
        ry2 = -gamepad2.right_stick_y;
        lx2 = gamepad2.left_stick_x;
        ly2 = -gamepad2.left_stick_y;
        TelemetryD("rx2", rx2);
        TelemetryD("ry2", ry2);
        TelemetryD("lx2", lx2);
        TelemetryD("ly2", ly2);
    }

    private void driveMode() {
        //set buttons to change the drive mode between tank drive, field-centric, pov drive, and the motor test mode
        if (gamepad1.a) {
            drive = "tank drive";
        } else if (gamepad1.b) {
            drive = "field-centric";
        } else if (gamepad1.x) {
            drive = "pov drive";
        } else if (gamepad1.y) {
            drive = "motor test";
        }
        TelemetryS("drivemode", drive);
        //based on string drive, run a specific drive method
        switch (drive) {
            case "field-centric":
                robotCentricDrive(rx, ry, gyro - 90, turn);
                break;
            case "tank drive":
                tankDrive(ly, ry, turn);
                break;
            case "pov drive":
                povDrive(ly, lx, rx);
                break;
            case "motor test":
                motorTest(gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_down, gamepad1.dpad_left);

        }
    }

    private void robotCentricDrive(double x, double y, double gyro, double turn) {
        double vel = Math.hypot(x, y); //find the hypotenuse if the joystick was a unit circle
        double basetheta = Math.atan2(x, y); //find the inside angle of the triangle formed
        double realtheta = basetheta;// - gyro; //ignore imu heading for now because robot-centric works, but not the translation to field-centric
        double vx = (vel * Math.sin(realtheta));
        double vy = (vel * Math.cos(realtheta));
        TelemetryD("vel", vel);
        //TelemetryD("gyro", gyro); //unneeded when not using imu input
        TelemetryD("basetheta", basetheta);
        //TelemetryD("realtheta", realtheta); //unneeded when not using imu input
        TelemetryD("vx", vx);
        TelemetryD("vy", vy);
        LFMP = vy + vx + turn;
        RFMP = vy - vx - turn;
        LBMP = vy - vx + turn;
        RBMP = vy + vx - turn;
        setDrive();
    }

    private void tankDrive(double y1, double y2, double strafe) {
        //set the y of the left joystick to control the left side drive motors
        //set the y of the right joystick to control the right side drive motors
        LFMP = y1;
        RFMP = y2;
        LBMP = y1;
        RBMP = y2;
        TelemetryD("y1", y1);
        TelemetryD("y2", y2);
        TelemetryD("strafe", strafe);
        if (Math.abs(strafe) > margin) {
            LFMP = strafe;
            RFMP = -strafe;
            LBMP = -strafe;
            RBMP = strafe;
        }
        setDrive();
    }

    private void povDrive(double drive, double strafe, double turn) {
        //make left joystick control forwards and backwards, as well as strafing
        //make right joystick control turning of the robot
        LFMP = drive + turn;
        RFMP = drive - turn;
        LBMP = drive + turn;
        RBMP = drive - turn;
        TelemetryD("drive", drive);
        TelemetryD("turn", turn);
        TelemetryD("strafe", strafe);
        if (Math.abs(strafe) > margin) {
            LFMP = strafe;
            RFMP = -strafe;
            LBMP = -strafe;
            RBMP = strafe;
        }
        setDrive();
    }

    private void motorTest(boolean LFM, boolean RFM, boolean LBM, boolean RBM) {
        //debug drive mode runs one motor at a time to diagnose possible motor issues
        if (LFM) {
            TelemetryS("LFM", "Running");
            LFMP = 1;
            setDrive();
        } else if (RFM) {
            TelemetryS("RFM", "Running");
            RFMP = 1;
            setDrive();
        } else if (LBM) {
            TelemetryS("LBM", "Running");
            LBMP = 1;
            setDrive();
        } else if (RBM) {
            TelemetryS("RBM", "Running");
            RBMP = 1;
            setDrive();
        }
        LFMP = 0;
        RFMP = 0;
        LBMP = 0;
        RBMP = 0;

    }

    private void scale() {
        //slow mode
        if (!gamepad1.left_bumper) {
            LFMP *= 0.5;
            RFMP *= 0.5;
            LBMP *= 0.5;
            RBMP *= 0.5;
        }
        //ensures motor powers never exceed one
        List<Double> list = Arrays.asList(Math.abs(LFMP), Math.abs(RFMP), Math.abs(LBMP), Math.abs(RBMP));
        double max = Collections.max(list);
        if (max > 1) {
            LFMP /= max;
            RFMP /= max;
            LBMP /= max;
            RBMP /= max;
        }
    }

    private void setDrive() {
        scale();
        TelemetryD("LFMP", LFMP);
        TelemetryD("RFMP", RFMP);
        TelemetryD("LBMP", LBMP);
        TelemetryD("RBMP", RBMP);
        //set the drive-train motors to a certain power
        robot.LFM.setPower(LFMP);
        robot.RFM.setPower(RFMP);
        robot.LBM.setPower(LBMP);
        robot.RBM.setPower(RBMP);
    }

    private void intake() {
        if (gamepad2.a) { //intake
            LIP = 1;
            RIP = 1;
        } else if (gamepad2.b) { //push out
            LIP = -1;
            RIP = -1;
        } else { //stop motors if neither a nor b are being pushed
            LIP = 0;
            RIP = 0;
        }
        setIntake();
        if (gamepad1.dpad_up) {
            robot.Cap.setPosition(0.95);
        } else if (gamepad1.dpad_down) {
            robot.Cap.setPosition(robot.Cap.getPosition() - 0.01);
        }
        if (gamepad2.dpad_up) {
            servoPos(robot.LFMo, 0.69);
            servoPos(robot.RFMo, 0.36);
        } else if (gamepad2.dpad_down) {
            servoPos(robot.LFMo, 0.07);
            servoPos(robot.RFMo, 0.97);
        }
        TelemetryD("RFMo", robot.RFMo.getPosition());
        TelemetryD("LFMo", robot.LFMo.getPosition());
    }

    private void setIntake() {
        TelemetryD("LIP", LIP);
        TelemetryD("RIP", RIP);
        //set the intake motors to a certain power
        robot.RI.setPower(RIP);
        robot.LI.setPower(LIP);
    }

    private void output(double rx, double ly) {
        if (Math.abs(rx) < margin) {
            rx = 0.02;
        }
        if (rx < 0) {
            rx /= 5;
        }
        ramp(rx, robot.B);
        BP = MP;
        setOutput();

        if (gamepad2.left_bumper) {
            clampPos = 0;
        } else if (gamepad2.right_bumper) {
            clampPos = 0.8;
        }
        TelemetryD("clampPos", clampPos);
        TelemetryD("Clamp Position", robot.BClamp.getPosition());
        servoPos(robot.BClamp, clampPos);

        if (Math.abs(ly) > margin) {
            if (ly > 0) {
                rotPos = 0;
            } else if (ly < 0) {
                rotPos = 1;
            }
        }
        TelemetryD("rotPos", rotPos);
        TelemetryD("Rotation Position", robot.BRot.getPosition());
        servoPos(robot.BRot, rotPos);
    }

    private void ramp(double MT, DcMotor motor) {
        MP = motor.getPower();

        MT /= 1.5;

        if (MP != MT) {
            MP += (MT - MP) / 6;
        }
        TelemetryD("MT", MT);
        TelemetryD("MP", MP);
    }

    private void setOutput() {
        TelemetryD("B", BP);
        //set the output motors to a certain motor
        robot.B.setPower(BP);
    }

    private void servoPos(Servo servo, double position) {
        //init LFMo - 0.69, RFMo - 0.35
        //skystone LFMo - 0.15, RFMo - 0.90
        //foundation LFMo - 0.09, RFMo - 0.97
        servo.setPosition(position);
    }

    private void getImu() {
        //define variables with imu input
        //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    private void TelemetryS(String caption, String message) {
        telemetry.addLine(caption + ": " + message);
    }

    private void TelemetryD(String caption, double value) {
        telemetry.addLine(caption + ": " + value);
    }

    private void TelemetryB(String caption, boolean tf) {
        telemetry.addLine(caption + ": " + tf);
    }

    private void TelemetryI(String caption, int value) {
        telemetry.addLine(caption + ": " + value);
    }

    /*
    Controls:

        Gamepad 1:


        Gamepad 2:


     */
}
