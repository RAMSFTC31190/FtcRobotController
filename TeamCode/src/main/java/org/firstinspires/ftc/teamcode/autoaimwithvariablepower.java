package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range; // Import Range for safety clamping

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import java.util.Locale;

@TeleOp(name = "DecodeTeleop_AutoAim_AutoPower", group = "Competition")
public class autoaimwithvariablepower extends LinearOpMode {

    GoBildaPinpointDriver odo;
    HuskyLens huskyLens;

    // --- TUNING VARIABLES FOR AUTO-POWER ---
    // Measure these on the field using Telemetry!
    // HEIGHT is in pixels (0 to 240). POWER is 0.0 to 1.0.
    double closeHeight = 60.0; // The pixel height of the tag when you are CLOSE
    double closePower  = 0.6;  // The motor power needed to make the shot from CLOSE

    double farHeight   = 20.0; // The pixel height of the tag when you are FAR
    double farPower    = 0.95; // The motor power needed to make the shot from FAR

    // Live variable to hold the calculated power
    double autoLauncherPower = 0.7; // Default safe speed

    @Override
    public void runOpMode() throws InterruptedException {
        // --- HARDWARE MAPPING ---
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        DcMotor leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        DcMotor rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        Servo frontPusher = hardwareMap.get(Servo.class, "frontPusher");
        Servo ballStopper = hardwareMap.get(Servo.class, "ballStopper");
        DcMotor belt = hardwareMap.get(DcMotor.class, "belt");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // --- MOTOR SETUP ---
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        belt.setDirection(DcMotor.Direction.REVERSE);

        double slowSpeed = 0.7;
        double fastSpeed = 0.9;

        // --- PINPOINT SETUP ---
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Ready. Use Gamepad 2 'X' for Auto-Power shots.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();

            // --- 1. ALWAYS READ HUSKYLENS (To calculate power constantly) ---
            HuskyLens.Block[] blocks = huskyLens.blocks();
            HuskyLens.Block target = null;

            for (HuskyLens.Block block : blocks) {
                if (block.id == 1) {
                    target = block;
                    break;
                }
            }

            // --- 2. CALCULATE AUTO-POWER ---
            if (target != null) {
                double currentHeight = target.height;

                // MATH: Linear Interpolation (Point-Slope Form)
                // As height gets smaller (further away), power increases.
                double slope = (farPower - closePower) / (farHeight - closeHeight);
                autoLauncherPower = closePower + (slope * (currentHeight - closeHeight));

                // Safety Clamp: Keep power between 0 and 1
                autoLauncherPower = Range.clip(autoLauncherPower, 0.0, 1.0);

                telemetry.addData("Tag Found", "ID: " + target.id);
                telemetry.addData("Tag Height (Pixels)", currentHeight);
                telemetry.addData("Calculated Power", "%.2f", autoLauncherPower);
            } else {
                telemetry.addData("Tag", "Not Visible - Using Default Power");
            }

            // --- 3. DRIVER CONTROLS (AUTO AIM) ---
            double y = gamepad1.right_stick_y;
            double x = -gamepad1.right_stick_x;
            double rx = -gamepad1.left_stick_x;

            if (gamepad1.left_bumper && target != null) {
                double targetX = target.x;
                double error = 160 - targetX;
                double Kp = 0.005;
                rx = error * Kp;
                rx = Range.clip(rx, -0.5, 0.5);
            }

            if (gamepad1.start) odo.resetPosAndIMU();

            Pose2D pos = odo.getPosition();
            double botHeading = pos.getHeading(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftMotor.setPower((rotY + rotX + rx) / denominator);
            backLeftMotor.setPower((rotY - rotX + rx) / denominator);
            frontRightMotor.setPower((rotY - rotX - rx) / denominator);
            backRightMotor.setPower((rotY + rotX - rx) / denominator);

            // --- 4. LAUNCHER LOGIC (UPDATED) ---
            if (gamepad2.a) {
                // Manual Slow
                rightLauncher.setPower(slowSpeed);
                leftLauncher.setPower(slowSpeed);
            } else if (gamepad2.y) {
                // Manual Fast
                rightLauncher.setPower(fastSpeed);
                leftLauncher.setPower(fastSpeed);
            } else if (gamepad2.x) {
                // [NEW] AUTO POWER MODE
                rightLauncher.setPower(autoLauncherPower);
                leftLauncher.setPower(autoLauncherPower);
            } else {
                rightLauncher.setPower(0);
                leftLauncher.setPower(0);
            }

            // --- SERVO & INTAKE ---
            if (gamepad2.b) {
                frontPusher.setPosition(0.6);
            } else {
                frontPusher.setPosition(0.52);
            }

            if (gamepad2.left_bumper) {
                ballStopper.setPosition(0.5);
            } else {
                ballStopper.setPosition(0.1);
            }

            if (gamepad2.right_bumper) {
                intake.setPower(-1);
                belt.setPower(.8);
            } else {
                intake.setPower(0);
                belt.setPower(0);
            }

            telemetry.addData("Heading", String.format(Locale.US, "%.1f Deg", pos.getHeading(AngleUnit.DEGREES)));
            telemetry.update();
        }
    }
}