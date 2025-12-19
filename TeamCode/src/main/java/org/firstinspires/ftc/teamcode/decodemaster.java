package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// --- PINPOINT IMPORTS ---
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(name = "DecodeTeleop", group = "Competition")
public class decodemaster extends LinearOpMode {

    GoBildaPinpointDriver odo;

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

        // --- MOTOR SETUP ---
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- MOTOR DIRECTIONS ---
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Aux Motors
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        belt.setDirection(DcMotor.Direction.REVERSE);

        // --- LAUNCHER SPEED VARIABLES ---
        double slowSpeed = 0.7;
        double fastSpeed = 0.9;

        // =========================================================
        // --- PINPOINT ODOMETRY SETUP ---
        // =========================================================
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");

        // STEP 1: Set the Offset from the center of the robot
        // MEASURE THESE ON YOUR ROBOT! (In Millimeters)
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);

        // STEP 2: Set the Encoder type
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // STEP 3: Set Direction.
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        // =========================================================

        telemetry.addData("Status", "Initialized. Ready to Run.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- 1. CRITICAL: UPDATE THE ODOMETRY COMPUTER ---
            odo.update();

            // --- GAMEPAD INPUTS ---
            double y = gamepad1.right_stick_y;
            double x = -gamepad1.right_stick_x;
            double rx = -gamepad1.left_stick_x;

            // --- RESET LOGIC ---
            if (gamepad1.start) {
                odo.resetPosAndIMU();
            }

            // --- FIELD CENTRIC MATH ---
            Pose2D pos = odo.getPosition();
            double botHeading = pos.getHeading(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            // --- POWER CALCULATIONS ---
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // --- LAUNCHER LOGIC ---
            if (gamepad2.a) {
                rightLauncher.setPower(slowSpeed);
                leftLauncher.setPower(slowSpeed);
            } else if (gamepad2.y) {
                rightLauncher.setPower(fastSpeed);
                leftLauncher.setPower(fastSpeed);
            } else {
                rightLauncher.setPower(0);
                leftLauncher.setPower(0);
            }

            // --- SERVO & INTAKE LOGIC ---
            if (gamepad2.b) {
                frontPusher.setPosition(0.6);
            } else {
                frontPusher.setPosition(0.52);
            }
            //ball stopper
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

            // --- TELEMETRY ---
            telemetry.addData("Field Location", String.format(Locale.US, "(X: %.1f, Y: %.1f)", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM)));
            telemetry.addData("Heading", String.format(Locale.US, "%.1f Deg", pos.getHeading(AngleUnit.DEGREES)));
            telemetry.update();
        } // End of While Loop
    } // End of runOpMode Method
} // End of Class (This was the missing one!)