package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Design 1 - RC Drive V2", group = "Design 1")

public class Design1TeleOpV2simplified extends LinearOpMode {

// ==========================================
// HARDWARE
// ==========================================

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private Servo leftSteering;
    private Servo rightSteering;

// ==========================================
// ROBOT DIMENSIONS
// ==========================================

// Provisional dimensions in inches.
// Replace these with actual measurements.

    private static final double WHEELBASE = 12.0;
    private static final double TRACK_WIDTH = 10.0;

    private static final double MAX_STEER_ANGLE = 30.0;

// ==========================================
// STEERING CALIBRATION
// ==========================================

    private static final double LEFT_CENTER = 0.50;
    private static final double RIGHT_CENTER = 0.50;

    private static final double LEFT_SERVO_RANGE = 0.08;
    private static final double RIGHT_SERVO_RANGE = 0.08;

    private static final double LEFT_SERVO_SIGN = 1.0;
    private static final double RIGHT_SERVO_SIGN = -1.0;

// ==========================================
// DRIVING SETTINGS
// ==========================================

    private static final double DEADZONE = 0.05;

    private static final double NORMAL_SPEED = 0.65;
    private static final double PRECISION_SPEED = 0.30;
    private static final double TURBO_SPEED = 1.0;

    // Power change per second.
    private static final double ACCELERATION_RATE = 2.0;

    private static final double MIN_LOOP_TIME = 0.001;
    private static final double MAX_LOOP_TIME = 0.100;

    // Independent motor power.
    private double leftPower = 0.0;
    private double rightPower = 0.0;

    private String drivingMode = "NORMAL";

// ==========================================
// MAIN PROGRAM
// ==========================================

    @Override
    public void runOpMode() {

// Hardware mapping.
        leftMotor = hardwareMap.get(
                DcMotor.class, "leftMotor");

        rightMotor = hardwareMap.get(
                DcMotor.class, "rightMotor");

        leftSteering = hardwareMap.get(
                Servo.class, "leftSteering");

        rightSteering = hardwareMap.get(
                Servo.class, "rightSteering");

// Reverse right motor based on mounting orientation.
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

// Configure motor stopping behavior.
        leftMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        centerSteering();

        telemetry.addLine("DESIGN 1 - V2 READY");
        telemetry.addLine("R1: LEFT WHEEL ONLY");
        telemetry.addLine("L1: RIGHT WHEEL ONLY");
        telemetry.addLine("Both or neither: NORMAL");
        telemetry.update();

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();

        double previousTime = runtime.seconds();

        while (opModeIsActive()) {

// ==================================
// LOOP TIMING
// ==================================

            double currentTime = runtime.seconds();

            double deltaTime = currentTime - previousTime;
            previousTime = currentTime;

            deltaTime = clip(
                    deltaTime,
                    MIN_LOOP_TIME,
                    MAX_LOOP_TIME
            );

// ==================================
// CONTROLLER INPUT
// ==================================

            double throttle = -gamepad1.left_stick_y;
            double steering = gamepad1.right_stick_x;

            throttle = applyDeadzone(throttle);
            steering = applyDeadzone(steering);

// ==================================
// SPEED MODES
// ==================================

            double speedMultiplier = NORMAL_SPEED;

            if (gamepad1.left_trigger > 0.5) {
                speedMultiplier = PRECISION_SPEED;
            }

            if (gamepad1.right_trigger > 0.5) {
                speedMultiplier = TURBO_SPEED;
            }

            double targetPower = throttle * speedMultiplier;

// ==================================
// INDEPENDENT WHEEL SELECTION
// ==================================

            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;

            double targetLeft = targetPower;
            double targetRight = targetPower;

            if (rightBumper && !leftBumper) {

// R1: Left wheel only.
                targetRight = 0.0;
                drivingMode = "LEFT ONLY";

            } else if (leftBumper && !rightBumper) {

// L1: Right wheel only.
                targetLeft = 0.0;
                drivingMode = "RIGHT ONLY";

            } else {

// Both bumpers or neither.
                drivingMode = "NORMAL";
            }

// ==================================
// STOP CONTROL
// ==================================

            if (gamepad1.b) {

                leftPower = 0.0;
                rightPower = 0.0;

                stopMotors();
                centerSteering();

                drivingMode = "STOPPED";

            } else {

// ==================================
// DIFFERENTIAL TURNING
// ==================================

                double[] wheelFactors =
                        calculateDifferentialFactors(steering);

                targetLeft *= wheelFactors[0];
                targetRight *= wheelFactors[1];

// ==================================
// INDEPENDENT ACCELERATION
// ==================================

                if (rightBumper && !leftBumper) {

// Stop unselected right motor.
                    rightPower = 0.0;

                    leftPower = rampPower(
                            leftPower,
                            targetLeft,
                            deltaTime
                    );

                } else if (leftBumper && !rightBumper) {

// Stop unselected left motor.
                    leftPower = 0.0;

                    rightPower = rampPower(
                            rightPower,
                            targetRight,
                            deltaTime
                    );

                } else {

// Both motors accelerate independently.
                    leftPower = rampPower(
                            leftPower,
                            targetLeft,
                            deltaTime
                    );

                    rightPower = rampPower(
                            rightPower,
                            targetRight,
                            deltaTime
                    );
                }

// ==================================
// MOTOR COMMANDS
// ==================================

                boolean leftInactive =
                        leftBumper && !rightBumper;

                boolean rightInactive =
                        rightBumper && !leftBumper;

                commandMotor(
                        leftMotor,
                        leftPower,
                        leftInactive
                );

                commandMotor(
                        rightMotor,
                        rightPower,
                        rightInactive
                );

// ==================================
// STEERING
// ==================================

                if (gamepad1.a) {
                    centerSteering();
                } else {
                    updateSteering(steering);
                }
            }

// ==================================
// TELEMETRY
// ==================================

            telemetry.addData("Mode", drivingMode);

            telemetry.addData(
                    "Left Motor Power",
                    "%.2f", leftPower);

            telemetry.addData(
                    "Right Motor Power",
                    "%.2f", rightPower);

            telemetry.addData(
                    "Throttle",
                    "%.2f", throttle);

            telemetry.addData(
                    "Steering",
                    "%.2f", steering);

            telemetry.addData("R1 Held", rightBumper);
            telemetry.addData("L1 Held", leftBumper);

            telemetry.update();
        }

// Stop when OpMode exits.
        stopMotors();
        centerSteering();
    }

// ==========================================
// MOTOR CONTROL
// ==========================================

    private void commandMotor(
            DcMotor motor,
            double power,
            boolean inactive) {

        if (inactive) {

            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.FLOAT);

            motor.setPower(0.0);

        } else {

            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE);

            motor.setPower(power);
        }
    }

    private void stopMotors() {

        leftMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

// ==========================================
// ACCELERATION / DECELERATION
// ==========================================

    private double rampPower(
            double current,
            double target,
            double deltaTime) {

        double maxChange = ACCELERATION_RATE * deltaTime;

        double difference = target - current;

        difference = clip(
                difference,
                -maxChange,
                maxChange
        );

        return current + difference;
    }

// ==========================================
// DIFFERENTIAL TURNING
// ==========================================

    private double[] calculateDifferentialFactors(
            double steeringInput) {

        double angle = Math.abs(steeringInput)
                * Math.toRadians(MAX_STEER_ANGLE);

        if (angle < 0.001) {
            return new double[]{1.0, 1.0};
        }

        double radius = WHEELBASE / Math.tan(angle);

        double halfTrack = TRACK_WIDTH / 2.0;

        double innerFactor = Math.max(
                0.0,
                (radius - halfTrack) / radius
        );

        double outerFactor =
                (radius + halfTrack) / radius;

        innerFactor /= outerFactor;

        double leftFactor;
        double rightFactor;

        if (steeringInput > 0) {

// Right turn: left wheel is outside.
            leftFactor = 1.0;
            rightFactor = innerFactor;

        } else {

// Left turn: right wheel is outside.
            leftFactor = innerFactor;
            rightFactor = 1.0;
        }

        return new double[]{leftFactor, rightFactor};
    }

// ==========================================
// ACKERMANN STEERING
// ==========================================

    private void updateSteering(double input) {

        double maxAngle =
                Math.toRadians(MAX_STEER_ANGLE);

        double angle = Math.abs(input) * maxAngle;

        if (angle < 0.001) {
            centerSteering();
            return;
        }

        double radius = WHEELBASE / Math.tan(angle);

        double innerAngle = Math.atan(
                WHEELBASE / (radius - TRACK_WIDTH / 2.0)
        );

        double outerAngle = Math.atan(
                WHEELBASE / (radius + TRACK_WIDTH / 2.0)
        );

        double leftAngle;
        double rightAngle;

        if (input > 0) {

            leftAngle = outerAngle;
            rightAngle = innerAngle;

        } else {

            leftAngle = -innerAngle;
            rightAngle = -outerAngle;
        }

        double maxRadius =
                WHEELBASE / Math.tan(maxAngle);

        double maxInnerAngle = Math.atan(
                WHEELBASE / (maxRadius - TRACK_WIDTH / 2.0)
        );

        double leftPosition = LEFT_CENTER
                + LEFT_SERVO_SIGN
                * (leftAngle / maxInnerAngle)
                * LEFT_SERVO_RANGE;

        double rightPosition = RIGHT_CENTER
                + RIGHT_SERVO_SIGN
                * (rightAngle / maxInnerAngle)
                * RIGHT_SERVO_RANGE;

        leftSteering.setPosition(
                clip(leftPosition, 0.0, 1.0));

        rightSteering.setPosition(
                clip(rightPosition, 0.0, 1.0));
    }

// ==========================================
// CENTER STEERING
// ==========================================

    private void centerSteering() {

        leftSteering.setPosition(LEFT_CENTER);
        rightSteering.setPosition(RIGHT_CENTER);
    }

// ==========================================
// INPUT DEADZONE
// ==========================================

    private double applyDeadzone(double input) {

        if (Math.abs(input) < DEADZONE) {
            return 0.0;
        }

        return input;
    }

// ==========================================
// VALUE LIMITING
// ==========================================

    private double clip(
            double value,
            double min,
            double max) {

        return Math.max(min, Math.min(max, value));
    }
}