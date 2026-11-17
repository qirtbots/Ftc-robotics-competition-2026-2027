package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Design 1 - RC Drive", group = "Design 1")
public class Design1TeleOp extends LinearOpMode {

    private DcMotor leftmotor;
    private DcMotor rightmotor;
    private Servo leftsteering;
    private Servo rightsteering;

    private static final double WHEELBASE = 12.0; // in
    private static final double TRACK_WIDTH = 10.0; // in
    private static final double MAX_STEER_ANGLE = 30; // degrees

    private static final double LEFT_CENTER = 0.50;
    private static final double RIGHT_CENTER = 0.50;

    private static final double LEFT_SERVO_RANGE = 0.08;
    private static final double RIGHT_SERVO_RANGE = 0.08;

    private static final double LEFT_SERVO_SIGN = 1.0;
    private static final double RIGHT_SERVO_SIGN = -1.0;

    private static final double DEADZONE = 0.05;
    private static final double NORMAL_SPEED = 0.65;
    private static final double PRECISION_SPEED = 0.30;
    private static final double TURBO_SPEED = 1.0;

    private static final double ACCELERATION_RATE = 2.0;

    private double currentPower = 0.0;

    @Override
    public void runOpMode() {
        leftmotor = hardwareMap.get(DcMotor.class, "leftmotor");
        rightmotor = hardwareMap.get(DcMotor.class, "rightmotor");

        leftsteering = hardwareMap.get(Servo.class, "leftsteering");
        rightsteering = hardwareMap.get(Servo.class, "rightsteering");

        rightmotor.setDirection(DcMotor.Direction.REVERSE);
        leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        centerSteering();

        telemetry.addLine("Design1 Initialized");
        telemetry.addLine("RC style ackerman drive");
        telemetry.update();

        waitForStart();
        resetRuntime();

        double previousTime = getRuntime();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double deltaTime = currentTime - previousTime;
            previousTime = currentTime;

            double throttle = -gamepad1.left_stick_y;
            double steering = gamepad1.right_stick_x;

            throttle = applyDeadzone(throttle);
            steering = applyDeadzone(steering);

            double speedMultiplier = NORMAL_SPEED;

            if (gamepad1.left_trigger > 0.5) {
                speedMultiplier = PRECISION_SPEED;
            }

            if (gamepad1.right_trigger > 0.5) {
                speedMultiplier = TURBO_SPEED;
            }

            double targetPower = throttle * speedMultiplier;

            if (gamepad1.b) {
                currentPower = 0.0;

                leftmotor.setPower(0);
                rightmotor.setPower(0);

                centerSteering();
            } else {
                double maxChange = ACCELERATION_RATE * deltaTime;
                double difference = targetPower - currentPower;

                difference = clip(difference, -maxChange, maxChange);

                currentPower += difference;

                leftmotor.setPower(currentPower);
                rightmotor.setPower(currentPower);

                if (gamepad1.a) {
                    centerSteering();
                } else {
                    updateSteering(steering);
                }
            }

            telemetry.addData("Throttle", "%.2f", throttle);
            telemetry.addData("Steering", "%.2f", steering);
            telemetry.addData("Motor Power", "%.2f", currentPower);
            telemetry.addData("Speed Mode", "%.2f", speedMultiplier);
            telemetry.addData("Emergency Stop", gamepad1.b);
            telemetry.update();
        }

        leftmotor.setPower(0);
        rightmotor.setPower(0);
        centerSteering();
    }

    private void updateSteering(double input) {
        double maxAngleRad = Math.toRadians(MAX_STEER_ANGLE);
        double steerAngle = input * maxAngleRad;

        if (Math.abs(steerAngle) < 0.001) {
            centerSteering();
            return;
        }

        // Ackerman geometry calculations
        double turningRadius = WHEELBASE / Math.tan(Math.abs(steerAngle));

        double leftAngle, rightAngle;
        if (steerAngle > 0) {
            // Turning right: left wheel is on the outside, right wheel is on the inside
            leftAngle = Math.atan(WHEELBASE / (turningRadius - (TRACK_WIDTH / 2.0)));
            rightAngle = Math.atan(WHEELBASE / (turningRadius + (TRACK_WIDTH / 2.0)));
        } else {
            // Turning left: right wheel is on the outside, left wheel is on the inside
            leftAngle = Math.atan(WHEELBASE / (turningRadius + (TRACK_WIDTH / 2.0)));
            rightAngle = Math.atan(WHEELBASE / (turningRadius - (TRACK_WIDTH / 2.0)));
            leftAngle = -leftAngle;
            rightAngle = -rightAngle;
        }

        // Convert angles to servo positions
        double leftPos = LEFT_CENTER + (LEFT_SERVO_SIGN * (leftAngle / maxAngleRad) * LEFT_SERVO_RANGE);
        double rightPos = RIGHT_CENTER + (RIGHT_SERVO_SIGN * (rightAngle / maxAngleRad) * RIGHT_SERVO_RANGE);

        leftsteering.setPosition(clip(leftPos, 0.0, 1.0));
        rightsteering.setPosition(clip(rightPos, 0.0, 1.0));
    }

    private void centerSteering() {
        leftsteering.setPosition(LEFT_CENTER);
        rightsteering.setPosition(RIGHT_CENTER);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0.0 : value;
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}