package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Hex Core Auto", group="autonomous")
public class HexcoreAutonomous extends LinearOpMode {
    private DcMotor hexMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        hexMotor = hardwareMap.get(DcMotor.class, "hex_motor");

        hexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("status", "initialized.ready to start!");
        telemetry.update();

        waitForStart();

        // The entire autonomous routine now stays safe inside this block
        if (opModeIsActive()) {
            // Step 1: Spin forward
            telemetry.addData("Motor status", "spinning forward");
            telemetry.update();
            hexMotor.setPower(0.5);
            sleep(1000);

            // Step 2: Pause
            telemetry.addData("motor status", "paused");
            telemetry.update();
            hexMotor.setPower(0.0);
            sleep(1000);

            // Step 3: Spin backward
            telemetry.addData("motor status", "spinning backward");
            telemetry.update();
            hexMotor.setPower(-0.5);
            sleep(2000);

            // Step 4: Stop
            hexMotor.setPower(0.0);
            telemetry.addData("status", "autonomous complete");
            telemetry.update();
        } // <-- Moved the closing brace here to safely wrap all your actions!
    }
}
