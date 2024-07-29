package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "PID_Controller", group = "Robot")
public class PID_Controller extends LinearOpMode {
    private DcMotor arm;
    private CRServo bucketServo;
    private int arm_pos;

    @Override
    public void runOpMode() {
        bucketServo = hardwareMap.crservo.get("BucketServo");
        arm = hardwareMap.dcMotor.get("ArmMotor");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm_pos = 0;

        waitForStart();

        arm(1000, 0.5, 500);
        sleep(500);
        bucket(0.5, 500);
    }

    private void arm(int targetPosition, double armPower, long timeoutMillis) {
        arm_pos = targetPosition;
        arm.setTargetPosition(arm_pos);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);

        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && arm.isBusy()) {
            idle();

            if (System.currentTimeMillis() - startTime > timeoutMillis) {
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                telemetry.addLine("Timeout occurred while moving the arm");
                telemetry.update();
                return;
            }
        }


    }

    private void bucket(double bucketPower, long timeoutMillis) {
        long bucketEndTime = System.currentTimeMillis() + timeoutMillis;
        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() < bucketEndTime && opModeIsActive()) {
            bucketServo.setPower(bucketPower);

            if (System.currentTimeMillis() - startTime > timeoutMillis) {
                bucketServo.setPower(0);
                telemetry.addLine("Timeout occurred while moving the bucket");
                telemetry.update();
                return;
            }
        }
    }
}
