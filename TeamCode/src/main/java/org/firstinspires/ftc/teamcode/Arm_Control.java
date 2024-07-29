package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "teleop_robot")
public class Arm_Control extends OpMode {


    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    DcMotor Intake = null;
    DcMotor arm = null;
    CRServo contServo;
    CRServo rackpServo;


    double armpower;
    boolean armMoving = false;
    long armStartTime = 0;
    final long ARM_MOVEMENT_TIME = 2000; // Time in milliseconds for arm movement


    @Override
    public void init() {

        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");
        Intake = hardwareMap.dcMotor.get("Intake");
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        arm = hardwareMap.dcMotor.get("arm");
        contServo = hardwareMap.crservo.get("grabberServo");
        rackpServo = hardwareMap.crservo.get("rackpinnionServo");
    }

    @Override
    public void loop() {
        // Drive control
        if (gamepad1.left_stick_y != 0.0) {
            FrontRight.setPower(-gamepad1.left_stick_y);
            BackLeft.setPower(-gamepad1.left_stick_y);
        } else {
            FrontRight.setPower(0);
            BackLeft.setPower(0);
        }

        if (gamepad1.right_stick_y != 0.0) {
            FrontLeft.setPower(-gamepad1.right_stick_y);
            BackRight.setPower(-gamepad1.right_stick_y);
        } else {
            FrontLeft.setPower(0);
            BackRight.setPower(0);
        }

        if (gamepad1.left_bumper) {
            FrontLeft.setPower(-0.5);
            BackLeft.setPower(-0.5);
            FrontRight.setPower(0.5);
            BackRight.setPower(0.5);
        } else if (gamepad1.right_bumper) {
            FrontLeft.setPower(0.5);
            BackLeft.setPower(0.5);
            FrontRight.setPower(-0.5);
            BackRight.setPower(-0.5);
        }


        if (gamepad2.right_stick_y != 0.0) {
            Intake.setPower(gamepad2.right_stick_y);
        } else {
            Intake.setPower(0.);
        }


        if (gamepad2.left_stick_y != 0.0) {
            armpower = Range.clip(gamepad2.left_stick_y, -0.35, 0.35);
            arm.setPower(armpower);
            armMoving = true; // Arm is moving
            armStartTime = System.currentTimeMillis(); // Record start time
        } else if (armMoving && (System.currentTimeMillis() - armStartTime < ARM_MOVEMENT_TIME)) {
            // If the arm was moving and hasn't reached ARM_MOVEMENT_TIME yet, keep it powered
            arm.setPower(armpower);
        } else {
            arm.setPower(0.0); // Stop the arm
            armMoving = false; // Arm is no longer moving
        }


        if (gamepad2.y) {
            contServo.setPower(0.);
            rackpServo.setPower(0.);
        } else if (gamepad2.a) {
            contServo.setPower(0.2);
        } else if (gamepad2.b) {
            contServo.setPower(-0.2);
        }

        if (gamepad2.left_bumper) {
            rackpServo.setPower(0.9);
        } else if (gamepad2.right_bumper) {
            rackpServo.setPower(-0.9);
        }

    }
}

