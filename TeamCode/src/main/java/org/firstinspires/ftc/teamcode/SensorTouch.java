package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "SensorTouch", group = "Robot")
public class SensorTouch extends LinearOpMode {
    TouchSensor touchSensor;
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    DcMotor arm = null;
    CRServo bucketServo;
    private ElapsedTime runtime = new ElapsedTime();
    double bucketPower = 0.2;
    double armPower = 0.3;
    double motor_ticks_count = 728;
    double turn;
    double desiredPosition = 10;
    int newTarget;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();
        runAutonomousProgram();

    }

    private void runAutonomousProgram(){
        //check if team prop is on the center tape or not
        driveBackward(0.15,4);
        //team prop is on the center tape
        if (touchSensor.isPressed()) {
            telemetry.addData("Touch Sensor", "Center Tape Pressed");
            driveBackward(0.15, 2);
            runtime.reset();
            Arm();
            Turn(0.5, 1);
            strafeRight(0.2, 2);
            sleep(1000);
              driveForward(0.5,1);
            Bucket(-0.3,0.5);
            strafeRight(0.5,1);
            stopDriveMotors();
            stopAllMotors();
            // team prop is on the right tape
        } else {
            telemetry.addData("Touch Sensor", "On the right tape");
            runtime.reset();
            Arm();
            strafeRight(0.5,0.1);
            Turn(0.5,1);
            driveForward(0.5,1);
            sleep(1000);
            Bucket(-0.3,0.5);
            Bucket(0.3,0.1);
            strafeRight(0.5,1);
            stopDriveMotors();
            stopAllMotors();

        }

        telemetry.update();
    }
    private void initializeHardware(){
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");

        arm = hardwareMap.dcMotor.get("arm");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        bucketServo = hardwareMap.crservo.get("grabberServo");
    }
    private void driveForward(double power, double seconds) {
        FrontLeft.setPower(power);
        BackLeft.setPower(power);
        FrontRight.setPower(power);
        BackRight.setPower(power);
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    private void driveBackward(double power, double seconds) {
        FrontLeft.setPower(-power);
        BackLeft.setPower(-power);
        FrontRight.setPower(-power);
        BackRight.setPower(-power);
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    private void strafeLeft(double power, double seconds) {
        FrontLeft.setPower(power);
        BackLeft.setPower(power);
        FrontRight.setPower(-power);
        BackRight.setPower(-power);
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    private void strafeRight(double power, double seconds) {
        FrontLeft.setPower(-power);
        BackLeft.setPower(-power);
        FrontRight.setPower(power);
        BackRight.setPower(power);
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    private void Turn(double power, double seconds) {
        FrontLeft.setPower(-power);
        BackLeft.setPower(power);
        FrontRight.setPower(power);
        BackRight.setPower(-power);
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    private void Bucket(double power,double seconds) {
        bucketServo.setPower(bucketPower);
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    private void Arm(){
        arm.setPower(armPower);
        turn = motor_ticks_count/4;

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = arm.getCurrentPosition() + (int)turn;
        arm.setTargetPosition(newTarget);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && arm.isBusy()) {
            telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed");
            telemetry.update();
        }
    }
    private void stopDriveMotors() {
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }
    private void stopAllMotors() {
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
        bucketServo.setPower(0);
        arm.setPower(0);
    }
}


