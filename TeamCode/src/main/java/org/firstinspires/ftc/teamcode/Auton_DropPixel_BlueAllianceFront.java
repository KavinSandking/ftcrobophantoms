package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@Autonomous(name="Auton_DropPixel_BlueAllianceFront", group="Robot")
//@Disabled
public class Auton_DropPixel_BlueAllianceFront extends LinearOpMode {
    private DistanceSensor sensorleft;
    private DistanceSensor sensorright;
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    //DcMotor Intake = null;
    DcMotor arm = null;
    CRServo racpServo;
    private ElapsedTime runtime = new ElapsedTime();
    double racpPower = 0.2;
    double armPower = 0.3;
    double motor_ticks_count = 728;
    double turn;
    double desiredPosition = 10;
    int newTarget;
    double valueleft = 300;
    double valuemiddle = 300;
    double valueleftmeasure;
    double distanceThreshold = 24;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();
        //calibration
        Turn(-0.3,0.3);
        driveBackward(0.2,0.7);
        stopDriveMotors();
        valueleftmeasure = sensorleft.getDistance(DistanceUnit.INCH);

        if(valueleftmeasure< distanceThreshold) {
            valueleft = valueleftmeasure;
            Turn(0.3, 0.3);
            driveForward(0.2, 0.7);
            stopDriveMotors();
        }
        else{
            Turn(0.3,0.3);
            driveForward(0.2,0.7);
            stopDriveMotors();
            valueleftmeasure = sensorleft.getDistance(DistanceUnit.INCH);
        }
        if(valueleftmeasure< distanceThreshold){
            valuemiddle=valueleftmeasure;
        }
        if(valueleft>1 && valueleft< distanceThreshold){
            driveBackward(0.15,5);
            strafeRight(0.2,1.2);
            stopDriveMotors();
            runtime.reset();
            Arm();
            driveForward(0.15,1);


        }
    }

    private void initializeHardware() {
        sensorleft = hardwareMap.get(DistanceSensor.class, "sensor_left");
        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");

        arm = hardwareMap.dcMotor.get("arm");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        racpServo = hardwareMap.crservo.get("grabberServo");
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
    private void Bucket(double power, double seconds) {
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
            telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
            telemetry.update();
        }
    }
    private void stopDriveMotors() {
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }
}