package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="ShellyOp")
public class ShellyOp extends OpMode {

    private BNO055IMU IMU;
    private Orientation orientation;

    private DcMotor frontMotor;
    private DcMotor backMotor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private double frontPower;
    private double leftPower;
    private double backPower;
    private double rightPower;
    private boolean directionCorrectOn;
    private int targetOrientation;

    private boolean FOD;

    private DcMotor arm;
    private Servo claw;

    private boolean clawOpened = false;
    private int targetArmPosition;

    @Override
    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        IMU = hardwareMap.get(BNO055IMU.class, "IMU");
        IMU.initialize(parameters);

        frontMotor = hardwareMap.get(DcMotor.class, "FrontMotor");
        backMotor = hardwareMap.get(DcMotor.class, "BackMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "LeftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "RightMotor");

        //arm = hardwareMap.get(DcMotor.class, "Claw");
        //claw = hardwareMap.get(Servo.class, "Arm");

        frontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    private void turn (double turnPower) {
        frontPower += turnPower;
        backPower -= turnPower;
        leftPower += turnPower;
        rightPower -= turnPower;
    }

    @Override
    public void loop() {
        double angle = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).thirdAngle;

        frontPower = gamepad1.left_stick_x * Math.sin(angle) + gamepad1.left_stick_y * Math.cos(angle);
        leftPower = gamepad1.left_stick_x * Math.cos(angle) + gamepad1.left_stick_y * Math.sin(angle);
        backPower = gamepad1.left_stick_x * Math.sin(angle) + gamepad1.left_stick_y * Math.cos(angle);
        rightPower = gamepad1.left_stick_x * Math.cos(angle) + gamepad1.left_stick_y * Math.sin(angle);

        if (gamepad1.right_stick_x != 0) {
            turn(gamepad1.right_stick_x);
        }

        /*if (gamepad2.left_stick_y == 0 && targetArmPosition == 0) {
            targetArmPosition = arm.getCurrentPosition();
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setPower(gamepad2.right_stick_y);
        } else {
            arm.setTargetPosition(targetArmPosition);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad2.right_bumper) {
            claw.setPosition(clawOpened ? Constants.CLAW_MIN : Constants.CLAW_MAX);
            clawOpened = !clawOpened;
        }*/
        if (gamepad1.dpad_up) {
            targetOrientation = 0;
            directionCorrectOn = !directionCorrectOn;
        }
        if (gamepad1.dpad_down) {
            targetOrientation = 180;
            directionCorrectOn = !directionCorrectOn;
        }
        if (gamepad1.dpad_left) {
            targetOrientation = -90;
            directionCorrectOn = !directionCorrectOn;
        }
        if (gamepad1.dpad_right) {
            targetOrientation = 90;
            directionCorrectOn = !directionCorrectOn;
        }
        if (directionCorrectOn) {
            if (angle > targetOrientation) {
                turn(-Constants.ORI_CORRECT_SPEED);
            } else {
                turn(Constants.ORI_CORRECT_SPEED);
            }
        }
        telemetry.addData("Rotation:", angle);
        frontMotor.setPower(frontPower);
        backMotor.setPower(backPower);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
}
