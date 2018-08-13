package org.firstinspires.ftc.teamcode;

/**
 * Created by shuuzantake on 2018/3/4.
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotMap {
//    public static ColorSensor colorSensor = null;

    private HardwareMap hwmap = null;

    public static DcMotor leftFront = null;
    public static DcMotor rightFront = null;
    public static DcMotor leftRear = null;
    public static DcMotor rightRear = null;

    public static DcMotor leftBrush = null;
    public static DcMotor rightBrush = null;

    public static DcMotor liftMotor = null;
    public static DcMotor liftMotor2 = null;

    public static Servo upperServo = null;
    public static Servo rotateServo = null;

    public static ColorSensor colorSensor = null;

    private final double r_middle = 0.56;
    private final double r_left = 0.73;
    private final double r_right = 0.37;

    private final double u_up = 0.3;
    private final double u_down = 0.85;
    public static final double kDefaultDeadband = 0.02;
    public static final double kDefaultMaxOutput = 1.0;

    protected double m_deadband = kDefaultDeadband;
    protected double m_maxOutput = kDefaultMaxOutput;

    public void RobotInit(HardwareMap hardwareMap) {
        this.hwmap = hardwareMap;
        this.colorSensor = hwmap.colorSensor.get("color");

        this.leftFront = hwmap.dcMotor.get("leftfront");
        this.leftRear = hwmap.dcMotor.get("leftrear");
        this.rightFront = hwmap.dcMotor.get("rightfront");
        this.rightRear = hwmap.dcMotor.get("rightrear");

        this.leftBrush = hwmap.dcMotor.get("leftbrush");
        this.rightBrush = hwmap.dcMotor.get("rightbrush");

        this.liftMotor = hwmap.dcMotor.get("liftmotor");
        this.liftMotor2 = hwmap.dcMotor.get("liftmotor2");

        this.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.upperServo = hwmap.servo.get("upperservo");
        this.rotateServo = hwmap.servo.get("rotateservo");

        this.upperServo.setPosition(u_up);
    }

    public void Drivearcade(double xSpeed, double zRotation, boolean squaredInputs) {
        xSpeed = limit(xSpeed);
        xSpeed = applyDeadband(xSpeed, m_deadband);

        zRotation = limit(zRotation);
        zRotation = applyDeadband(zRotation, m_deadband);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squaredInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }

        this.leftRear.setPower(limit(leftMotorOutput) * m_maxOutput);
        this.leftFront.setPower(limit(leftMotorOutput) * m_maxOutput);
        this.rightRear.setPower(-limit(rightMotorOutput) * m_maxOutput);
        this.rightFront.setPower(-limit(rightMotorOutput) * m_maxOutput);
    }


    public void Cubeout() {
        leftBrush.setPower(1);
        rightBrush.setPower(-1);
    }

    public void Cubestop() {
        leftBrush.setPower(0);
        rightBrush.setPower(0);
    }

    public void LowerArmandTurnLeft() {
        this.upperServo.setPosition(u_down);
        delay(100);
        this.rotateServo.setPosition(r_left);
    }

    public void LowerArmandTurnRight() {
        this.upperServo.setPosition(u_down);
        delay(100);
        this.rotateServo.setPosition(r_right);
    }

    public void LowerArm() {
        this.upperServo.setPosition(u_down);
    }
    public void LiftArm() {
        this.upperServo.setPosition(u_up);
        this.rotateServo.setPosition(r_middle);
    }

    protected double applyDeadband ( double value, double deadband){
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    protected double limit(double value) {
        if (value > 1.0) {
            return 1.0;
        }
        if (value < -1.0) {
            return -1.0;
        }
        return value;
    }

    public static void delay(long minisecond) {
        try {
            Thread.currentThread().sleep(minisecond);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
