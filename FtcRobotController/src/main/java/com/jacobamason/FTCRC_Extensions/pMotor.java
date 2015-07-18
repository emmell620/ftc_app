package com.jacobamason.FTCRC_Extensions;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.LinkedList;

public class pMotor
{
    final private OpMode opMode;
    final private double wheelDiameter;
    final private double gearRatio;
    final private int encoderCountsPerDriverGearRotation = 1440;

    final private LinkedList<DcMotor> leftMotorsWithEncoders =
            new LinkedList<>();
    final private LinkedList<DcMotor> rightMotorsWithEncoders =
            new LinkedList<>();
    final private LinkedList<DcMotor> leftMotors = new LinkedList<>();
    final private LinkedList<DcMotor> rightMotors = new LinkedList<>();

    private double leftSpeed;
    private double rightSpeed;
    private long leftEncoderTarget;
    private long rightEncoderTarget;

    private int leftThreshold;
    private int rightThreshold;
    private int checkMotorsFrequency = 60;

    private class MotorStallException extends Exception
    {
        public MotorStallException(String errorMessage)
        {
            super(errorMessage);
        }
    }

    /**
     * .
     * <p/>
     * pMotor should be constructed at the top of the init() block. You
     * should send it 'this' as the first argument and then your primary left
     * and right drive motors. If you have more motors on your drive train,
     * add them after using pMotor.addLeftMotor(), pMotor.addRightMotor(),
     * pMotor.addLeftMotorWithEncoder(), and pMotor.addRightMotorWithEncoder() .
     * <p/>
     * Whichever unit of measurement you use for the wheelDiameter is the
     * same unit of measurement that will be used in any distance parameters.
     * So, for example, if you measure the wheel diameter in inches, the
     * distance traveled by the linear function will be in inches.
     * <p/>
     * It is important that you add all your drive train motors so that
     * pMotor will be able to control them all. It is also important that the
     * primary drive motors have encoders. For best results, the primary
     * motors should probably be the ones that have continuous contact with
     * the ground.
     *
     * @param opMode                Use the 'this' keyword as the argument here.
     * @param leftMotorWithEncoder  A left motor with an encoder.
     * @param rightMotorWithEncoder A right motor with an encoder.
     */
    public pMotor(OpMode opMode, DcMotor leftMotorWithEncoder,
                  DcMotor rightMotorWithEncoder, double wheelDiameter,
                  double gearRatio)
    {
        this.opMode = opMode;
        leftMotorsWithEncoders.add(leftMotorWithEncoder);
        rightMotorsWithEncoders.add(rightMotorWithEncoder);
        this.wheelDiameter = wheelDiameter;
        this.gearRatio = gearRatio;
    }

    public void addLeftMotor(DcMotor leftMotor)
    {
        leftMotors.add(leftMotor);
    }

    public void addLeftMotorWithEncoder(DcMotor leftMotor)
    {
        leftMotorsWithEncoders.add(leftMotor);
    }

    public void addRightMotor(DcMotor rightMotor)
    {
        rightMotors.add(rightMotor);
    }

    public void addRightMotorWithEncoder(DcMotor rightMotor)
    {
        rightMotorsWithEncoders.add(rightMotor);
    }

    @SuppressLint("Assert")
    public void setCheckMotorsFrequency(int frequency)
    {
        assert (frequency > 0);

        checkMotorsFrequency = frequency;
    }

    // function used to stop the drive train for the next movement, pausing
    // for one second.
    private void haltDrive()
    {
        for (DcMotor motor : leftMotorsWithEncoders)
        {
            motor.setPower(0);
        }

        for (DcMotor motor : rightMotorsWithEncoders)
        {
            motor.setPower(0);
        }

        for (DcMotor motor : leftMotors)
        {
            motor.setPower(0);
        }

        for (DcMotor motor : rightMotors)
        {
            motor.setPower(0);
        }
    }

    private void setPowers(double leftSpeed, double rightSpeed)
    {
        for (DcMotor motor : leftMotorsWithEncoders)
        {
            motor.setPower(leftSpeed);
        }

        for (DcMotor motor : rightMotorsWithEncoders)
        {
            motor.setPower(rightSpeed);
        }

        for (DcMotor motor : leftMotors)
        {
            motor.setPower(leftSpeed);
        }

        for (DcMotor motor : rightMotors)
        {
            motor.setPower(rightSpeed);
        }
    }

    private boolean checkIsTargetReached()
    {
        boolean targetIsReached = false;

        for (DcMotor motor : leftMotorsWithEncoders)
        {
            if (Math.abs(leftEncoderTarget - motor.getCurrentPosition()) <
                    leftThreshold)
            {
                targetIsReached = true;
            }
        }

        for (DcMotor motor : rightMotorsWithEncoders)
        {
            if (Math.abs(rightEncoderTarget - motor.getCurrentPosition()) <
                    rightThreshold)
            {
                targetIsReached = true;
            }
        }

        return targetIsReached;
    }

    private void checkForStall(int previousLeftCounts, int previousRightCounts)
            throws MotorStallException
    {
        for (DcMotor motor : leftMotorsWithEncoders)
        {
            if (Math.abs(previousLeftCounts - motor.getCurrentPosition()) <
                    leftThreshold)
            {
                throw new MotorStallException("A left motor has stalled.");
            }
        }

        for (DcMotor motor : rightMotorsWithEncoders)
        {
            if (Math.abs(previousRightCounts - motor.getCurrentPosition()) <
                    rightThreshold)
            {
                throw new MotorStallException("A right motor has stalled.");
            }
        }
    }

    private void pMotorRun() throws MotorStallException
    {
        int previousLeftCounts = -100;
        int previousRightCounts = -100;
        long previousTime = System.currentTimeMillis();

        while (checkIsTargetReached())
        {
            setPowers(leftSpeed, rightSpeed);

            if (System.currentTimeMillis() >
                    (previousTime + checkMotorsFrequency))
            {
                checkForStall(previousLeftCounts, previousRightCounts);
            }

            previousTime = System.currentTimeMillis();
        }

        haltDrive();
    }

    private int calculateThreshold(double speed)
    {
        // TODO: Calculate stall threshold.

        return 0;
    }

    /**
     * Moves the robot straight forward or backward.
     * <p/>
     * The distance argument will be in whichever units the wheelDiameter
     * parameter was in when you constructed pMotor.
     * The distance argument should be positive or negative depending on the
     * direction you want the robot to travel. A positive distance should
     * make the robot go forward.
     * <p/>
     * The speed argument is some speed fraction from 0 to 1.0 (anything
     * outside this range and I will turn this car around).
     * <p/>
     * If the distance target cannot be reached, a MotorStallException exception
     * is thrown.
     *
     * @param distance The distance to move forward or backward.
     * @param speed    The speed at which to move.
     * @throws MotorStallException
     */
    @SuppressLint("Assert")
    public void linear(float distance, double speed) throws MotorStallException
    {
        assert ((0.0 <= speed) && (speed <= 1.0));

        leftSpeed = Math.signum(distance) * speed;
        rightSpeed = Math.signum(distance) * speed;
        leftEncoderTarget = Math.round(
                ((distance / (Math.PI * wheelDiameter)) * gearRatio) /
                        encoderCountsPerDriverGearRotation);
        rightEncoderTarget = Math.round(
                ((distance / (Math.PI * wheelDiameter)) * gearRatio) /
                        encoderCountsPerDriverGearRotation);
        leftThreshold = calculateThreshold(leftSpeed);
        rightThreshold = calculateThreshold(rightSpeed);

        pMotorRun();
    }

    /**
     * Spins the robot in place, performing a point-turn.
     * <p/>
     * The degrees argument should be positive or negative depending on the
     * direction you want the robot to turn. Positive degrees turn the robot
     * counter-clockwise (left).
     * <p/>
     * The speed argument is some speed fraction from 0 to 1.0 (anything
     * outside this range and I will turn this car around).
     * <p/>
     * If the distance target cannot be reached, a MotorStallException exception
     * is thrown.
     *
     * @param degrees The number of degrees to turn.
     * @param speed   The speed at which to move.
     * @throws MotorStallException
     */
    @SuppressLint("Assert")
    public void pointTurn(float degrees, double speed)
            throws MotorStallException
    {
        assert ((0.0 <= speed) && (speed <= 1.0));

        leftSpeed = Math.signum(degrees) * speed;
        rightSpeed = Math.signum(degrees) * -speed;

        // TODO: Convert degrees to encoder counts.
        leftEncoderTarget = 0;
        rightEncoderTarget = 0;

        pMotorRun();
    }
}
