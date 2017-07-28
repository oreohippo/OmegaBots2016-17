package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Omegabot.
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and
 */
public class HardwareOmegaBots
{
    /* Public OpMode members. */
    public DcMotor  leftMotor       = null;
    public DcMotor  rightMotor      = null;
    public DcMotor  scooperMotor    = null;
    public DcMotor  shooterMotor    = null;
    public Servo    beacon          = null;
    public Servo    feeder          = null;


    public static final double MID_SERVO       =  0.5 ;
    public static final double BEACON_SERVO       =  0.3 ;
    public static final double SHOOTER_SPEED       =  15.0 ;
    public static final double SCOOPER_SPEED      =  5.0 ;
    public static final double ROBOT_SPEED      =  1.0 ;
    public static final double FEEDER_LOW      =  0.5 ;
    public static final double FEEDER_HIGH     =  0.1 ;
    public static final double MOTOR_OFF_SPEED      =  0 ;

    private ElapsedTime period  = new ElapsedTime();


    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareOmegaBots(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        scooperMotor    = hwMap.dcMotor.get("scooper");
        shooterMotor    = hwMap.dcMotor.get("shooter");


        scooperMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE
        shooterMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE
        // Set all motors to zero power
        leftMotor.setPower(MOTOR_OFF_SPEED);
        rightMotor.setPower(MOTOR_OFF_SPEED);
        scooperMotor.setPower(MOTOR_OFF_SPEED);
        shooterMotor.setPower(MOTOR_OFF_SPEED);
        // Set all motors to run without encoders.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scooperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.
        beacon = hwMap.servo.get("beacon");
        feeder = hwMap.servo.get("feeder");
        beacon.setPosition(MID_SERVO);
        feeder.setPosition(MID_SERVO);
        feeder.setDirection(Servo.Direction.FORWARD);
    }


    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

