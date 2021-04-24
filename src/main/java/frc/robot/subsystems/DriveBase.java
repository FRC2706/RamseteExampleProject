// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CTREUnits;
import frc.robot.config.Config;

public class DriveBase extends SubsystemBase {

    private static DriveBase currentInstance;

    private WPI_TalonSRX leftMaster, rightMaster;
    private BaseMotorController leftSlave, rightSlave;
    private PigeonIMU pigeon;

    private DifferentialDrive differentialDrive; 

    private DifferentialDriveOdometry odometry; 
    private SimpleMotorFeedforward feedforward;

    // Variables to help calculate acceleration for Ramsete
    private Timer timer;
    private double prevTime = 0;
    private double prevLeftVelocity = 0;
    private double prevRightVelocity = 0;

    // NetworkTable Values
    private NetworkTableEntry xOdometry, yOdometry, rotOdometry;

    public static DriveBase getInstance() {
        if (currentInstance == null) {
            currentInstance = new DriveBase();
        }
        return currentInstance;
    }

    private DriveBase() {
        leftMaster = new WPI_TalonSRX(Config.LEFT_MASTER);
        rightMaster = new WPI_TalonSRX(Config.RIGHT_MASTER);
        leftSlave = new WPI_VictorSPX(Config.LEFT_SLAVE);
        rightSlave = new WPI_VictorSPX(Config.RIGHT_SLAVE);
        pigeon = new PigeonIMU(new WPI_TalonSRX(Config.PIGEON_ID));

        setTalonConfigurations();

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getCurrentAngle())); 
        feedforward = new SimpleMotorFeedforward(Config.ksVolts, Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter);

        timer = new Timer();
        timer.reset();
        timer.start();

        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
        differentialDrive.setRightSideInverted(Config.DRIVETRAIN_INVERT_DIFFERENTIALDRIVE);

        var table = NetworkTableInstance.getDefault().getTable("DrivetrainOdometry");
        xOdometry = table.getEntry("xOdometry");
        yOdometry = table.getEntry("yOdometry");
        rotOdometry = table.getEntry("rotOdometry"); 
        
    }

    /** 
     * Set all the settings on the talons/victors
     */
    private void setTalonConfigurations() {

        // Set all talons to factory default values
        leftMaster.configFactoryDefault();
        rightMaster.configFactoryDefault();
        leftSlave.configFactoryDefault();
        rightMaster.configFactoryDefault();

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        TalonSRXConfiguration talonConfig = new TalonSRXConfiguration(); 

        // Slot 1 belongs to Ramsete
        talonConfig.slot1.kF = 0;
        talonConfig.slot1.kP = Config.kRamsetePGain;
        talonConfig.slot1.kI = 0;
        talonConfig.slot1.kD = 0;
        talonConfig.slot1.allowableClosedloopError = 0;

        // Config all talon settings - automatically returns the "worst error"
        ErrorCode leftMasterError = leftMaster.configAllSettings(talonConfig);
        ErrorCode rightMasterError = rightMaster.configAllSettings(talonConfig);

        // Config the encoder and check if it worked
        ErrorCode leftEncoderError = leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        ErrorCode rightEncoderError = rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        ErrorCode pigeonError = pigeon.getLastError();
        PigeonState pigeonState = pigeon.getState();

        /** Make sure to log the .name() of both error codes and check if they are ErrorCode.OK  */
        /** PigeonState, check for PigeonState.Ready */

        // Invert motor controllers if boolean is true
        leftMaster.setInverted(Config.INVERT_LEFT_MASTER);
        rightMaster.setInverted(Config.INVERT_RIGHT_MASTER);
        leftSlave.setInverted(Config.INVERT_LEFT_SLAVE);
        rightMaster.setInverted(Config.INVERT_RIGHT_MASTER);

        // Invert encoders if boolean is true
        leftMaster.setSensorPhase(Config.INVERT_LEFT_ENCODER);
        rightMaster.setSensorPhase(Config.INVERT_RIGHT_ENCODER);

    }

    /**
     * Get the fused heading from the pigeon
     * 
     * @return Heading of the robot in degrees
     */
    private double getCurrentAngle() {
        return pigeon.getFusedHeading();
    }
    
    /**
     * Get the encoder data in meters
     */
    private double getLeftPosition() {
        return CTREUnits.talonPosistionToMeters(leftMaster.getSelectedSensorPosition());
    }

    /**
     * Get the encoder data in meters
     */
    private double getRightPosition() {
        return CTREUnits.talonPosistionToMeters(rightMaster.getSelectedSensorPosition());
    }

    @Override
    public void periodic() {

        /** 
         * Give heading (from gyro) and encoder data in meters to odometry to calculate a new robot pose.
         */
        Pose2d newPose = odometry.update(Rotation2d.fromDegrees(getCurrentAngle()), getLeftPosition(), getRightPosition());

        xOdometry.setDouble(newPose.getX());
        yOdometry.setDouble(newPose.getY());
        rotOdometry.setDouble(newPose.getRotation().getDegrees());
    }

    /**
     * Returns the a Pose2d of the current robot location
     * 
     * Odometry calculates a new pose every robot cycle and stores
     * the value so this method is only reading the stored value.
     * This means it already does only 1 hardware read every cycle instead of 
     * many things calling hardware redundantly.
     * 
     * @param Pose2d the current pose of the robot
     */
    public Pose2d getPose() { 
        return odometry.getPoseMeters();
    }

    /**
     * This method will return the heading from odometry
     * 
     * Odometry keeps track of the gyro heading and in relation to
     * the value it was reset to using an offset so it's important to ask
     * the odometry for the rotation instead of directly from the gyro.
     * 
     * @param Rotation2d The heading. Rotation2d has a .getDegrees() & a .getRadians() method.
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Set the odometry to a given pose.
     * 
     * Necessary to do at the beginning of a match.
     * 
     * Very important to set the encoders to 0 when resetting odometry.
     * 
     * @param newPose New pose to set odometry to.
     */
    public void resetPose(Pose2d newPose) {
        ErrorCode leftError = leftMaster.setSelectedSensorPosition(0, Config.TALON_PRIMARY_PID, Config.CAN_TIMEOUT_LONG);
        ErrorCode rightError = rightMaster.setSelectedSensorPosition(0, Config.TALON_PRIMARY_PID, Config.CAN_TIMEOUT_LONG);

        odometry.resetPosition(newPose, Rotation2d.fromDegrees(getCurrentAngle()));
    }


    /**
     * Set left and right velocity of the drivetrain
     * 
     * This method of using SimpleMotorFeedforward and frc-characterization is from the other
     * constructor of RamseteCommand. Instead of running a PIDController in RamseteCommand, the
     * PID loop is run on the talon using ControlMode.Velocity. However only a P gain is required
     * since it only has to compensate for any error in SimpleMotorFeedforward.
     * 
     * See writeup on frc-characterization for more info on that equation and how to characterize.
     * 
     * @param leftVelocity Meters per second for the left side.
     * @param rightVelocity Meters per second for the right side.
     */
    public void setVelocity(double leftVelocity, double rightVelocity) {
        // Calculate time since last call (deltaTime)
        double currentTime = timer.get();
        double deltaTime = timer.get() - prevTime;
        prevTime = currentTime;

        // Throw out a time longer than a few robot cycles (in between ramsete commands this builds up)
        if (deltaTime > 0.1)  {
            return;
        }

        // Calculate acceleration
        double leftAcceleration = (leftVelocity - prevLeftVelocity) / deltaTime;
        double rightAcceleration = (rightVelocity - prevRightVelocity) / deltaTime;

        // Calculate feed forward value
        double leftFeedforward = feedforward.calculate(leftVelocity, leftAcceleration);
        double rightFeedforward = feedforward.calculate(rightVelocity, rightAcceleration);

        // Give the feed forward value and velocity set point to the talons
        leftMaster.set(ControlMode.Velocity, CTREUnits.metersPerSecondToTalonVelocity(leftVelocity),
                DemandType.ArbitraryFeedForward, leftFeedforward / leftMaster.getBusVoltage()); 

        rightMaster.set(ControlMode.Velocity, CTREUnits.metersPerSecondToTalonVelocity(rightVelocity),  
                DemandType.ArbitraryFeedForward, rightFeedforward / rightMaster.getBusVoltage()); 

        // Make sure motor safety knows the motors are being used
        differentialDrive.feed();

        /**
         * ArbFF accepts a value from -1 to 1. This is a percentage output to apply. It considers 100% (or a value of 1)
         * to be the voltage coming into the TalonSRX. So to give the motor the voltage calculated from SimpleMotorFeedforward,
         * SimpleMotorFeedforward's value is divided by the bus voltage, then the talon multiplies it by the bus voltage.
         * 
         * Because it is done this way, it means VoltageCompensation should NOT be on since it will conflict.
         */
    }

}
