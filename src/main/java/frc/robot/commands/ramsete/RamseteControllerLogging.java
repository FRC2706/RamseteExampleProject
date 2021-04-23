// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ramsete;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class RamseteControllerLogging extends RamseteController {

    // This must match the double[] in this objects calculate() method
    private final String[] m_loggerHeadings = new String[] {
        "Approx. Match Time",
        "odometryX", "odometryY", "odometryRot",
        "plannedX", "plannedY", "plannedRot",
        "errorX", "errorY", "errorRot",
        "plannedVelocity",
        "plannedAcceleration"
    };

    // NetworkTable Values
    private NetworkTableEntry xError, yError, rotError;

    private String m_loggingDataFileName;
    private SimpleCsvLogger usbLogger = new SimpleCsvLogger();
    private boolean loggerInitialized = false;

    public RamseteControllerLogging(String loggingDataFileName) {
        super();
        m_loggingDataFileName = loggingDataFileName;

        var table = NetworkTableInstance.getDefault().getTable("RamseteAutoError"); 
        xError = table.getEntry("xError");
        yError = table.getEntry("yError");
        rotError = table.getEntry("rotError"); 
    }

    public void startLogging() {
        usbLogger.init(m_loggingDataFileName, m_loggerHeadings);
    }

    public void stopLogging() {
        usbLogger.close();
        loggerInitialized = false;
    }

    /**
     * Interjects logging into ramsete
     * 
     * Adds some code on top of the RamseteController calculate() method to log the values passed into it.
     */
    @Override
    public ChassisSpeeds calculate( 
            Pose2d currentPose,
            Pose2d poseRef,
            double linearVelocityRefMeters,
            double angularVelocityRefRadiansPerSecond) {

        // Error between odometry and desired pose
        Pose2d poseError = poseRef.relativeTo(currentPose);

        // Initialize the logger if not done yet
        if (loggerInitialized == false) {
            startLogging();
            loggerInitialized = true;
        }

        // This series of entries must match the string of headings at the top of this file
        usbLogger.writeData(
            Timer.getMatchTime(),
            currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees(),
            poseRef.getX(), poseRef.getY(), poseRef.getRotation().getDegrees(),
            poseError.getX(), poseError.getY(), poseError.getRotation().getDegrees(),
            linearVelocityRefMeters,
            angularVelocityRefRadiansPerSecond);

        xError.setNumber(poseError.getX());
        yError.setNumber(poseError.getY());
        rotError.setNumber(poseError.getRotation().getDegrees());

        // Let ramsete continue on as normal
        return super.calculate(currentPose, poseRef, linearVelocityRefMeters, angularVelocityRefRadiansPerSecond);
    }



}
