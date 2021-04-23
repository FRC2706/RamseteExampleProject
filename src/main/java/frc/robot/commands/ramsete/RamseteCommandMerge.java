// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ramsete;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveBase;

public class RamseteCommandMerge extends RamseteCommand {

    private final RamseteControllerLogging m_loggingRamseteController;

    /**
     * Construct a RamseteCommandMerge to use RamseteCommand to drive a given trajectory
     * 
     * @param trajectory Path to follow.
     * @param loggingDataFileName A logging data file will be created on a usb drive mounted on
     *                            the RoboRio. Give it a good name to know what path it recorded.
     */
    public RamseteCommandMerge(Trajectory trajectory, String loggingDataFileName) { 
        this(trajectory, DriveBase.getInstance(), new RamseteControllerLogging(loggingDataFileName));
    }

    /**
     * Construct a RamseteCommandMerge to store the instance of RamseteControllerLogging
     */
    private RamseteCommandMerge(Trajectory trajectory, DriveBase driveBase, RamseteControllerLogging loggingRamseteController) {
        super(
            trajectory,
            driveBase::getPose,
            loggingRamseteController,
            Config.differentialDriveKinematics,
            driveBase::setVelocity,
            driveBase
        );
        m_loggingRamseteController = loggingRamseteController; 
    }

    @Override
    public boolean isFinished() {
        // If RamseteCommand wants to end, stop logging
        if (super.isFinished()) {
            m_loggingRamseteController.stopLogging();
            return true;
        } else {
            return false;
        }
    }
}
