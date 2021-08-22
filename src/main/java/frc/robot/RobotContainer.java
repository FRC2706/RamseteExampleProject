// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ramsete.RamseteCommandMerge;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveBase; 

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Joystick driverStick = new Joystick(0); 


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Example trajectory - Drive forward 1 meter
        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
            new Pose2d(),
            new Pose2d(1, 0, Rotation2d.fromDegrees(0))),
            getConfig(0, 0, false));
        RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "ExampleTrajectory-DriveForward");
        
        return new SequentialCommandGroup(
            resetOdometry(trajectory1),
            ramsete1
        );
    }
    
    /**
     * Get Config will return a TrajectoryConfig object needed to generate trajectories
     * 
     * If you want to add extra constraints its important to create a new 
     * TrajectoryConfig object for them.
     * 
     * For the same reason its important to set the startVelocity, endVelocity and isReversed
     * or else a value from a previous config may be used.
     * 
     * @param startVelocity Meters per second the robot starts at.
     * @param endVelocity Meters per second to ramp down to at the end of the trajectory.
     * @param isReversed True if the robot should drive in reverse. Poses must be 180 deg offset.
     * @return TrajectoryConfig object to pass to TrajectoryGenerator
     */
    private TrajectoryConfig getConfig(double startVelocity, double endVelocity, boolean isReversed) {
        return Config.trajectoryConfig
            .setStartVelocity(startVelocity)
            .setEndVelocity(endVelocity)
            .setReversed(isReversed);
    }
    
    /**
     * Create a command that will reset the odometry to the initial pose of a given trajectory.
     * 
     * @param trajectory Trajectory to get the inital pose from.
     * @return Command to reset odometry
     */
    private Command resetOdometry(Trajectory trajectory) {
        Pose2d initalPose = trajectory.getInitialPose();
        return new InstantCommand(() -> DriveBase.getInstance().resetPose(initalPose));
    }

    /**
     * Create a command that will reset the odometry to the given pose
     * 
     * @param pose Pose to reset odometry to
     * @return Command to reset odometry
     */
    private Command resetOdometry(Pose2d pose) {
        return new InstantCommand(() -> DriveBase.getInstance().resetPose(pose));
    }
}
