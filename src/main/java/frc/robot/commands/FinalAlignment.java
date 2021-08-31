// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class FinalAlignment extends CommandBase {

    TrapezoidProfile m_profile;
    TrapezoidProfile.Constraints m_constraints;

    Timer m_timer = new Timer();

    double m_desiredAngle;
    PIDController m_angleController;

    /**
     * MaxVel and MaxAcc should not be the physical max but the desired vel and acc. 
     * Recommend a slow vel to have more time to get vision data and adjust
     */
    public FinalAlignment(double maxVel, double maxAcc, PIDController angleController) {
        m_constraints = new TrapezoidProfile.Constraints(maxVel, maxAcc);
        m_angleController = angleController;
        addRequirements(DriveBase.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /**
         * GET VISION DATA HERE
         */
        boolean newVisionData = true;
        double distance = 0;
        double angle = 0;

        if (newVisionData) {
            /** Distance   */
            double currentVelocity = DriveBase.getInstance().getAverageVelocity();

            // If the robot is going faster than the max velocity, vel PID targets max vel. 
            // If the current velocity is less than the max velocity, start at the current 
            //      veloity and the trapizoid will plan to accelerate or deaccelerate accordingly.
            double desiredVelocity = Math.min(currentVelocity, m_constraints.maxVelocity);
            
            // Start with a distance of 0 and a goal of the vision measured distance. End with 0 m/s.
            TrapezoidProfile.State currentState = new State(0, desiredVelocity);
            TrapezoidProfile.State desiredState = new State(distance, 0);

            // Overwrite the old profile with a new profile.
            m_profile = new TrapezoidProfile(m_constraints, desiredState, currentState);

            // New profile means start from 0 seconds again.
            m_timer.reset();
   
            /** Angle  */
            // error = setpoint - measured, setpoint = 0
            double deltaAngle = 0 - angle;
            double gyroAngle = DriveBase.getInstance().getHeading().getDegrees();

            m_desiredAngle = gyroAngle + deltaAngle;
        }

        // Distance
        double distanceOutput = m_profile.calculate(m_timer.get() + 0.02).position;

        // Angle
        double angleOutput = m_angleController.calculate(DriveBase.getInstance().getHeading().getDegrees(), m_desiredAngle);

        double leftVelocity = distanceOutput - angleOutput;
        double rightVelocity = distanceOutput + angleOutput;

        DriveBase.getInstance().setVelocity(leftVelocity, rightVelocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean trapizoidProfileCompleted = m_profile.isFinished(m_timer.get());
        return trapizoidProfileCompleted;
    }

}
