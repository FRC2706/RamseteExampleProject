// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AngleTest extends CommandBase {

    private final Supplier<Double> xAxis;
    private final Supplier<Double> yAxis;

    /** Creates a new AngleTest. */
    public AngleTest(Supplier<Double> xAxis, Supplier<Double> yAxis) {

        this.xAxis = xAxis;
        this.yAxis = yAxis;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = xAxis.get();
        double y = yAxis.get();

        Rotation2d angle = new Rotation2d(x, y);
        
        System.out.println(angle.getDegrees());

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
