// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import jdk.nashorn.api.tree.YieldTree;

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
        // Axis of a single stick on a joystick
        double x = xAxis.get();
        double y = -yAxis.get();

        Rotation2d angle; 
        if (Math.abs(x) < 0.6 && Math.abs(y) < 0.6) {
            angle = new Rotation2d(0);
        } else {
            angle = new Rotation2d(x, y);
        }
        angle.getDegrees(); // 2 possible methods to get the angle
        angle.getRadians();

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
