// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends PIDCommand {

  final Swerve swerve;
  /** Creates a new AutoBalance. */
  public AutoBalance(final Swerve swerve) {
    super(
        // The controller that the command will use
        new PIDController(0.09 , 0, 0),
        // This should return the measurement
        swerve::getPitch,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          {
            swerve.drive(-(output / 7), 0, 0);
          }
        });
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerve);
    // Configure additional PID options by calling `getController` here.
    this.getController().setSetpoint(0);
    this.getController().setTolerance(5, 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.getController().atSetpoint();
  }
}
