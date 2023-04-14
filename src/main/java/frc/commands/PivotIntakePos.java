// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class PivotIntakePos extends CommandBase {

  private ArmSubsystem ARM_SUBSYSTEM;
  private PIDController pidController;
  /** Creates a new PivotIntakePos. */
  public PivotIntakePos(ArmSubsystem arm, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.ARM_SUBSYSTEM = arm;
    this.pidController = new PIDController(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);
    pidController.setSetpoint(setpoint);

    addRequirements(ARM_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    System.out.println("Arm Intake position PID started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(ARM_SUBSYSTEM.getEncoderPosition());
    ARM_SUBSYSTEM.set(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ARM_SUBSYSTEM.stop();
    System.out.println("Arm Intake position PID ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
