// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.utils.Util;

// public class SetArmAngle extends CommandBase {
//   private final ArmSubsystem ARM_SUBSYSTEM;
//   private final double position;
//   /** Creates a new SetArmAngle. */
//   public SetArmAngle(ArmSubsystem arm, double position) {
//     this.ARM_SUBSYSTEM = arm;
//     this.position = position;
//     addRequirements(arm);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return Util.isWithinTolerance(ARM_SUBSYSTEM.getDegrees(), ARM_SUBSYSTEM.getSetpoint(), 2);
//   }
// }
