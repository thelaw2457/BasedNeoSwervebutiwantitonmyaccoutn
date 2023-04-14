// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.ArmSubsystem;

// public class ControlArmCommand extends CommandBase {
//   private final ArmSubsystem ARM_SUBSYSTEM;
//   private final DoubleSupplier speed;
//   /** Creates a new ControlArmCommand. */
//   public ControlArmCommand(ArmSubsystem armSubsystem, DoubleSupplier speed) {
//     this.ARM_SUBSYSTEM = armSubsystem;
//     this.speed = speed;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(ARM_SUBSYSTEM);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (speed.getAsDouble() < 0 && ARM_SUBSYSTEM.getAbsEncoder() > Constants.ARM_MIN_DEG) {
//       ARM_SUBSYSTEM.setMotorOutput(speed.getAsDouble());
//     } else {
//       ARM_SUBSYSTEM.setMotorOutput(0);
//     }
//     if (speed.getAsDouble() > 0 && ARM_SUBSYSTEM.getAbsEncoder() < Constants.ARM_MAX_DEG) {
//       ARM_SUBSYSTEM.setMotorOutput(speed.getAsDouble());
//     } else {
//       ARM_SUBSYSTEM.setMotorOutput(0);
//     }

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     ARM_SUBSYSTEM.setMotorOutput(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
