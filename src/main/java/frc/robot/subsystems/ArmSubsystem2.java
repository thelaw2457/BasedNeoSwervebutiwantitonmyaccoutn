// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem2 extends SubsystemBase {

  private final CANSparkMax m_ArmMotor = new CANSparkMax(13, MotorType.kBrushless);
  public final DutyCycleEncoder absEncoder = new DutyCycleEncoder(5);

  /** Creates a new ArmSubsystem2. */
  public ArmSubsystem2() {
    m_ArmMotor.setIdleMode(IdleMode.kBrake);
    m_ArmMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("absEncoder pos", getEncoderPosition());
    SmartDashboard.putNumber("Speed", m_ArmMotor.get());
    SmartDashboard.putNumber("intake setpoint", ArmConstants.MIN_SETPOINT);
    SmartDashboard.putNumber("stow setpoint", ArmConstants.MAX_SETPOINT);
    SmartDashboard.putNumber("score setpoint", ArmConstants.SCORING_SETPOINT);
  }

  public double getEncoderPosition() {
    return absEncoder.getDistance();
  }

  public void set(double armSpeed) {
    m_ArmMotor.set(armSpeed);
  }

  public void stop() {
    m_ArmMotor.set(0);
  }
}
