// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {

    private final CANSparkMax m_FClamp = new CANSparkMax(17, MotorType.kBrushless);
    private final CANSparkMax m_BClamp = new CANSparkMax(16, MotorType.kBrushless);

    RelativeEncoder FrontClampEncoder = m_FClamp.getEncoder();

  private final double encoderOffset = 225.0;
  private double kp = 0.01; // 0.025
  private final boolean usingPID  = false;
  private double setpoint = Constants.GripperConstants.fullOpen;
  private final double setpointIncrementer = 0.8;
  PIDController pid = new PIDController(kp, 0, 0);

  public GripperSubsystem() {
    m_FClamp.setInverted(false);

    if (Constants.GripperConstants.isTunable) {
      SmartDashboard.putNumber("gripper kp", kp);
      SmartDashboard.putNumber("gripper setpoint", setpoint);
    }
  }

  public double getEncoderPos() {
    System.out.println((FrontClampEncoder.getPosition())*360.0 - encoderOffset);
    return (FrontClampEncoder.getPosition())*360.0 - encoderOffset;
  }

  public void setSetpoint(double newSetpoint) {
    if (newSetpoint < Constants.GripperConstants.fullOpenWhenExtended) {
      setpoint = Constants.GripperConstants.fullOpenWhenExtended;
    } else if (newSetpoint > Constants.GripperConstants.fullClosed) {
      setpoint = Constants.GripperConstants.fullClosed;
    } else {
      setpoint = newSetpoint;
    }
  }

  public void autonSetSetpoint(double newSetpoint) {
    setpoint = newSetpoint;
  }

  public double getPidOutput() {
    double output = pid.calculate(getEncoderPos(), setpoint);
    if(Math.abs(output) > 0.25) output = 0.25 * Math.signum(output);
    return output;
  }

  public void closeGripper1() {
    if(usingPID) {
      setSetpoint(setpoint += setpointIncrementer);
    } else {
      m_FClamp.set(0.2);
    }
  }

  public void openGripper1() {
    if(usingPID) {
    setSetpoint(setpoint -= setpointIncrementer);
    } else {
      m_FClamp.set(-0.2);
    }
  }

  public void closeGripper2() {
    if(usingPID) {
      setSetpoint(setpoint += setpointIncrementer);
    } else {
      m_BClamp.set(0.2);
    }
  }

  public void openGripper2() {
    if(usingPID) {
    setSetpoint(setpoint -= setpointIncrementer);
    } else {
      m_BClamp.set(-0.2);
    }
  }
//hold gripper position yippie
  public void hold() {
    m_FClamp.set(0.1);
  }

  public void stop() {
    m_FClamp.set(0.0);
    m_BClamp.set(0);
  }

  @Override
  public void periodic() {
    if (Constants.GripperConstants.isTunable) {
      SmartDashboard.putNumber("gripper encoder", FrontClampEncoder.getPosition());
      SmartDashboard.putNumber("gripper setpoint", setpoint);
      kp = SmartDashboard.getNumber("gripper kp", kp);
      pid.setP(kp);
    }

    if(usingPID) {
      m_FClamp.set(getPidOutput());
    }
  }
}
