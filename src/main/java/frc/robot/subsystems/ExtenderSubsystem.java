// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtenderSubsystem extends SubsystemBase {
  private final CANSparkMax m_ArmSlide;
  private final RelativeEncoder ArmSlideEncoder;
  private final double minSetpoint = Constants.ExtenderConstants.EXTENDER_MIN_EXTENSION;
  private final double maxSetpoint = Constants.ExtenderConstants.EXTENDER_MAX_EXTENSION;
  private double setpoint = minSetpoint;
  private double kp = 0.07;
  private final PIDController pid = new PIDController(kp, 0.0, 0.0);
  private boolean usingPID = false;
  private double setpointIncrementer = 0.8;
  public ExtenderSubsystem() {
    pid.setTolerance(20.0);
    
    m_ArmSlide = new CANSparkMax(14, MotorType.kBrushless);
    m_ArmSlide.setInverted(true);

    ArmSlideEncoder = m_ArmSlide.getEncoder();


    if (Constants.ExtenderConstants.isTunable) {
      SmartDashboard.putNumber("extender kp", kp);
      SmartDashboard.putNumber("extender setpoint", setpoint);
    }

    setSetpoint(minSetpoint);
  }

  public void extend() {
    System.out.println("Extend Run");
    if (usingPID) {
      setSetpoint(setpoint += setpointIncrementer);
    } else {
 
      if ((getPosition() < Constants.ARM_MAX_EXT)) {
        m_ArmSlide.set(0.2);
      } else {
        m_ArmSlide.set(0.0);
      }
    }
  }

  public void retract() {
    System.out.println("Retract Run");
    if (usingPID) {
      setSetpoint(setpoint -= setpointIncrementer);
    } else {
      if ((getPosition() > Constants.ARM_MIN_EXT)) {
        m_ArmSlide.set(-0.2);
      } else {
        m_ArmSlide.set(0.0);
      }
    }
  }

  public void stop() {
    System.out.println("Stop Run");
    if (!usingPID) {
      m_ArmSlide.set(0);
    }
  }

  public double getPosition() {
    return ArmSlideEncoder.getPosition() ;
  }

  public void setSetpoint(double newSetpoint) {
    if (newSetpoint > maxSetpoint) {
      setpoint = maxSetpoint;
    } else if (newSetpoint < minSetpoint) {
      setpoint = minSetpoint;
    } else {
      setpoint = newSetpoint;
    }
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  @Override
  public void periodic() {
    if (Constants.ExtenderConstants.isTunable) {
      SmartDashboard.putNumber("extender encoder", getPosition());
      //kp = SmartDashboard.getNumber("extender kp", kp);
      //pid.setP(kp);
    }

    double speed = pid.calculate(getPosition(), setpoint);
    if (usingPID && !atSetpoint()) {
      m_ArmSlide.set(speed);
    }

  }
}
