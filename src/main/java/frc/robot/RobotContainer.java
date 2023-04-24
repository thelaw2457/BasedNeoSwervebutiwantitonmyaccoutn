package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeSubsystem;
import frc.commands.AutoBalance;
import frc.commands.AutoPathHelper;
import frc.commands.CubeDrool;
import frc.commands.CubeSlurp;
import frc.commands.CubeSpew;
import frc.commands.CubeSpit;
import frc.commands.PivotIntakePos;
import frc.commands.PivotScoringPos;
import frc.commands.PivotStartPos;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final ArmSubsystem m_ArmSubsystem2 = new ArmSubsystem();

  public final Joystick driver;

  public final XboxController xbox;

  public final Swerve swerve;

  public final AutoCommands auto;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    driver = new Joystick(0);

    xbox = new XboxController(1);

    swerve = new Swerve();

    auto = new AutoCommands(swerve);
    configureButtonBindings();
    configureSmartDashboard();
    configureAutoCommands();
  }

  public void configureSmartDashboard() {
    SmartDashboard.putNumber("Angle", 5);
  }

  public void printWheelAngles() {
    swerve.printAngles();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    swerve.setDefaultCommand(swerve.drive(
      () -> -driver.getY(),
      () -> -driver.getX(),
      () -> -driver.getZ(),
      true,
      false,
      0
    ));

    // new JoystickButton(driver, 5)
    // .onTrue(new SetArmAngle(m_armSubsystem, 30));
    // new JoystickButton(driver, 6)
    // .onTrue(new SetArmAngle(m_armSubsystem, 40));
    // new JoystickButton(driver, 3)
    // .onTrue(new SetArmAngle(m_armSubsystem, 50));

    new JoystickButton(driver, 7)
      .onTrue(swerve.zeroGyroCommand());

      //Arm Extend and Retract
      // new JoystickButton(xbox, 4).whileTrue(new RunCommand(() -> m_extenderSubsystem.extend())).onFalse(new InstantCommand(() -> m_extenderSubsystem.stop()));
      // new JoystickButton(xbox, 1).whileTrue(new RunCommand(() -> m_extenderSubsystem.retract())).onFalse(new InstantCommand(() -> m_extenderSubsystem.stop()));

      //Grippers
      // new JoystickButton(xbox, 5).whileTrue(new RunCommand(() -> m_gripperSubsystem.closeGripper1())).onFalse(new InstantCommand(() -> m_gripperSubsystem.hold()));
      // new JoystickButton(xbox, 6).whileTrue(new RunCommand(() -> m_gripperSubsystem.openGripper1())).onFalse(new InstantCommand(() -> m_gripperSubsystem.stop()));
      //new JoystickButton(xbox, 5).whileTrue(new RunCommand(() -> m_gripperSubsystem.closeGripper2())).onFalse(new InstantCommand(() -> m_gripperSubsystem.stop()));
      //new JoystickButton(xbox, 3).whileTrue(new RunCommand(() -> m_gripperSubsystem.openGripper2())).onFalse(new InstantCommand(() -> m_gripperSubsystem.stop()));

      // Intake
      new JoystickButton(xbox, 1).whileTrue(new CubeSlurp(m_intakeSubsystem, IntakeConstants.SLURP_SPEED));
      new JoystickButton(xbox, 3).whileTrue(new CubeDrool(m_intakeSubsystem, IntakeConstants.DROOL_SPEED));
      new JoystickButton(xbox, 4).whileTrue(new CubeSpit(m_intakeSubsystem, IntakeConstants.SPIT_SPEED));
      new JoystickButton(xbox, 2).whileTrue(new CubeSpew(m_intakeSubsystem, IntakeConstants.SPEW_SPEED));

      // ARM Angle
      //  new JoystickButton(xbox, 5).whileTrue(new RunCommand(() -> m_armSubsystem.raise())).onFalse(new InstantCommand(() -> m_armSubsystem.stop()));
      //  new JoystickButton(xbox, 6).whileTrue(new RunCommand(() -> m_armSubsystem.lower())).onFalse(new InstantCommand(() -> m_armSubsystem.stop()));

      new JoystickButton(xbox, 5).whileTrue(new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      new JoystickButton(xbox, 7).whileTrue(new PivotScoringPos(m_ArmSubsystem2, ArmConstants.SCORING_SETPOINT));
      new JoystickButton(xbox, 6).whileTrue(new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      new JoystickButton(driver, 5).whileTrue(new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      new JoystickButton(driver, 3).whileTrue(new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
    }

    private String getMode() {
    return null;
  }

    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
      //return new AutoBalance(swerve);
      //return AutoPathHelper.followPath(swerve, "Test", null);
      // return swerve.autoDrive(0.3, 0, 0, false, false, 2);
  }


  public void configureAutoCommands() {
      autoChooser.setDefaultOption("Do Nothing", new InstantCommand());

      final HashMap<String, Command> backupAndBalanceEventMap = new HashMap<>();
      backupAndBalanceEventMap.put("TestPrint", new PrintCommand("Hi!"));
      // backupAndBalanceEventMap.put("event1", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      // backupAndBalanceEventMap.put("event2", new CubeSlurp(m_intakeSubsystem));
      // backupAndBalanceEventMap.put("event3", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));

      final HashMap<String, Command> AllianceAutoEventMap = new HashMap<>();

      final HashMap<String, Command> BlueChargeStationEventMap = new HashMap<>();
      BlueChargeStationEventMap.put("BCSinitspit", new CubeSpit(m_intakeSubsystem, IntakeConstants.AUTO_SPIT_SPEED));
      BlueChargeStationEventMap.put("BCSdroppivot", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      BlueChargeStationEventMap.put("BCSslurp", new CubeSlurp(m_intakeSubsystem, IntakeConstants.AUTO_SLURP_SPEED));
      BlueChargeStationEventMap.put("BCSraisepivot", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      BlueChargeStationEventMap.put("BCSspitpt2", new CubeSpit(m_intakeSubsystem, IntakeConstants.AUTO_SPIT_SPEED));
      // BlueChargeStationEventMap.put("BCSlowerpivotpt2", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      // BlueChargeStationEventMap.put("BCSspit/spew", new CubeSpit(m_intakeSubsystem));
      // BlueChargeStationEventMap.put("BCSraisepivotpt2", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));

      final HashMap<String, Command> BlueLeftWorldsEventMap = new HashMap<>();
      BlueLeftWorldsEventMap.put("BLWlowerpivot", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      BlueLeftWorldsEventMap.put("BLWslurp", new CubeSlurp(m_intakeSubsystem, IntakeConstants.AUTO_SLURP_SPEED));
      BlueLeftWorldsEventMap.put("BLWraisepivot", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      BlueLeftWorldsEventMap.put("BLWlowerpivotpt2", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      BlueLeftWorldsEventMap.put("BLWspit/spew", new CubeSpit(m_intakeSubsystem, IntakeConstants.AUTO_SPIT_SPEED));
      BlueLeftWorldsEventMap.put("BLWraisepivotpt2", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      BlueLeftWorldsEventMap.put("BLWlowerpivotpt3", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      BlueLeftWorldsEventMap.put("BLWslurppt2", new CubeSlurp(m_intakeSubsystem, IntakeConstants.AUTO_SLURP_SPEED));

      final HashMap<String, Command> BlueRightWorldsEventMap = new HashMap<>();
      BlueRightWorldsEventMap.put("BRWlowerpivot", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      BlueRightWorldsEventMap.put("BRWslurp", new CubeSlurp(m_intakeSubsystem, IntakeConstants.AUTO_SLURP_SPEED));
      BlueRightWorldsEventMap.put("BRWraisepivot", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      BlueRightWorldsEventMap.put("BRWlowerpivotpt2", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      BlueRightWorldsEventMap.put("BRWspit/spew", new CubeSpit(m_intakeSubsystem, IntakeConstants.AUTO_SPIT_SPEED));
      BlueRightWorldsEventMap.put("BRWraisepivotpt2", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      BlueRightWorldsEventMap.put("BRWlowerpivotpt3", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      BlueRightWorldsEventMap.put("BRWslurppt2", new CubeSlurp(m_intakeSubsystem, IntakeConstants.AUTO_SLURP_SPEED));
      
      final HashMap<String, Command> RedChargeStationEventMap = new HashMap<>();
      RedChargeStationEventMap.put("RCSlowerpivot", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      RedChargeStationEventMap.put("RCSslurp", new CubeSlurp(m_intakeSubsystem, IntakeConstants.AUTO_SLURP_SPEED));
      RedChargeStationEventMap.put("RCSraisepivot", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      
      final HashMap<String, Command> RedLeftWorldsEventMap = new HashMap<>();
      RedLeftWorldsEventMap.put("RLWlowerpivot", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      RedLeftWorldsEventMap.put("RLWslurp", new CubeSlurp(m_intakeSubsystem, IntakeConstants.AUTO_SLURP_SPEED));
      RedLeftWorldsEventMap.put("RLWraisepivot", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      RedLeftWorldsEventMap.put("RLWlowerpivotpt2", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      RedLeftWorldsEventMap.put("RLWspit/spew", new CubeSpit(m_intakeSubsystem, IntakeConstants.AUTO_SPIT_SPEED));
      RedLeftWorldsEventMap.put("RLWraisepivotpt2", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      RedLeftWorldsEventMap.put("RLWlowerpivotpt3", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      RedLeftWorldsEventMap.put("RLWslurppt2", new CubeSlurp(m_intakeSubsystem, IntakeConstants.AUTO_SLURP_SPEED));
      RedLeftWorldsEventMap.put("RLWraisepivotpt3", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));

      final HashMap<String, Command> RedRightWorldsEventMap = new HashMap<>();
      RedRightWorldsEventMap.put("RRWlowerpivot", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      RedRightWorldsEventMap.put("RRWslurp", new CubeSlurp(m_intakeSubsystem, IntakeConstants.AUTO_SLURP_SPEED)); 
      RedRightWorldsEventMap.put("RRWraisepivot", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      RedRightWorldsEventMap.put("RRWlowerpivotpt2", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      RedRightWorldsEventMap.put("RRWspit/spew", new CubeSpit(m_intakeSubsystem, IntakeConstants.AUTO_SPIT_SPEED));
      RedRightWorldsEventMap.put("RRWraisepivotpt2", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      RedRightWorldsEventMap.put("RRWlowerpivotpt3", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      RedRightWorldsEventMap.put("RRWslurppt2", new CubeSlurp(m_intakeSubsystem, IntakeConstants.AUTO_SLURP_SPEED));
      RedRightWorldsEventMap.put("RRWraisepivotpt3", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));

      final HashMap<String, Command> BlueAutoEventMap = new HashMap<>();
      BlueAutoEventMap.put("BAinitspit", new CubeSpit(m_intakeSubsystem, IntakeConstants.AUTO_SPIT_SPEED));
      BlueAutoEventMap.put("BAlowerpivot", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      BlueAutoEventMap.put("BAslurp", new CubeSlurp(m_intakeSubsystem, IntakeConstants.AUTO_SLURP_SPEED));
      BlueAutoEventMap.put("BAraisepivot", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      BlueAutoEventMap.put("BAspit", new CubeSpit(m_intakeSubsystem, IntakeConstants.AUTO_SPIT_SPEED));

      final HashMap<String, Command> BlueAutoCopyEventMap = new HashMap<>();
      BlueAutoCopyEventMap.put("BACinitspit", new CubeSpit(m_intakeSubsystem, IntakeConstants.AUTO_SPIT_SPEED));
      BlueAutoCopyEventMap.put("BAClowerpivot", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      BlueAutoCopyEventMap.put("BACslurp", new CubeSlurp(m_intakeSubsystem, IntakeConstants.AUTO_SLURP_SPEED));
      BlueAutoCopyEventMap.put("BACraisepivot", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      BlueAutoCopyEventMap.put("BACscoringpos", new PivotScoringPos(m_ArmSubsystem2, ArmConstants.SCORING_SETPOINT));
      BlueAutoCopyEventMap.put("BACspit", new CubeSpit(m_intakeSubsystem, IntakeConstants.AUTO_SPEW_SPEED));

      final HashMap<String, Command> RedWorldsEventMap = new HashMap<>();
      RedWorldsEventMap.put("RWinitspit", new CubeSpit(m_intakeSubsystem, IntakeConstants.AUTO_SPIT_SPEED));
      RedWorldsEventMap.put("RWlowerpivot", new PivotIntakePos(m_ArmSubsystem2, ArmConstants.MIN_SETPOINT));
      RedWorldsEventMap.put("RWslurp", new CubeSlurp(m_intakeSubsystem, IntakeConstants.AUTO_SLURP_SPEED));
      RedWorldsEventMap.put("RWraisepivot", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      RedWorldsEventMap.put("RWscoringpos", new PivotScoringPos(m_ArmSubsystem2, ArmConstants.SCORING_SETPOINT));
      RedWorldsEventMap.put("RWraisepivot2", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));
      RedWorldsEventMap.put("RWspit", new CubeSpit(m_intakeSubsystem, IntakeConstants.AUTO_SPIT_SPEED));

      final HashMap<String, Command> ATeamWantedMeToMakeThisEventMap = new HashMap<>();
      ATeamWantedMeToMakeThisEventMap.put("scoringpos", new PivotScoringPos(m_ArmSubsystem2, ArmConstants.SCORING_SETPOINT));
      ATeamWantedMeToMakeThisEventMap.put("spit", new CubeSpit(m_intakeSubsystem, IntakeConstants.AUTO_SPIT_SPEED));
      ATeamWantedMeToMakeThisEventMap.put("raisepivot", new PivotStartPos(m_ArmSubsystem2, ArmConstants.MAX_SETPOINT));

      // autoChooser.addOption("BackupAndBalance", AutoPathHelper.followPath(swerve, "BackupAndBalance", backupAndBalanceEventMap)
      // .andThen(new AutoBalance(swerve)));

      // autoChooser.addOption("AllianceAuto", AutoPathHelper.followPath(swerve, "AllianceAuto", AllianceAutoEventMap));

      autoChooser.addOption("WorldsChargeStation", AutoPathHelper.followPath(swerve, "WorldsChargeStation", BlueChargeStationEventMap)
      .andThen(new AutoBalance(swerve)));

      // autoChooser.addOption("BlueWireWorlds", AutoPathHelper.followPath(swerve, "BlueWireWorlds", BlueLeftWorldsEventMap));

      // autoChooser.addOption("BlueRightWorlds", AutoPathHelper.followPath(swerve, "BlueRightWorlds", BlueRightWorldsEventMap));

      // autoChooser.addOption("BlueAuto", AutoPathHelper.followPath(swerve, "BlueAuto", BlueAutoEventMap));

      autoChooser.addOption("BlueWorlds", AutoPathHelper.followPath(swerve, "BlueWorlds", BlueAutoCopyEventMap));

      // autoChooser.addOption("RedChargeStation", AutoPathHelper.followPath(swerve, "RedChargeStation", RedChargeStationEventMap));

      // autoChooser.addOption("RedLeftWorlds", AutoPathHelper.followPath(swerve, "RedLeftWorlds", RedLeftWorldsEventMap));

      // autoChooser.addOption("RedRightWorlds", AutoPathHelper.followPath(swerve, "RedRightWorlds", RedRightWorldsEventMap));

      autoChooser.addOption("RedWorlds", AutoPathHelper.followPath(swerve, "RedWorlds", RedWorldsEventMap));

      autoChooser.addOption("ATeamWantedMeToMakeThis", AutoPathHelper.followPath(swerve, "ATeamWantedMeToMakeThis", ATeamWantedMeToMakeThisEventMap));

      SmartDashboard.putData(autoChooser);
  }
}
