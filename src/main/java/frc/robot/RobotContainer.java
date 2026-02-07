package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shootOnTheFlyCommand;

import java.util.Set;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.05)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.Velocity);
;

  // Controllers
  private final ThrustmasterJoystick leftJoystick = new ThrustmasterJoystick(0);
  private final ThrustmasterJoystick rightJoystick = new ThrustmasterJoystick(1);
  private final LogitechController operatorController = new LogitechController(2);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final Auto auto;
  private DoubleSupplier leftJoystickVelocityX;
  private DoubleSupplier leftJoystickVelocityY;
  private DoubleSupplier rightJoystickVelocityTheta;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    if (Robot.isReal()) {

    } else {

    }
    auto = new Auto(this);

    configureButtonBindings();
    
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityY(
                        -Math.pow(leftJoystick.getXAxis().getRaw(), 3)
                            * MaxSpeed) // Drive forward with negative Y
                    // (forward) POSSIBLY READD -
                    // TO FIX ANY INVERT ISSUES
                    .withVelocityX(
                        -Math.pow(leftJoystick.getYAxis().getRaw(), 3)
                            * MaxSpeed) // Drive left with negative X
                    // (left)
                    .withRotationalRate(
                        Math.pow(-rightJoystick.getXAxis().getRaw(), 3) * MaxAngularRate)
                    .withDeadband(0.02) // Drive counterclockwise with negative X
            // (left)
            ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    LimelightHelpers.SetIMUMode("limelight-left", 1);
    LimelightHelpers.SetIMUMode("limelight-right", 1);
    

    rightJoystick
        .getLeftTopLeft()
        .onTrue(
            Commands.runOnce(
                () -> 
                    drivetrain.resetPose(
                        new Pose2d(0, 0, drivetrain.getOperatorForwardDirection()))));

    leftJoystick
        .getLeftTopLeft()
        .whileTrue(new shootOnTheFlyCommand(
            () -> {return drivetrain.getRobotPose().getTranslation();} 
            ));
            // drivetrain::getPose,
            // () -> new Translation2d(0, 0), // hub position - this would be set to the actual hub position
            // drivetrain::getVelocity,
            // drivetrain::getAngularVelocity,
            // drivetrain::getRotation));
    operatorController.getA().whileTrue(drivetrain.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    operatorController.getB().whileTrue(drivetrain.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
    operatorController.getX().whileTrue(drivetrain.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    operatorController.getY().whileTrue(drivetrain.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));

    operatorController.getLeftBumper().onTrue(Commands.runOnce(() -> {
        SignalLogger.stop();
    }, drivetrain));
    }
    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto.getAutoCommand();
  }
}