package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Autos {
    private final AutoFactory autoFactory;

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    private final SwerveRequest.ApplyFieldSpeeds autoApplySpeeds =
      new SwerveRequest.ApplyFieldSpeeds();

    private CommandSwerveDrivetrain drivetrain;

    public Autos() {
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        // autoFactory = new AutoFactory(
        //     drivetrain::getPose, // A function that returns the current robot pose
        //     drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
        //     drivetrain::followTrajectory, // The drive subsystem trajectory follower 
        //     true, // If alliance flipping should be enabled 
        //     drivetrain // The drive subsystem
        // );
        autoFactory = new AutoFactory(drivetrain::getPose, drivetrain::resetPose, this::followTrajectory, true, drivetrain);
    }


    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = drivetrain.getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // Apply the generated speeds
        drivetrain.setControl(
            autoApplySpeeds.withSpeeds(speeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }
}


