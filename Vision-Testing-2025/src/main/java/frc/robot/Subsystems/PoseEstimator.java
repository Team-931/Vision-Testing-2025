package frc.robot.Subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Subsystems.Drivetrain.Drivetrain;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Drivetrain drivetrain;
    private final Field2d field = new Field2d();
    private boolean auto = true;

    public PoseEstimator(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        poseEstimator = new SwerveDrivePoseEstimator(this.drivetrain.getKinematics(),
         this.drivetrain.getHeading(),
         this.drivetrain.getModulePositions(), 
         this.drivetrain.getInitPoseMetres());
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        updatePoseEstimator(false);
        updateShuffleboard();
    }

    private void updatePoseEstimator(boolean force) {
        SmartDashboard.putBoolean("Auto Pose", auto);

        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), drivetrain.getHeading(),
                drivetrain.getModulePositions());
        updateWithVision("limelight");
    }

    private void updateWithVision(String limelightName) {
        double ta = LimelightHelpers.getTA(limelightName);
        PoseEstimate limelightBotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        int validTagCount = limelightBotPose.tagCount;
        boolean slowRotate = drivetrain.getChassisSpeed().omegaRadiansPerSecond <= 4 * Math.PI;

        if ((validTagCount >= 2.0 && ta >= 0.030) && slowRotate) {
            if (auto) {
                double antiTrust = 4*(-150.0 * ta + 10.0);
                if (antiTrust <= 8.0) {
                    antiTrust = 8.0;
                }                poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds,
                        VecBuilder.fill(antiTrust, antiTrust, antiTrust));
            } else {
                double antiTrust = -150.0 * ta + 10.0;
                if (antiTrust <= 2.0) {
                    antiTrust = 2.0;
                }
                poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds,
                        VecBuilder.fill(antiTrust, antiTrust, antiTrust));

            }
        } else if ((validTagCount == 1 && ta >= 0.060) && !auto && slowRotate) {
            if (auto) {
                poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds,
                        VecBuilder.fill(30.0, 30.0, 30.0));
            } else {
                double antiTrust = -69.0 * ta + 14.83;
                if (antiTrust <= 10.0) {
                    antiTrust = 10.0;
                }

                poseEstimator.addVisionMeasurement(limelightBotPose.pose, limelightBotPose.timestampSeconds,
                        VecBuilder.fill(antiTrust, antiTrust, antiTrust));
            }
        }

    }

    private void updateShuffleboard() {
        Pose2d pose = getPose();
        SmartDashboard.putNumber("PoseEstX", pose.getX());
        SmartDashboard.putNumber("PoseEstY", pose.getY());
        SmartDashboard.putNumber("PoseEstRot", pose.getRotation().getRadians());

        field.setRobotPose(pose);
    }

    public Pose2d getPose() {
            return poseEstimator.getEstimatedPosition();

    }

    public void setAuto(boolean auto) {
        this.auto = auto;
    }

    public boolean inside(Translation2d[] bounds, boolean onEdge) {
        Pose2d currentPose = getPose();
        double xMin = Math.min(bounds[0].getX(), bounds[1].getX());
        double xMax = Math.max(bounds[0].getX(), bounds[1].getX());
        double yMin = Math.min(bounds[0].getY(), bounds[1].getY());
        double yMax = Math.max(bounds[0].getY(), bounds[1].getY());
        return ((currentPose.getX() > xMin && currentPose.getX() < xMax)
                || (onEdge && (currentPose.getX() >= xMin && currentPose.getX() <= xMax))
                        &&
                        (currentPose.getY() > yMin && currentPose.getY() < yMax)
                || (onEdge && (currentPose.getY() >= yMin && currentPose.getY() <= yMax)));
    }

    public void resetOdometry(Pose2d pose) {
        drivetrain.resetOdometry(pose.getRotation());
        poseEstimator.resetPosition(drivetrain.getHeading(), drivetrain.getModulePositions(), pose);
    }
}

