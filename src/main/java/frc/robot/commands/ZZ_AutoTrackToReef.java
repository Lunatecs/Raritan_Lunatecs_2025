package frc.robot.commands;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AprilTagPositions;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZZ_AutoTrackToReef extends Command {
  private Command fullPath;
  private CommandSwerveDrivetrain drive;
  private boolean isLeftBumper = false;

  /** Creates a new DriveToNearestReefSideCommand. */
  public ZZ_AutoTrackToReef(CommandSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //drive.configureAutoBuilder();
    //Pose2d botPose2d = LimelightHelpers.getBotPose2d_wpiRed("limelight-left");
    //SmartDashboard.putString("limelight bot pose", botPose2d.toString());
    //drive.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiRed("limelight-left"), LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left").timestampSeconds);
    Pose2d closestAprilTagPose = getClosestReefAprilTagPose();
    SmartDashboard.putString("april tag pose", closestAprilTagPose.toString());
    Command pathfindPath = AutoBuilder.pathfindToPose(
      translateCoord(closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.1),
      //closestAprilTagPose,
        new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)));
    //pathfindPath.schedule();
             
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath pathToFront = new PathPlannerPath(
          PathPlannerPath.waypointsFromPoses(
            translateCoord(closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.1),
            //drive.getState().Pose,
              closestAprilTagPose),
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
          null, 
          new GoalEndState(0.0, closestAprilTagPose.getRotation())
      );
      pathToFront.preventFlipping = true;
      fullPath = pathfindPath.andThen(AutoBuilder.followPath(pathToFront));
      fullPath.schedule(); 
    } catch (Exception e) {
      e.printStackTrace();
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (fullPath != null) {
      fullPath.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private Pose2d getClosestReefAprilTagPose() {
    HashMap<Integer, Pose2d> aprilTagsToAlignTo = AprilTagPositions.WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        aprilTagsToAlignTo = AprilTagPositions.WELDED_RED_CORAL_APRIL_TAG_POSITIONS;
      }
    }
    //Pose2d currentPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-left");
    Pose2d currentPose = drive.getState().Pose;
    //drive.getState().Pose = currentPose;
    SmartDashboard.putString("Robot Pose", currentPose.toString());
    Pose2d closestPose = new Pose2d();
    double closestDistance = Double.MAX_VALUE;
    Integer aprilTagNum = -1;

    for (Map.Entry<Integer, Pose2d> entry : aprilTagsToAlignTo.entrySet()) {
      Pose2d pose = entry.getValue();
      double distance = findDistanceBetween(currentPose, pose);
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = pose;
        aprilTagNum = entry.getKey();
      }
    }
    SmartDashboard.putString("April Tag NUmber", aprilTagNum.toString());
    Pose2d inFrontOfAprilTag = translateCoord(closestPose, closestPose.getRotation().getDegrees(),
        -Units.inchesToMeters(15.773));

    /*Pose2d leftOrRightOfAprilTag;
    if (isLeftBumper) {
      leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.1232265);
    } else {
      leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, -0.1232265);
    } */
/* 
    if (List.of(11, 10, 9, 22, 21, 20).contains(aprilTagNum)) {
      if (isLeftBumper) {
        leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, -0.1232265);
      } else {
        leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.1232265);
      }
    } */

    return translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.2); //0.2 is distance in meters from left limelight to endeffector
  } 

  private Pose2d translateCoord(Pose2d originalPose, double degreesRotate, double distance) {
    double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
    double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

    return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
  }

  private double findDistanceBetween(Pose2d pose1, Pose2d pose2) {
    return Math.sqrt(Math.pow((pose2.getX() - pose1.getX()), 2) + Math.pow((pose2.getY() - pose1.getY()), 2));
  }
}