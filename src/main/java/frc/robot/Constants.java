// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // ---------------- Autopilot (vendor lib) ----------------
  private static final APConstraints kAutopilotConstraints =
      new APConstraints().withAcceleration(6.0).withJerk(5.0);

  private static final APProfile kAutopilotProfile =
      new APProfile(kAutopilotConstraints)
          .withErrorXY(Units.Centimeters.of(2))
          .withErrorTheta(Units.Degrees.of(2))
          .withBeelineRadius(Units.Centimeters.of(10));

  /** Shared Autopilot instance (create once, reuse everywhere). */
  public static final Autopilot kAutopilot = new Autopilot(kAutopilotProfile);

  /** Tuning for trench traversal commands (SmashTrench, etc.). */
  public static final class TrenchCommandConstants {
    // ALIGN -> RUN transition tolerance
    public static final double ALIGN_Y_TOL_METERS = 0.010;
    public static final double ALIGN_THETA_TOL_DEG = 10.0;

    // RUN behavior: push forward this distance along the chosen approach direction
    public static final double RUN_PUSH_DISTANCE_METERS = 2.0;
    public static final double RUN_PUSH_DONE_TOL_METERS = 0.02;

    // Autopilot target config
    public static final double ALIGN_END_VELOCITY_MPS = 3.0;

    // Heading controller (ProfiledPIDController)
    public static final double HEADING_KP = 8.0;
    public static final double HEADING_KI = 0.0;
    public static final double HEADING_KD = 0.4;

    /** Max angular velocity (rad/s). */
    public static final double HEADING_MAX_VEL_RAD_PER_SEC = 20.0;

    /** Max angular acceleration (rad/s^2). */
    public static final double HEADING_MAX_ACCEL_RAD_PER_SEC2 = 30.0;

    private TrenchCommandConstants() {}
  }

  /** Tuning for automatic bump traversal (SmashBumpCommand). */
  public static final class BumpCommandConstants {
    // ALIGN completion tolerances
    public static final double ALIGN_XY_TOL_METERS = 0.05;
    public static final double ALIGN_THETA_TOL_DEG = 5.0;

    // Autopilot target config during ALIGN
    public static final double ALIGN_END_VELOCITY_MPS = 2.0;

    // Sprint behavior (field-relative along approach direction)
    public static final double SPRINT_SPEED_MPS = 2.5;

    // Heading hold during sprint
    public static final double HEADING_KP = 6.0;
    public static final double HEADING_KI = 0.0;
    public static final double HEADING_KD = 0.2;

    /** After tilt clears (downhill), keep sprinting for this long then stop. */
    public static final double DOWNHILL_TIME_SEC = 0.1;

    private BumpCommandConstants() {}
  }

  /** Tuning for MegaTrackCommand (non-iterative moving-shot). */
  public static final class MegaTrackCommandConstants {
    // Heading controller (PIDController)
    public static final double HEADING_KP = 6.0;
    public static final double HEADING_KI = 0.0;
    public static final double HEADING_KD = 0.0;

    /** Scale for shooter acceleration feedforward computed from slope*dDot. */
    public static final double SHOOTER_ACCEL_FF_GAIN = 1.0;
    /** If true, include measured acceleration contribution in FF calculations (can be jumpy). */
    public static final boolean USE_ACCEL_IN_FF = false;

    // Shooting distance gates + heading-hold distance range
    public static final double MIN_SHOOT_DIST_METERS = 0.8;
    public static final double MAX_SHOOT_DIST_METERS = 5.8;
    public static final double HEADING_HOLD_MIN_DIST_METERS = 0.9;
    public static final double HEADING_HOLD_MAX_DIST_METERS = 5.6;

    // Shoot gating thresholds (hysteresis): enter tight, stay loose
    public static final double ENTER_FLYWHEEL_RPS_TOL = 1.0;
    public static final double EXIT_FLYWHEEL_RPS_TOL = 2.0;
    public static final double ENTER_HOOD_DEG_TOL = 1.0;
    public static final double EXIT_HOOD_DEG_TOL = 2.0;
    public static final double ENTER_HEADING_TOL_RAD = Math.toRadians(2.0);
    public static final double EXIT_HEADING_TOL_RAD = Math.toRadians(4.0);

    // Feed outputs
    public static final double FEEDER_RPS = 30.0;
    public static final double INDEXER_VOLTS = 10.0;

    private MegaTrackCommandConstants() {}
  }

  /** Tuning for MegaTrackIterativeCommand (iterative flight-time compensation). */
  public static final class MegaTrackIterativeCommandConstants {
    public static final String LOG_PREFIX = "AutoShootIter";

    // Heading controller (PIDController)
    public static final double HEADING_KP = 6.0;
    public static final double HEADING_KI = 0.0;
    public static final double HEADING_KD = 0.1;

    // Fixed-point iteration tuning
    public static final int MAX_ITERS = 6;
    public static final double EPS_T_SEC = 1e-3;
    public static final double RELAX = 0.7; // 1.0 = plain fixed-point, <1 adds damping
    public static final double MIN_T_SEC = 0.0;
    public static final double MAX_T_SEC = 2.0;

    // Shoot gating
    public static final double MIN_SHOOT_DIST_METERS = 0.8;
    public static final double MAX_SHOOT_DIST_METERS = 5.8;

    // Hysteresis: tighter enter, looser stay
    public static final double ENTER_HEADING_TOL_RAD = Math.toRadians(2.0);
    public static final double EXIT_HEADING_TOL_RAD = Math.toRadians(4.0);
    public static final double ENTER_FLYWHEEL_RPS_TOL = 1.5;
    public static final double EXIT_FLYWHEEL_RPS_TOL = 2.5;
    public static final double ENTER_HOOD_DEG_TOL = 1.0;
    public static final double EXIT_HOOD_DEG_TOL = 2.0;

    public static final double TRIGGER_AXIS_THRESHOLD = 0.25;
    public static final double FEEDER_RPS = 20.0;
    public static final double INDEXER_VOLTS = 10.0;

    private MegaTrackIterativeCommandConstants() {}
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** Intake CAN IDs and tuning. Update these to match your robot wiring. */
  public static final class IntakeConstants {
    // CAN IDs
    public static final int LEADER_MOTOR_ID = 50;
    public static final int FOLLOWER_MOTOR_ID = 19;
    /** TODO: set to your deploy/arm motor CAN ID (收放电机). */
    public static final int DEPLOY_MOTOR_ID = 0;

    // Motor directions
    public static final boolean LEADER_INVERTED = false;
    public static final boolean FOLLOWER_INVERTED = true;
    public static final boolean DEPLOY_INVERTED = false;

    // ---------------- Roller (吸球滚轮) ----------------
    /** Rollers voltage for intaking (volts). */
    public static final double ROLLER_INTAKE_VOLTS = 10.0;
    /** Rollers voltage for stopping (volts). */
    public static final double ROLLER_STOP_VOLTS = 0.0;

    // ---------------- Deploy (收放摆臂) ----------------
    /** Sensor-to-mechanism ratio for deploy motor (motor rotations per mechanism rotation). */
    public static final double DEPLOY_SENSOR_TO_MECH_RATIO = 1.0;

    /** Deploy position when stowed (mechanism rotations). TODO tune. */
    public static final double DEPLOY_POS_UP_ROT = 0.0;
    /** Deploy position when deployed down (mechanism rotations). TODO tune. */
    public static final double DEPLOY_POS_DOWN_ROT = 0.25;

    // Motion Magic (mechanism rotations/sec and rotations/sec^2)
    public static final double DEPLOY_MM_CRUISE_VELOCITY = 1.0;
    public static final double DEPLOY_MM_ACCELERATION = 2.0;
    public static final double DEPLOY_MM_JERK = 0.0;

    // Slot0 gains for MotionMagicTorqueCurrentFOC
    public static final double DEPLOY_KP = 80.0;
    public static final double DEPLOY_KI = 0.0;
    public static final double DEPLOY_KD = 0.0;
    public static final double DEPLOY_KS = 0.0;
    public static final double DEPLOY_KG = 0.0;
    public static final double DEPLOY_KV = 0.0;
    public static final double DEPLOY_KA = 0.0;

    // Flick/backfeed behavior (间歇性收放拨球) - simple oscillation
    public static final double FLICK_ON_SEC = 0.20;
    public static final double FLICK_OFF_SEC = 0.20;
    /** In flick mode, rollers voltage (volts). TODO tune (can be 0 or slight reverse). */
    public static final double FLICK_ROLLER_VOLTS = 0.0;

    private IntakeConstants() {}
  }

  public static final class FieldConstants {
    public static final double HUB_HEIGHT_METERS = 2.64;
    public static final Translation2d RED_HUB_LOCATION = new Translation2d(11.917, 4.030);
    public static final Translation2d BLUE_HUB_LOCATION = new Translation2d(4.623, 4.030);

    /** Estimated field center from hub locations (used for symmetry helpers). */
    public static final Translation2d FIELD_CENTER =
        new Translation2d(
            (RED_HUB_LOCATION.getX() + BLUE_HUB_LOCATION.getX()) / 2.0,
            (RED_HUB_LOCATION.getY() + BLUE_HUB_LOCATION.getY()) / 2.0);

    // ---------------- Trench alignment helpers ----------------
    /**
     * Standoff distance before the trench along the chosen approach line.
     *
     * <p>This is the "坡前指定位置" distance.
     */
    public static final double TRENCH_PRE_DISTANCE_METERS = 0.60;

    /** Trench usable alignment length along the trench axis (meters). */
    public static final double TRENCH_LENGTH_METERS = 0.25;

    /**
     * Base trench center (blue alliance, lower-left one) in field coordinates.
     *
     * <p>TODO: Set this to your measured field value.
     */
    public static final Translation2d BLUE_LL_TRENCH_CENTER = new Translation2d(4.623, 0.640);

    /**
     * Base trench axis heading (blue alliance, lower-left trench). This is the direction along the
     * trench length (the finite segment direction).
     *
     * <p>If your trench is "horizontal", set this to 0 deg. If it's "vertical", set to 90 deg.
     *
     * <p>IMPORTANT: Trench centers are symmetric about {@link #FIELD_CENTER} midlines, but trench
     * axis orientation is NOT assumed to change with the centers. The field trenches are parallel
     * in global field coordinates (e.g., both blue-side bumps share the same axis).
     */
    public static final Rotation2d BLUE_LL_TRENCH_AXIS_HEADING = Rotation2d.fromDegrees(90.0);

    private static Translation2d mirrorAboutXMidline(
        Translation2d point, Translation2d fieldCenter) {
      // Mirror across the vertical midline x = fieldCenter.x
      return new Translation2d(2.0 * fieldCenter.getX() - point.getX(), point.getY());
    }

    private static Translation2d mirrorAboutYMidline(
        Translation2d point, Translation2d fieldCenter) {
      // Mirror across the horizontal midline y = fieldCenter.y
      return new Translation2d(point.getX(), 2.0 * fieldCenter.getY() - point.getY());
    }

    /** Snap a field heading to the nearest multiple of 90 degrees. */
    private static Rotation2d snapTo90Deg(Rotation2d heading) {
      double deg = heading.getDegrees();
      double snappedDeg = Math.round(deg / 90.0) * 90.0;
      return Rotation2d.fromDegrees(snappedDeg);
    }

    /** Represents an approach reference segment for a trench (finite length). */
    public record TrenchApproachLine(
        Translation2d trenchCenter, Rotation2d approachHeading, double halfLengthMeters) {
      public Translation2d dirUnit() {
        return new Translation2d(approachHeading.getCos(), approachHeading.getSin());
      }

      /** Unit vector along the trench axis (perpendicular to approach direction). */
      public Translation2d axisUnit() {
        Rotation2d axisHeading = approachHeading.plus(Rotation2d.fromRotations(0.25)); // +90deg
        return new Translation2d(axisHeading.getCos(), axisHeading.getSin());
      }

      /** Closest point on the finite segment to the given point. */
      public Translation2d closestPointOnSegment(Translation2d point) {
        Translation2d a = axisUnit();
        Translation2d d = point.minus(trenchCenter);
        double t = d.getX() * a.getX() + d.getY() * a.getY(); // projection onto axis
        t = Math.max(-halfLengthMeters, Math.min(halfLengthMeters, t));
        return trenchCenter.plus(a.times(t));
      }

      /** Signed distance along the line direction from trenchCenter to point. */
      public double along(Translation2d point) {
        Translation2d d = point.minus(trenchCenter);
        Translation2d u = dirUnit();
        return d.getX() * u.getX() + d.getY() * u.getY();
      }

      /** Distance from the point to this finite segment (meters). */
      public double distanceToSegment(Translation2d point) {
        return point.minus(closestPointOnSegment(point)).getNorm();
      }

      /** Returns the "pre-trench" pose for this line. */
      public Pose2d preTrenchPose(double preDistanceMeters) {
        return preTrenchPose(preDistanceMeters, trenchCenter);
      }

      /** Returns the "pre-trench" pose using a chosen point on the segment. */
      public Pose2d preTrenchPose(double preDistanceMeters, Translation2d pointOnSegment) {
        Translation2d u = dirUnit();
        Translation2d prePoint = pointOnSegment.minus(u.times(preDistanceMeters));
        return new Pose2d(prePoint, approachHeading);
      }
    }

    /**
     * 4 trench centers, generated by mirror symmetry about the field midlines (x = {@link
     * #FIELD_CENTER}.x and y = {@link #FIELD_CENTER}.y).
     */
    public static final Translation2d[] TRENCH_CENTERS =
        new Translation2d[] {
          BLUE_LL_TRENCH_CENTER,
          mirrorAboutXMidline(BLUE_LL_TRENCH_CENTER, FIELD_CENTER),
          mirrorAboutYMidline(BLUE_LL_TRENCH_CENTER, FIELD_CENTER),
          mirrorAboutYMidline(
              mirrorAboutXMidline(BLUE_LL_TRENCH_CENTER, FIELD_CENTER), FIELD_CENTER)
        };

    /** 4 trench axis headings (direction along trench length), NOT changed with the centers. */
    public static final Rotation2d[] TRENCH_AXIS_HEADINGS =
        new Rotation2d[] {
          BLUE_LL_TRENCH_AXIS_HEADING,
          BLUE_LL_TRENCH_AXIS_HEADING,
          BLUE_LL_TRENCH_AXIS_HEADING,
          BLUE_LL_TRENCH_AXIS_HEADING
        };

    /**
     * 8 approach reference lines: for each trench, we provide two approach directions normal to the
     * trench axis (axis +90 and axis -90). This matches the "8 条预瞄参考线" idea.
     *
     * <p>IMPORTANT: Segment direction = trench axis; Pose heading = approach direction.
     */
    public static final TrenchApproachLine[] TRENCH_APPROACH_LINES =
        new TrenchApproachLine[] {
          // trench 0
          new TrenchApproachLine(
              TRENCH_CENTERS[0],
              TRENCH_AXIS_HEADINGS[0].plus(Rotation2d.fromDegrees(90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          new TrenchApproachLine(
              TRENCH_CENTERS[0],
              TRENCH_AXIS_HEADINGS[0].plus(Rotation2d.fromDegrees(-90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          // trench 1
          new TrenchApproachLine(
              TRENCH_CENTERS[1],
              TRENCH_AXIS_HEADINGS[1].plus(Rotation2d.fromDegrees(90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          new TrenchApproachLine(
              TRENCH_CENTERS[1],
              TRENCH_AXIS_HEADINGS[1].plus(Rotation2d.fromDegrees(-90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          // trench 2
          new TrenchApproachLine(
              TRENCH_CENTERS[2],
              TRENCH_AXIS_HEADINGS[2].plus(Rotation2d.fromDegrees(90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          new TrenchApproachLine(
              TRENCH_CENTERS[2],
              TRENCH_AXIS_HEADINGS[2].plus(Rotation2d.fromDegrees(-90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          // trench 3
          new TrenchApproachLine(
              TRENCH_CENTERS[3],
              TRENCH_AXIS_HEADINGS[3].plus(Rotation2d.fromDegrees(90.0)),
              TRENCH_LENGTH_METERS / 2.0),
          new TrenchApproachLine(
              TRENCH_CENTERS[3],
              TRENCH_AXIS_HEADINGS[3].plus(Rotation2d.fromDegrees(-90.0)),
              TRENCH_LENGTH_METERS / 2.0)
        };

    /**
     * Returns the best "pre-trench" alignment pose by snapping to the nearest approach reference
     * line.
     *
     * <p>Selection rule:
     *
     * <ul>
     *   <li>Pick the line with smallest perpendicular distance to the robot translation
     *   <li>Tie-breaker prefers the line where the robot is \"behind\" the trench center relative
     *       to the line direction (so the pre-pose is reachable without flipping 180°)
     * </ul>
     */
    public static Pose2d getNearestTrenchPrePose(Pose2d robotPose) {
      Translation2d p = robotPose.getTranslation();
      TrenchApproachLine best = null;
      double bestDist = Double.POSITIVE_INFINITY;
      boolean bestBehind = false;
      Translation2d bestClosest = null;

      for (TrenchApproachLine line : TRENCH_APPROACH_LINES) {
        Translation2d closest = line.closestPointOnSegment(p);
        double dist = line.distanceToSegment(p);
        // behind = robot is "before" the segment along the approach direction
        boolean behind =
            p.minus(closest).getX() * line.dirUnit().getX()
                    + p.minus(closest).getY() * line.dirUnit().getY()
                < 0.0;
        if (dist < bestDist - 1e-9) {
          best = line;
          bestDist = dist;
          bestBehind = behind;
          bestClosest = closest;
        } else if (Math.abs(dist - bestDist) <= 1e-9) {
          // Tie-breaker: prefer behind=true
          if (behind && !bestBehind) {
            best = line;
            bestBehind = true;
            bestClosest = closest;
          }
        }
      }

      if (best == null) {
        return robotPose;
      }

      // Keep translation on the chosen line, but snap the traverse heading to a 90° multiple.
      Translation2d u = best.dirUnit();
      Translation2d prePoint = bestClosest.minus(u.times(TRENCH_PRE_DISTANCE_METERS));
      return new Pose2d(prePoint, snapTo90Deg(best.approachHeading()));
    }

    /** ---------------- Bump alignment helpers ---------------- */
    /** Standoff distance before the bump along the chosen approach line (meters). */
    public static final double BUMP_PRE_DISTANCE_METERS = 1.0;

    /** Bump usable alignment length along the bump axis (meters). */
    public static final double BUMP_LENGTH_METERS = 0.5;

    /**
     * Base bump center (blue alliance, lower-left one) in field coordinates.
     *
     * <p>TODO: Set this to your measured field value.
     */
    public static final Translation2d BLUE_LL_BUMP_CENTER = new Translation2d(4.633, 2.480);

    /**
     * Base bump axis heading (blue alliance, lower-left bump). This is the direction along the bump
     * length (the finite segment direction).
     *
     * <p>IMPORTANT: Bump centers are symmetric about {@link #FIELD_CENTER} midlines, but bump axis
     * orientation is NOT assumed to change with the centers.
     */
    public static final Rotation2d BLUE_LL_BUMP_AXIS_HEADING = Rotation2d.fromDegrees(90.0);

    /** Represents an approach reference segment for a bump (finite length). */
    public record BumpApproachLine(
        Translation2d bumpCenter, Rotation2d approachHeading, double halfLengthMeters) {
      public Translation2d dirUnit() {
        return new Translation2d(approachHeading.getCos(), approachHeading.getSin());
      }

      /** Unit vector along the bump axis (perpendicular to approach direction). */
      public Translation2d axisUnit() {
        Rotation2d axisHeading = approachHeading.plus(Rotation2d.fromRotations(0.25)); // +90deg
        return new Translation2d(axisHeading.getCos(), axisHeading.getSin());
      }

      /** Closest point on the finite segment to the given point. */
      public Translation2d closestPointOnSegment(Translation2d point) {
        Translation2d a = axisUnit();
        Translation2d d = point.minus(bumpCenter);
        double t = d.getX() * a.getX() + d.getY() * a.getY(); // projection onto axis
        t = Math.max(-halfLengthMeters, Math.min(halfLengthMeters, t));
        return bumpCenter.plus(a.times(t));
      }

      /** Distance from the point to this finite segment (meters). */
      public double distanceToSegment(Translation2d point) {
        return point.minus(closestPointOnSegment(point)).getNorm();
      }
    }

    /** 4 bump centers, generated by mirror symmetry about the field midlines. */
    public static final Translation2d[] BUMP_CENTERS =
        new Translation2d[] {
          BLUE_LL_BUMP_CENTER,
          mirrorAboutXMidline(BLUE_LL_BUMP_CENTER, FIELD_CENTER),
          mirrorAboutYMidline(BLUE_LL_BUMP_CENTER, FIELD_CENTER),
          mirrorAboutYMidline(mirrorAboutXMidline(BLUE_LL_BUMP_CENTER, FIELD_CENTER), FIELD_CENTER)
        };

    /** 4 bump axis headings (direction along bump length), NOT changed with the centers. */
    public static final Rotation2d[] BUMP_AXIS_HEADINGS =
        new Rotation2d[] {
          BLUE_LL_BUMP_AXIS_HEADING,
          BLUE_LL_BUMP_AXIS_HEADING,
          BLUE_LL_BUMP_AXIS_HEADING,
          BLUE_LL_BUMP_AXIS_HEADING
        };

    /** 8 approach reference lines for bumps. */
    public static final BumpApproachLine[] BUMP_APPROACH_LINES =
        new BumpApproachLine[] {
          new BumpApproachLine(
              BUMP_CENTERS[0],
              BUMP_AXIS_HEADINGS[0].plus(Rotation2d.fromDegrees(90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[0],
              BUMP_AXIS_HEADINGS[0].plus(Rotation2d.fromDegrees(-90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[1],
              BUMP_AXIS_HEADINGS[1].plus(Rotation2d.fromDegrees(90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[1],
              BUMP_AXIS_HEADINGS[1].plus(Rotation2d.fromDegrees(-90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[2],
              BUMP_AXIS_HEADINGS[2].plus(Rotation2d.fromDegrees(90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[2],
              BUMP_AXIS_HEADINGS[2].plus(Rotation2d.fromDegrees(-90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[3],
              BUMP_AXIS_HEADINGS[3].plus(Rotation2d.fromDegrees(90.0)),
              BUMP_LENGTH_METERS / 2.0),
          new BumpApproachLine(
              BUMP_CENTERS[3],
              BUMP_AXIS_HEADINGS[3].plus(Rotation2d.fromDegrees(-90.0)),
              BUMP_LENGTH_METERS / 2.0)
        };

    /**
     * Returns the best "pre-bump" alignment pose by snapping to the nearest bump approach segment.
     *
     * <p>NO 90° heading snap. Returned pose rotation equals chosen approach heading.
     */
    public static Pose2d getNearestBumpPrePose(Pose2d robotPose) {
      Translation2d p = robotPose.getTranslation();
      BumpApproachLine best = null;
      double bestDist = Double.POSITIVE_INFINITY;
      boolean bestBehind = false;
      Translation2d bestClosest = null;

      for (BumpApproachLine line : BUMP_APPROACH_LINES) {
        Translation2d closest = line.closestPointOnSegment(p);
        double dist = line.distanceToSegment(p);
        boolean behind =
            p.minus(closest).getX() * line.dirUnit().getX()
                    + p.minus(closest).getY() * line.dirUnit().getY()
                < 0.0;
        if (dist < bestDist - 1e-9) {
          best = line;
          bestDist = dist;
          bestBehind = behind;
          bestClosest = closest;
        } else if (Math.abs(dist - bestDist) <= 1e-9) {
          if (behind && !bestBehind) {
            best = line;
            bestBehind = true;
            bestClosest = closest;
          }
        }
      }

      if (best == null) {
        return robotPose;
      }

      Translation2d u = best.dirUnit();
      Translation2d prePoint = bestClosest.minus(u.times(BUMP_PRE_DISTANCE_METERS));
      return new Pose2d(prePoint, best.approachHeading());
    }

    public static final Translation2d getHubLocation(Alliance alliance) {

      return alliance == Alliance.Red ? RED_HUB_LOCATION : BLUE_HUB_LOCATION;
    }
  }

  /** Vision tuning for Limelight MegaTag2 pose updates. */
  public static final class VisionConstants {
    /** TA->XY std dev map (meters). Tune these points based on your camera mounting & lighting. */
    public static final InterpolatingDoubleTreeMap taToXYStdDevMeters =
        new InterpolatingDoubleTreeMap();

    /**
     * Theta std dev (radians). Use a huge value to effectively ignore vision heading correction.
     * (We also override the vision measurement rotation to the current gyro heading.)
     */
    public static final double thetaStdDevRad = 1.0e6;

    static {
      // Placeholder tuning points (TA is % of image, 0-100). Adjust to your robot.
      // Larger TA (bigger tag) => lower std dev (more trust)
      taToXYStdDevMeters.put(0.17, 0.08);
      taToXYStdDevMeters.put(0.22, 0.20);
      taToXYStdDevMeters.put(0.071, 0.35);
      taToXYStdDevMeters.put(0.046, 0.4);
      taToXYStdDevMeters.put(0.03, 0.7);
      taToXYStdDevMeters.put(0.01, 1.);
    }

    private VisionConstants() {}
  }

  public static final class AutoShootConstants {
    public static final double FlyTime = 0.9;
    public static InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
    /**
     * Flight time map: distance (m) -> flight time (s). Used by iterative moving-shot compensation.
     */
    public static InterpolatingDoubleTreeMap flightTimeMap = new InterpolatingDoubleTreeMap();

    public static final double MAX_SHOOTING_VELOCITY = 3;

    static {

      // hoodAngleMap.put(0.0, 0.0);
      // hoodAngleMap.put(0.506, 5.0);
      // hoodAngleMap.put(1.02, 10.0);
      // hoodAngleMap.put(1.550, 15.0);
      // hoodAngleMap.put(2.106, 20.0);
      // hoodAngleMap.put(2.70, 25.);
      // hoodAngleMap.put(3.34, 30.0);
      // hoodAngleMap.put(4.05, 35.0);
      // hoodAngleMap.put(4.85, 40.0);
      // hoodAngleMap.put(5.787, 45.);

      hoodAngleMap.put(1.072, 0.);
      hoodAngleMap.put(1.5, 1.);
      hoodAngleMap.put(2.02, 3.2);
      hoodAngleMap.put(2.65, 7.);
      hoodAngleMap.put(2.983, 10.);
      hoodAngleMap.put(3.42, 12.);
      hoodAngleMap.put(4.08, 18.);
      hoodAngleMap.put(5.05, 18.);

      shooterSpeedMap.put(1.072, 30.);
      shooterSpeedMap.put(1.5, 30.);
      shooterSpeedMap.put(2.02, 33.5);
      shooterSpeedMap.put(2.65, 35.3);
      shooterSpeedMap.put(2.983, 36.8);
      shooterSpeedMap.put(3.42, 37.3);
      shooterSpeedMap.put(4.08, 37.);
      shooterSpeedMap.put(5.05, 45.8);

      // Default flight-time map (placeholder). Start with constant FlyTime and tune with real data.
      flightTimeMap.put(0.0, FlyTime);
      flightTimeMap.put(1.0, FlyTime);
      flightTimeMap.put(2.0, FlyTime);
      flightTimeMap.put(3.0, FlyTime);
      flightTimeMap.put(4.0, FlyTime);
      flightTimeMap.put(5.0, FlyTime);

      // shooterSpeedMap.put(0.0, 54.48);
      // shooterSpeedMap.put(0.506, 54.69);
      // shooterSpeedMap.put(1.02, 55.32);
      // shooterSpeedMap.put(1.550, 56.40);
      // shooterSpeedMap.put(2.106, 57.98);
      // shooterSpeedMap.put(2.70, 60.117);
      // shooterSpeedMap.put(3.34, 62.913);
      // shooterSpeedMap.put(4.05, 66.513);
      // shooterSpeedMap.put(4.85, 71.125);
      // shooterSpeedMap.put(5.787, 77.053);
    }
  }
  /** Shooter CAN IDs and tuning. Update these to match your robot wiring & tuning. */
  public static final class ShooterConstants {
    // CAN IDs
    public static final int FLYWHEEL_1_LEADER_ID = 15;
    public static final int FLYWHEEL_1_FOLLOWER_ID = 16;
    /** TODO: set to your second shooter leader CAN ID. */
    public static final int FLYWHEEL_2_LEADER_ID = 0;
    /** TODO: set to your second shooter follower CAN ID. */
    public static final int FLYWHEEL_2_FOLLOWER_ID = 0;

    // Motor directions
    public static final boolean FLYWHEEL_LEADER_INVERTED = false;

    public static final MotorAlignmentValue FLYWHEEL_1_FOLLOWER_INVERTED =
        MotorAlignmentValue.Opposed;
    public static final MotorAlignmentValue FLYWHEEL_2_FOLLOWER_INVERTED =
        MotorAlignmentValue.Opposed;

    // Gear ratios (motor rotations per mechanism rotation)
    public static final double FLYWHEEL_SENSOR_TO_MECH_RATIO = 1.0;

    /** Flywheel/exit location relative to robot center (meters). +X forward, +Y left. */
    public static final double FLYWHEEL_OFFSET_X_METERS = 0.18;

    public static final double FLYWHEEL_OFFSET_Y_METERS = 0.0;

    // Flywheel closed-loop gains (Phoenix Slot0)
    public static final double FLYWHEEL_KP = 6; // 6
    public static final double FLYWHEEL_KI = 0.0;
    public static final double FLYWHEEL_KD = 0.0;
    public static final double FLYWHEEL_KV = 0.0;
    public static final double FLYWHEEL_KS = 3.5; // 3.5

    private ShooterConstants() {}
  }

  /** Shared hood (angle adjustment) constants. Used by {@code Hood} subsystem. */
  public static final class HoodConstants {
    // CAN IDs
    public static final int MOTOR_ID = 14;

    // Motor direction
    public static final boolean INVERTED = true;

    // Gear ratio (motor rotations per mechanism rotation)
    public static final double SENSOR_TO_MECH_RATIO = 31.875;

    // Motion Magic (mechanism rotations/sec and rotations/sec^2)
    public static final double MM_CRUISE_VELOCITY = 0.5;
    public static final double MM_ACCELERATION = 1.0;
    public static final double MM_JERK = 0.0;

    // Closed-loop gains (Phoenix Slot0)
    public static final double KP = 2300;
    public static final double KI = 0.0;
    public static final double KD = 230.0;
    public static final double KS = 0.0;
    public static final double KG = 2.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;

    private HoodConstants() {}
  }

  /** Hopper detection constants (two CANrange sensors). */
  public static final class HopperConstants {
    /** TODO: set to your CANrange sensor IDs. */
    public static final int CANRANGE_1_ID = 0;

    public static final int CANRANGE_2_ID = 0;

    /** Distance threshold (meters) for "piece present". Tune on your robot. */
    public static final double DETECTION_DISTANCE_METERS = 0.25;

    /** Debounce time (sec) for declaring hopper full / not full. */
    public static final double FULL_DEBOUNCE_SEC = 0.10;

    private HopperConstants() {}
  }

  /** LED (CANdle) constants. */
  public static final class LEDConstants {
    /** TODO: set to your CANdle CAN ID. */
    public static final int CANDLE_ID = 0;

    /** Indices 0-7 onboard, 8-399 strip. Tune to your wiring. */
    public static final int LED_START_INDEX = 8;

    public static final int LED_END_INDEX = 399;

    private LEDConstants() {}
  }

  /**
   * Drivetrain torque-current deadband tuning (applies when
   * DriveMotorClosedLoopOutput=TorqueCurrentFOC).
   */
  public static final class DrivetrainConstants {
    /**
     * Deadband for torque-current requests (same units as {@code TorqueCurrentFOC.withOutput},
     * amps).
     */
    public static final double TORQUE_CURRENT_DEADBAND_AMPS = 0.0;

    // --- Tilt detection ---
    /** Angle threshold (deg) to declare robot is tilted. */
    public static final double TILT_TRIP_DEG = 4.0;
    /** Angle threshold (deg) to clear tilted state (hysteresis). */
    public static final double TILT_CLEAR_DEG = 2.0;
    /** Debounce time (sec) for declaring tilted. */
    public static final double TILT_TRIP_DEBOUNCE_SEC = 0.10;
    /** Debounce time (sec) for clearing tilted. */
    public static final double TILT_CLEAR_DEBOUNCE_SEC = 0.10;

    private DrivetrainConstants() {}
  }

  /** Feeder CAN IDs and tuning. Update these to match your robot wiring & tuning. */
  public static final class FeederConstants {
    // CAN IDs
    public static final int MOTOR_1_ID = 21;
    public static final int FOLLOWER_1_ID = 34;
    /** TODO: set to your second feeder leader CAN ID. */
    public static final int MOTOR_2_ID = 0;
    /** TODO: set to your second feeder follower CAN ID. */
    public static final int FOLLOWER_2_ID = 0;

    // Motor direction
    public static final boolean INVERTED = false;
    /** Feeder follower alignment relative to leader. Use Opposed to run opposite direction. */
    public static final MotorAlignmentValue FOLLOWER_1_ALIGNMENT = MotorAlignmentValue.Opposed;

    public static final MotorAlignmentValue FOLLOWER_2_ALIGNMENT = MotorAlignmentValue.Opposed;

    // Gear ratio (motor rotations per mechanism rotation)
    public static final double SENSOR_TO_MECH_RATIO = 1.0;

    // Velocity closed-loop gains (Phoenix Slot0)
    public static final double KP = 10.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KV = 0.;
    public static final double KS = 5;

    private FeederConstants() {}
  }

  /** Indexer CAN IDs and tuning. Update these to match your robot wiring & tuning. */
  public static final class IndexerConstants {
    public static final int MOTOR_1_ID = 35;
    /** TODO: set to your second indexer CAN ID. */
    public static final int MOTOR_2_ID = 0;

    public static final boolean INVERTED = false;

    private IndexerConstants() {}
  }
}
