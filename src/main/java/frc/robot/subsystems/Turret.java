// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Electrical;
import frc.robot.Constants.Btns;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Turret extends SubsystemBase {
  private CANSparkMax turretMotor;
  private RelativeEncoder turretEncoder;
  private SparkMaxPIDController m_turretPIDController;
  private double kP, kFF, kI, kD, kIz, kMaxOutput, kMinOutput, setpointP, setpointV;
  private float limitU, limitL;
  private boolean turnaround1 = false, turnaround2 = false;
  private Pose2d pose = new Pose2d(0, 0, new Rotation2d(0));
  public SwerveDriveOdometry m_odometry;

  /** Creates a new Turret. */
  public Turret() {

    // Odometry class for tracking robot pose
    double gyroAng = SmartDashboard.getNumber("GYRO ANGLE", 0.0);
    m_odometry = new SwerveDriveOdometry(SwerveConstants.kDriveKinematics,
        Rotation2d.fromDegrees((gyroAng + 180) * (SwerveConstants.kGyroReversed ? 1.0 : -1.0)));

    limitU = (float) SmartDashboard.getNumber("TurLimU", 223f);
    limitL = (float) SmartDashboard.getNumber("TurLimL", -137f);

    turretMotor = new CANSparkMax(Electrical.turret, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setSmartCurrentLimit(30);
    turretMotor.setIdleMode(IdleMode.kBrake);
    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kForward, limitU);

    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kReverse, limitL);

    turretMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    turretEncoder = turretMotor.getEncoder();

    turretEncoder.setPositionConversionFactor(36.0 / 235 / 25.0 * 360.0);

    m_turretPIDController = turretMotor.getPIDController();

    // PID coefficients
    kP = 0.35;
    kFF = 1. / 11000.;
    kI = 0;
    // TODO: try taking out to stop swiveling
    kD = 0; //0.0005;
    kIz = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // // set PID coefficients
    m_turretPIDController.setP(kP);
    m_turretPIDController.setI(kI);
    m_turretPIDController.setD(kD);
    m_turretPIDController.setIZone(kIz);
    m_turretPIDController.setFF(kFF);
    m_turretPIDController.setOutputRange(kMinOutput, kMaxOutput);

    turretMotor.burnFlash();
  }

  public void setPosition() {
    m_turretPIDController.setReference(setpointP, ControlType.kPosition);
  }

  public void turretVision() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double dTheta = SmartDashboard.getNumber("TurnRate", 0.0);
    String matchColor = SmartDashboard.getString("DriverStation", "Red");
    String detectedColor = SmartDashboard.getString("Detected Color", matchColor);
    String notColor = "unknown";
    double offset = 0;

    if (matchColor.equals("Blue")) {
      notColor = "Red";
    } else if (matchColor.equals("Red")) {
      notColor = "Blue";
    }

    if (!turnaround1 && !turnaround2) {
      if (tx != 0) {
        if (detectedColor.equals(notColor) &&
            SmartDashboard.getBoolean("RejectOpps", true)
            && !SmartDashboard.getBoolean("BotEmpty", true)) {
              SmartDashboard.putBoolean("RejectingBall", true);
              m_turretPIDController.setReference((tx + 17) * 0.04, ControlType.kDutyCycle);
        } else {
        // for non-moving
        SmartDashboard.putBoolean("RejectingBall", false);
        m_turretPIDController.setReference(tx * 0.04, ControlType.kDutyCycle);
        // for moving
        // m_turretPIDController.setReference(tx * 0.02 - dTheta, ControlType.kDutyCycle);
        }
      } else {
        faceGoalOdometry();
        // make it always face the goal
      }
    }

    if (turretEncoder.getPosition() > 205 && tx > 12.0) {
      turnaround1 = true;
      m_turretPIDController.setReference(-115, ControlType.kPosition);
      // System.out.println("turnaround1 true");

    } else if (turretEncoder.getPosition() < -115 && tx < -12.0) {
      turnaround2 = true;
      m_turretPIDController.setReference(185, ControlType.kPosition);
      // System.out.println("turnaround2 true");
    }

    if (turnaround1 && turretEncoder.getPosition() < -100) {
      turnaround1 = false;
      // System.out.println("turnaround1 false");
    }
    if (turnaround2 && turretEncoder.getPosition() > 170) {
      turnaround2 = false;
      // System.out.println("turnaround2 false");
    }

    if (isStuck()) {
      turnaround1 = false;
      turnaround2 = false;
      // m_turretPIDController.setReference(0, ControlType.kDutyCycle);
    }

    // Failsafe no wrap around code
    // if(tx!= 0) {
    // m_turretPIDController.setReference(tx * 0.04, ControlType.kDutyCycle);
    // } else {
    // m_turretPIDController.setReference(0, ControlType.kDutyCycle);
    // }
  }

  public boolean isStuck() {
    if (Math.abs(turretMotor.getAppliedOutput()) > 0.045 && turretEncoder.getVelocity() < 0.025) {
      return true;
    }
    return false;
  }

  public void faceGoal() {
    double heading = -SwerveDrive.getHeading();

    while (heading > limitU) {
      heading -= 360;
    }
    while (heading < limitL) {
      heading += 360;
    }

    SmartDashboard.putNumber("target", heading);

    m_turretPIDController.setReference(heading, ControlType.kPosition);
  }

  public void faceGoalOdometry() {
    double rot = Math.toDegrees(pose.getRotation().getRadians()
        - Math.atan2(pose.getY(), pose.getX()));
    // System.out.println("ROT: " + rot);
    while (rot > limitU) {
      rot -= 360;
    }
    while (rot < limitL) {
      rot += 360;
    }

    m_turretPIDController.setReference(
        rot,
        ControlType.kPosition);
  }

  public boolean checkTurret() {
    boolean turretGood = true;
    if (SmartDashboard.getBoolean("RejectingBall", false)) {
      if (Math.abs(SmartDashboard.getNumber("tx", 0.0)) > 16
        && Math.abs(SmartDashboard.getNumber("TurretVelocity", 0.0)) < 1000 ) {
        turretGood = true;
      } else {
        turretGood = false;
      }
    } else {
      if (Math.abs(SmartDashboard.getNumber("tx", 0.0)) < 1.5
      && Math.abs(SmartDashboard.getNumber("TurretVelocity", 0.0)) < 1000) {
        turretGood = true;
      } else {
        turretGood  = false;
      }
    }

    return turretGood;
  }

  // need to know:
  // - where robot is
  //   - know where goal is in relation
  // - current speed
  public void shootMove() {
    ChassisSpeeds currentMovement = RobotContainer.swerve.currentMovement;  
    /** wrong cuz it's position not velocity
    Pose2d cur = RobotContainer.swerve.getPose();
    // goal is at  (-1, 0, 0)
    Pose2d goal = new Pose2d(-1, 0, new Rotation2d(0));
    Twist2d twist2d = new Twist2d(cur.getX() - goal.getX(), 
                                  cur.getY() - goal.getY(), 
                                  cur.getRotation().minus(goal.getRotation()).getRadians());
    Twist2d velTwist2d = new Twist2d(currentMovement.vxMetersPerSecond, 
                                     currentMovement.vyMetersPerSecond, 
                                     currentMovement.omegaRadiansPerSecond);
    // rn it is x,y cartesian style
    // figure out how to map this so that y is pointing toward target
    // and x is along circle
    // x = rcos0, y =  rsin0, x^2 + y^2 = r^2
    double r = Math.hypot(twist2d.dx, twist2d.dy);
    double dr = Math.hypot(velTwist2d.dx, velTwist2d.dy);
    double theta = Math.acos(twist2d.dx/r);
    double dTheta = Math.acos(velTwist2d.dx/r);
    // currently doesn't account for turning
    m_turretPIDController.setReference(-dTheta, ControlType.kDutyCycle);
    **/
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);

    double speed = Math.hypot(currentMovement.vxMetersPerSecond, currentMovement.vyMetersPerSecond);
    
    double goalRelativeSpeed = speed * Math.cos(Math.atan2(-currentMovement.vyMetersPerSecond,
                                                           currentMovement.vxMetersPerSecond)
                                - Math.toRadians(turretEncoder.getPosition() - tx + 90));
    double goalDist = SmartDashboard.getNumber("goalDist", 0.0);

    m_turretPIDController.setReference(tx * 0.04 + goalRelativeSpeed * 0.173 * goalDist, ControlType.kDutyCycle);
    SmartDashboard.putNumber("goalRelSpeed", goalRelativeSpeed);
  }

  public void setVelocity() {
    m_turretPIDController.setReference(setpointV, ControlType.kVelocity);
  }

  public void runRight() {
    turretMotor.set(0.25);
    turnaround1 = false;
    turnaround2 = false;
  }

  public void runLeft() {
    turretMotor.set(-0.25);
    turnaround1 = false;
    turnaround2 = false;
  }

  public void stop() {
    turretMotor.set(0);
    turnaround1 = false;
    turnaround2 = false;
  }

  public void zeroTurret() {
    turretEncoder.setPosition(0);
  }

  public void initDefaultCommand() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setpointP = SmartDashboard.getNumber("TurretSetpointP", 0.0);
    setpointV = SmartDashboard.getNumber("TurretSetpointV", 0.0);
    SmartDashboard.putNumber("TurretPos", turretEncoder.getPosition());
    SmartDashboard.putNumber("ty",
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0));
    SmartDashboard.putNumber("tx",
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0));
    SmartDashboard.putBoolean("Rumble", isStuck());

    if (SmartDashboard.getBoolean("Blind me", true)) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    // if (SmartDashboard.getBoolean("Rumble", false)) {
    //   JoystickButtons.m_driverController.setRumble(RumbleType.kLeftRumble, 0.1);
    // }

    pose = RobotContainer.swerve.getPoseTur();
    SmartDashboard.putNumber("TurretVelocity", turretEncoder.getVelocity());
    SmartDashboard.putBoolean("Turret Good?", checkTurret());

  }
}
