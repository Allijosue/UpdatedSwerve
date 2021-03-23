// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public static final double kMaxSpeed = 1.0; // this is speed
  public static final double kMaxAngularSpeed = Math.PI;// radians

  // location of each module in meters
  private final Translation2d m_frontLeftLocation = new Translation2d(-0.229, 0.223);// correct
  private final Translation2d m_frontRightLocation = new Translation2d(0.229, 0.223);// correct
  private final Translation2d m_backLeftLocation = new Translation2d(-0.229, -0.223);// correct
  private final Translation2d m_backRightLocation = new Translation2d(0.229, -0.223);// correct

  // motor ids/ must change this drive m channel, turn m channel
  private final SwerveModule m_frontLeft = new SwerveModule(1, 7);// correct
  private final SwerveModule m_frontRight = new SwerveModule(10, 6);// correct
  private final SwerveModule m_backLeft = new SwerveModule(0, 11);// correct
  private final SwerveModule m_backRight = new SwerveModule(2, 8);// correct

  // drive kinematics
  AHRS m_gyro;
 // private final AnalogGyro m_gyro = new AnalogGyro(0);

  private SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());
      //new SwerveDriveOdometry(kinematics, gyroAngle, initialPose)
  public Drivetrain() {
     
      try {
          /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
          /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
          /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
          m_gyro = new AHRS(Port.kMXP); 
      } catch (RuntimeException ex ) {
          DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      }
      m_gyro.reset();
  
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
