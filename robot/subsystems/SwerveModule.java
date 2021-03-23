// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  WPI_TalonSRX turnM;
  WPI_VictorSPX driveM;
  double actualEnc;
  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI;
  private static final int kEncoderResolution = 1024;
  private final ProfiledPIDController m_turningPIDController = 
  new ProfiledPIDController(
  1, 
  0, 
  0, 
  new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);//must be determined for my own bot


  //constructing the swerve module
  public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
    driveM = new WPI_VictorSPX(driveMotorChannel);
    turnM = new WPI_TalonSRX(turningMotorChannel);
    turnM.configFactoryDefault();
    turnM.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    actualEnc = 2 * Math.PI/turnM.getSelectedSensorPosition();//this allows for conversion of encoder resolution to degrees/radians

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  //return the current state of the module
  public SwerveModuleState getState(){
    return new SwerveModuleState(0.01, new Rotation2d(actualEnc));
    }


    //sets the desired state of the modules
    public void setDesiredState(SwerveModuleState desiredState){
      //optimize to avoid more than 90 degree turns
      SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(actualEnc));

      /* // Calculate the drive output from the drive PID controller.
    final double driveOutput =
    m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
*///would have to look more into this, right now, just focus on turning

      final double output = m_turningPIDController.calculate(actualEnc, state.angle.getRadians());
      final double turnFeedForward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
      SmartDashboard.putNumber("turn output", output + turnFeedForward);
      //turnM.set(ControlMode.PercentOutput, output + turnFeedForward);


    }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
