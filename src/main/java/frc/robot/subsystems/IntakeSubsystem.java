package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor;

    public IntakeSubsystem(int motorCANID) {
        intakeMotor = new SparkMax(motorCANID, MotorType.kBrushed);
    }

    public Command Outtake() {
      return new Command() {
        @Override
        public void initialize() {
          intakeMotor.setVoltage(12);
        }

        @Override
        public void end(boolean interrupted) {
          intakeMotor.stopMotor();
        }
      };
    }

    public Command Intake() {
      return new Command() {
        @Override
        public void initialize() {
          intakeMotor.setVoltage(-5);
        }
    
        @Override
        public void end(boolean interrupted) {
          intakeMotor.stopMotor();
        }
      };
    }



    @Override
    public void periodic() {

      //   Example for SmartDashboard or logging
      //   SmartDashboard.putBoolean("Bottom Limit", isAtBottom());
      //   SmartDashboard.putBoolean("Top Limit", isAtTop());
      //   SmartDashboard.putNumber("Intake Position", getPosition());
      //   SmartDashboard.putNumber("Encoder Value", encoder.getPosition());
      //   SmartDashboard.putNumber("Intake Power", intakeMotor.getAppliedOutput());

    }
}