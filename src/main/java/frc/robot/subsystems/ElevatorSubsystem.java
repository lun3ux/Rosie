package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
//import com.revrobotics.spark.SparkLimitSwitch.Type;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final RelativeEncoder encoder;
 //   private final PIDController pid;

    private final SparkLimitSwitch bottomLimitSwitch;
    private final SparkLimitSwitch topLimitSwitch;

    // private static final double kP = 0.05;
    // private static final double kI = 0.0;
    // private static final double kD = 0.0;

    public ElevatorSubsystem(int motorCANID) {
        elevatorMotor = new SparkMax(motorCANID, MotorType.kBrushless);
        encoder = elevatorMotor.getEncoder();
 //       pid = new PIDController(kP, kI, kD);

       // elevatorMotor.restoreFactoryDefaults();

        bottomLimitSwitch = elevatorMotor.getReverseLimitSwitch();
        topLimitSwitch = elevatorMotor.getForwardLimitSwitch();

        // bottomLimitSwitch.enableLimitSwitch(true);
        // topLimitSwitch.enableLimitSwitch(true);

        encoder.setPosition(0);
    //    pid.setTolerance(0.05); // Adjust tolerance as needed
    }

    public void setVoltage(double voltage) {
        // Stop downward motion if bottom limit is hit
        if (voltage < 0 && isAtBottom()) {
            elevatorMotor.setVoltage(0);
        }
        // Stop upward motion if top limit is hit
        else if (voltage > 0 && isAtTop()) {
            elevatorMotor.setVoltage(0);
        } else {
            elevatorMotor.setVoltage(voltage);
        }
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public boolean isAtBottom() {
        return bottomLimitSwitch.isPressed();
    }

    public boolean isAtTop() {
        return topLimitSwitch.isPressed();
    }

    public void resetEncoderIfAtBottom() {
        if (isAtBottom()) {
            encoder.setPosition(0);
        }
    }

    public Command goToStow() {
        return new Command() {
          @Override
          public void initialize() {
            elevatorMotor.setVoltage(-6);  // down
          }
      
          @Override
          public boolean isFinished() {
            return bottomLimitSwitch.isPressed(); // define this
          }
      
          @Override
          public void end(boolean interrupted) {
            elevatorMotor.stopMotor();
          }
        };
      }
      
      public Command goToL3() {
        return new Command() {
          @Override
          public void initialize() {
            elevatorMotor.setVoltage(6); // up
          }
      
          @Override
          public boolean isFinished() {
            return topLimitSwitch.isPressed(); // define this
          }
      
          @Override
          public void end(boolean interrupted) {
            elevatorMotor.stopMotor();
            //stopMotor();
          }
        };
      }

      public Command goToL2() {
        return new Command() {
          @Override
          public void initialize() {
            if (encoder.getPosition() < 50) {
              elevatorMotor.setVoltage(6); // up
            } else {
              elevatorMotor.setVoltage(-6); // down
            }
          }
      
          @Override
          public boolean isFinished() {
            return (encoder.getPosition() < 50 && encoder.getPosition() > 45); // define this
          }
      
          @Override
          public void end(boolean interrupted) {
            elevatorMotor.stopMotor();
          }
        };
      }
      
            
      public Command ElevatorUp() {
        return new Command() {
          @Override
          public void initialize() {
            elevatorMotor.setVoltage(6);
          }
      
          @Override
          public void end(boolean interrupted) {
            elevatorMotor.stopMotor();
          }
        };
      }

      public Command ElevatorDown() {
        return new Command() {
          @Override
          public void initialize() {
            elevatorMotor.setVoltage(-6);
          }
      
          @Override
          public void end(boolean interrupted) {
            elevatorMotor.stopMotor();
          }
        };
      }



    @Override
    public void periodic() {
        resetEncoderIfAtBottom();

        // Example for SmartDashboard or logging
        SmartDashboard.putBoolean("Bottom Limit", isAtBottom());
        SmartDashboard.putBoolean("Top Limit", isAtTop());
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Elevator Power", elevatorMotor.getAppliedOutput());
    }
}