package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.geometry.Rotation2d.fromDegrees;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;

import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {
    SwerveDrive swerveDrive;
    private boolean fieldRelative = true;

    public SwerveDriveSubsystem(){

        double maximumSpeed = Constants.maximumSpeed;
        try{
             File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
             SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
            //  swerveDrive.setChassisDiscretization(true, .02);
             swerveDrive.setHeadingCorrection(true);

        } catch(IOException e){

            throw new RuntimeException(e);
        }
        

    } 
    public boolean isFieldRelative() {
        return fieldRelative;
    }


    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> {
            swerveDrive.drive(
                new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                  translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                fieldRelative,
                false
            );
        });
    }

    public void toggleDriveMode() {
        fieldRelative = !fieldRelative;
    }

    public void zeroGyro()
    {
        swerveDrive.zeroGyro();
        swerveDrive.setGyro(new Rotation3d(0,0,Math.PI));
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("Measured States", swervelib.telemetry.SwerveDriveTelemetry.measuredChassisSpeeds);
        SmartDashboard.putNumberArray("SwerveModuleStates", swervelib.telemetry.SwerveDriveTelemetry.measuredStates);
    }
}
