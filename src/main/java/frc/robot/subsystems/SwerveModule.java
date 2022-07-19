package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    //motor encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;
    
    //connect to turning motor, keeps values when robot off
    //maybe wrong vendor library (rev)
    //connect to analog input on roborio
    private final edu.wpi.first.wpilibj.AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    /*when first assemble robot, the absolute encoder reading
    may differ from the actual wheel angle. Record value now,
    compensate later*/

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
            
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new edu.wpi.first.wpilibj.AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        //only considering P value, change later if need more specific
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        //tells PID that system is circular, and the two points are connected 
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
        }
        
        //some methods to get the encoder values
        //first 4 concern built in encoders

        public double getDrivePosition(){
            return driveEncoder.getPosition();
        }

        public double getTurningPosition(){
            return turningEncoder.getPosition();
        }

        public double getDriveVelocity(){
            return driveEncoder.getVelocity();
        }

        public double getTurningVelocity(){
            return turningEncoder.getVelocity();
        }

        public double getAbsoluteEncoderRad(){
            //gives what percent of a rotation it is reading
            double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
            //convert to radians
            angle *= 2.0 * Math.PI;
            //get actual wheel angles
            angle -= absoluteEncoderOffsetRad;
            //mult -1 if reversed
            return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
        }

        public void resetEncoders() {
            driveEncoder.setPosition(0);
            //abs encoder aligned with actual wheel angle
            turningEncoder.setPosition(getAbsoluteEncoderRad());
        }

        public SwerveModuleState getState(){
            return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
        }

        public void setDesiredState(SwerveModuleState state){
            //if no substantial driving velocity, ignore command
            if (Math.abs(state.speedMetersPerSecond)<0.001){
                stop();
                return;
            }

            //never have to move more than 90 degrees
            state = SwerveModuleState.optimize(state, getState().angle);
            driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
            SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
        }

        public void stop(){
            driveMotor.set(0);
            turningMotor.set(0);
        }
}

