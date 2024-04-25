package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    CANSparkMax driveMotor;
    CANSparkMax rotationMotor;

    RelativeEncoder driveEncoder;
    RelativeEncoder rotationEncoder;

    AnalogInput absoluteEncoder;

    SparkPIDController drivePID;
    SparkPIDController rotationPID;

    int position;

    double driveSpeed = 0;

    double arcRadius;
    double inverseArcInput;

    double absEncoderDiff;

    double rotationTarget;
    double targetDiff;

    double rotationPointX = 0;
    double rotationPointY = 0;

    double pointDistance;
    double maxPointDistance;
    double currentModulePointDistance;

    double zeroPositionOffset[] = {0, 0, 0, 0};

    final double DRIVE_P = 0.15;
    final double DRIVE_I = 0.0;
    final double DRIVE_D = 0.015;

    final double ROTATION_P = 0.4;
    final double ROTATION_I = 0.0;
    final double ROTATION_D = 0.04;

    final double DRIVE_RAMP_RATE = 0.2;

    final double ROTATION_ENCODER_SCALE = 21.427;

    // final double MAX_ROTATION_ARC_X = 100;
    
    final double MAX_ROTATION_ARC_X = 60;

    public static final double[] X_POSITIONS = {-10.25, -10.25, 10.25, 10.25};
    public static final double[] Y_POSITIONS = {10.25, -10.25, -10.25, 10.25};
    // public static final double[] Y_POSITIONS = {7.25, -13.25, -13.25, 7.25};

    // going clockwise decreases value
    // final double[] ABSOLUTE_ENCODER_ZEROS = {1.183, 3.225, 3.913, 4.391};
    final double[] ABSOLUTE_ENCODER_ZEROS = {1.189, 1.852, 3.911, 0.676};

    final double[] TURN_ANGLES = {45, 315, 225, 135};

    final double ENCODER_DISTANCE_SCALE = 0.542;

    public SwerveModule(int driveCANID, int rotationCANID, int pos) {
        driveMotor = new CANSparkMax(driveCANID, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationCANID, MotorType.kBrushless);

        driveMotor.setOpenLoopRampRate(DRIVE_RAMP_RATE);
        driveMotor.setSmartCurrentLimit(70);

        rotationMotor.setOpenLoopRampRate(DRIVE_RAMP_RATE);
        rotationMotor.setSmartCurrentLimit(40);

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        absoluteEncoder = new AnalogInput(pos);
        
        drivePID = driveMotor.getPIDController();
        rotationPID = rotationMotor.getPIDController();

        drivePID.setP(DRIVE_P);
        drivePID.setD(DRIVE_D);

        rotationPID.setP(ROTATION_P);
        rotationPID.setD(ROTATION_D);

        position = pos;

        resetEncoders(zeroPositionOffset);
    }

    public void drive(double magnitude, double driveAngle, double arcInput, boolean autoDrive, boolean maxSpin) {
        // Check for input from the Left Analog Stick
        if (magnitude != 0) {
            driveSpeed = magnitude;

            // Check if there is also input from the Right Analog Stick
            if (arcInput != 0) {

                // if (arcInput < 0) {
                //     arcInput = -Math.pow(-arcInput, 0.75);
                // } else {
                //     arcInput = Math.pow(arcInput, 0.75);
                // }

                // Get the inverse of the Right stick input because a smaller input will result in a larger arc
                if (arcInput < 0) {
                    inverseArcInput = -1 - (arcInput * 0.95);
                } else {
                    inverseArcInput = 1 - (arcInput * 0.95);
                }

                // Calculate the imaginary point to rotate around
                rotationPointX = Math.cos(driveAngle * (Math.PI / 180)) * inverseArcInput * MAX_ROTATION_ARC_X;
                rotationPointY = -Math.sin(driveAngle * (Math.PI / 180)) * inverseArcInput * MAX_ROTATION_ARC_X;

                // Handle division by zero condition
                if (Y_POSITIONS[position] == rotationPointY) {
                    rotationTarget = ROTATION_ENCODER_SCALE / 4;
                // Calculate the slope of each wheel from the imaginary point to rotate around
                } else {
                    rotationTarget = (Math.atan((X_POSITIONS[position] - rotationPointX) /
                                                (Y_POSITIONS[position] - rotationPointY))
                                          * 180 / Math.PI / 360 * ROTATION_ENCODER_SCALE);

                    if (Y_POSITIONS[position] < rotationPointY) {
                        rotationTarget += (ROTATION_ENCODER_SCALE / 2);
                    }
                }

                // Rotate the wheel to be perpendicular to the slope of the wheel from the imaginary point
                if (arcInput < 0) {
                    rotationTarget = rotationTarget - (ROTATION_ENCODER_SCALE / 4);
                } else {
                    rotationTarget = rotationTarget + (ROTATION_ENCODER_SCALE / 4);
                }  

                maxPointDistance = 0;

                // Determine which wheel is the furthest from the imaginary point
                for (int i = 0; i <= 3; i++) {
                    pointDistance = Math.hypot((X_POSITIONS[i] - rotationPointX), (Y_POSITIONS[i] - rotationPointY));

                    if (maxPointDistance < pointDistance) {
                        maxPointDistance = pointDistance;
                    }

                    if (i == position) {
                        currentModulePointDistance = pointDistance;
                    }
                }

                // Scale each wheel speed based on the max speed
                driveSpeed *= (currentModulePointDistance / maxPointDistance);
            // If there is only Left Stick input, set all wheels to the direction of the Left Stick
            } else {
                rotationTarget = (driveAngle / 360.0) * ROTATION_ENCODER_SCALE;
            }
        // If there is only Right Stick input, form a circle with the wheels and set the speed of each wheel to the input magnitude
        } else if (arcInput != 0) {
            if (maxSpin) {
                driveSpeed = arcInput;
            } else {
                driveSpeed = arcInput / 2.0;
            }
            
            rotationTarget = (TURN_ANGLES[position] / 360.0) * ROTATION_ENCODER_SCALE;
        } else {
            driveSpeed = 0;
        }

        // Set the speed of the motors
        if (driveSpeed != 0) {
            // Determine which direction to rotate wheels for greatest efficiency
            targetDiff = rotationTarget - rotationEncoder.getPosition();

            // The most the wheel should need to rotate is 180 degrees, so get the target difference within that
            while (targetDiff > ROTATION_ENCODER_SCALE / 2) {
                rotationTarget -= ROTATION_ENCODER_SCALE;
                targetDiff = rotationTarget - rotationEncoder.getPosition();
            }

            while (targetDiff < -ROTATION_ENCODER_SCALE / 2) {
                rotationTarget += ROTATION_ENCODER_SCALE;
                targetDiff = rotationTarget - rotationEncoder.getPosition();
            }

            // If the target is with +/- 90 degrees, turn as normal
            if ((targetDiff >= -ROTATION_ENCODER_SCALE / 4) &&
                (targetDiff <= ROTATION_ENCODER_SCALE / 4)) {
                
            // If the target is 90 to 180 degrees away, turn the opposite direction and set the wheel to drive backwards
            } else if ((targetDiff > ROTATION_ENCODER_SCALE / 4) &&
                       (targetDiff <= ROTATION_ENCODER_SCALE / 2)) {
                
                rotationTarget -= ROTATION_ENCODER_SCALE / 2;
                driveSpeed *= -1;
            // If the target is -90 to -180 degrees away, turn the opposite direction and set the wheel to drive backwards 
            } else if ((targetDiff < -ROTATION_ENCODER_SCALE / 4) &&
                       (targetDiff >= -ROTATION_ENCODER_SCALE / 2)) {
            
                rotationTarget += ROTATION_ENCODER_SCALE / 2;
                driveSpeed *= -1;
            }
 
            // Set the rotate motor speed based on the pid controller
            rotationPID.setReference(rotationTarget, CANSparkMax.ControlType.kPosition);

            // If driving in Auto, don't drive until wheels are at the correct orientation
            if (autoDrive &&
               (Math.abs(rotationTarget - rotationEncoder.getPosition()) < 2)) {
                
                driveMotor.set(0);
            } else {
                driveMotor.set(driveSpeed);
            }
        } else {
            driveMotor.set(0);
            rotationMotor.set(0);
        }
    }

    public void resetEncoders(double offset[]) {
        absEncoderDiff = absoluteEncoder.getVoltage() - (ABSOLUTE_ENCODER_ZEROS[position] + offset[position]);

        if (absEncoderDiff < 0) {
            absEncoderDiff += RobotController.getVoltage5V();
        }

        rotationEncoder.setPosition((RobotController.getVoltage5V() - absEncoderDiff) / RobotController.getVoltage5V() * ROTATION_ENCODER_SCALE);
        
        SmartDashboard.putNumber("Absolute Encoder Zero - " + position, (ABSOLUTE_ENCODER_ZEROS[position] + offset[position]));

    }

    public double getDriveEncoder() {
        return driveEncoder.getPosition();
    }

    public void setCoastMode() {
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setBrakeMode() {
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    public void printValues() {
        SmartDashboard.putNumber("Rotation Point X - " + position, rotationPointX);
        SmartDashboard.putNumber("Rotation Point Y - " + position, rotationPointY);

        SmartDashboard.putNumber("Rotation Encoder - " + position, rotationEncoder.getPosition());
        SmartDashboard.putNumber("Rotation Target - " + position, rotationTarget);

        SmartDashboard.putNumber("Drive Encoder - " + position, driveEncoder.getPosition());
        SmartDashboard.putNumber("Drive Speed - " + position, driveSpeed);

        SmartDashboard.putNumber("Absolute Encoder - " + position, absoluteEncoder.getVoltage());
    }

    public double getYPosition(int pos) {
        return Y_POSITIONS[pos];
    }
    public double getXPosition(int pos) {
        return X_POSITIONS[pos];
    }
}
