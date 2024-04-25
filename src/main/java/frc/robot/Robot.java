// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    XboxController primaryDriver = new XboxController(0);
    XboxController secondaryDriver = new XboxController(1);

    SwerveModule frontLeftSwerve;
    SwerveModule backLeftSwerve;
    SwerveModule backRightSwerve;
    SwerveModule frontRightSwerve;

    CANSparkMax intakeMotor = new CANSparkMax(9, MotorType.kBrushless);             // Negative pulls in
    CANSparkMax stagingMotor = new CANSparkMax(10, MotorType.kBrushless);           // Positive ejects, negative feeds shooter
    CANSparkMax shooterMotorTop = new CANSparkMax(11, MotorType.kBrushless);        // Positive shoots out
    CANSparkMax shooterMotorBottom = new CANSparkMax(12, MotorType.kBrushless);     // Positive shoots out
    CANSparkMax leadScrewMotor = new CANSparkMax(13, MotorType.kBrushless);         // Negative goes up, Positive goes down
    CANSparkMax climberMotor = new CANSparkMax(14, MotorType.kBrushless);           // Negative is up
    CANSparkMax armMotor = new CANSparkMax(15, MotorType.kBrushless);               // Negative goes up, Positive goes down

    RelativeEncoder leadScrewEncoder;
    RelativeEncoder climberEncoder;
    RelativeEncoder shooterTopEncoder;
    RelativeEncoder shooterBottomEncoder;

    PIDController armPID;
    PIDController drivePID;
    SparkPIDController leadScrewPID;

    DutyCycleEncoder armAbsoluteEncoder = new DutyCycleEncoder(9);

    PowerDistribution powerDistributionHub = new PowerDistribution(16, ModuleType.kRev);
    AHRS gyro;

    DigitalInput backNoteSensor = new DigitalInput(7);
    DigitalInput frontNoteSensor = new DigitalInput(8);
    DigitalInput shooterNoteSensor = new DigitalInput(4);

    DigitalInput upperArmLimit = new DigitalInput(5);
    DigitalInput lowerArmLimit = new DigitalInput(6);

    NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("limelight-shooter");
    NetworkTableEntry shooterTx = shooterTable.getEntry("tx");
    NetworkTableEntry shooterTy = shooterTable.getEntry("ty");
    NetworkTableEntry shooterTID = shooterTable.getEntry("tid");


    NetworkTable frontTable = NetworkTableInstance.getDefault().getTable("limelight-front");
    NetworkTableEntry frontTx = frontTable.getEntry("tx");
    NetworkTableEntry frontTy = frontTable.getEntry("ty");

    double shooterLimelightX = 0;
    double shooterLimelightY = 0;
    double shooterTidValue = 0;
    double frontLimelightX = 0;
    double frontLimelightY = 0;

    double armMotorSpeed = 0;
    double shooterSpeed = 0;
    double leadScrewSpeed = 0;

    Timer timer = new Timer();

    double startTime = 0;
    double diffTime = 0;
    boolean timerStarted = false;

    double leftX;
    double leftY;
    double rightX;

    double secondaryRightY;
    double secondaryLeftY;

    double magnitude;
    double driveAngle;
    double angleDiff;
    double autoDriveDirection;
    double startGyroAngle;
    double autoTurnAngle;
    double autoDelay;
    double gyroOffset = 0;

    double resetEncoderCounter;

    boolean backNoteSensorHit;
    boolean frontNoteSensorHit;
    boolean shooterNoteSensorHit;
    boolean gamePieceStaged;
    boolean useMaxSpin = false;
    double gamePieceStagedTimer;
    boolean noteStaged;

    int ledLightCounter;

    double frontLeftArcLength;
    double backLeftArcLength;
    double backRightArcLength;
    double frontRightArcLength;
    double speedScale;

    double frontLeftDistDiff;
    double frontRightDistDiff;

    double armTarget;
    double armReference;
    boolean armOverride;

    double limelightOffsetX = 0;

    double targetLeadScrewPosition = 0;
    double leadScrewOffset = 0;
    double shooterSpeedOffset = 0;
    double shooterTargetSpeed = 0;

    int aimingCounter = 0;

    boolean shooterAtTarget = false;
    boolean leadScrewOffsetPressed = false;

    double prevRightX = 0;
    double currentAngle = 0;
    double turnAngle = 0;

    SendableChooser<String> autoChooser = new SendableChooser<String>();
    SendableChooser<String> delayChooser = new SendableChooser<String>();

    int autoStage;

    double[] zeroPositionOffset = {0.0, 0.0, 0.0, 0.0};

    final double DEAD_ZONE = 0.15;
    final double SLOW_SPEED = 0.4;
    final double RIGHTX_SCALE = 80;
    final double SLOW_SCALE = 0.5;

    final double RAMP_RATE = 0.2;

    final double LEAD_SCREW_SCALE = 1.0;
    final double LEAD_SCREW_SPEED_SCALE = 1.0;
    final double SHOOTER_SPEED = 0.3;
    // final double SHOOTER_SPEED_OFFSET = 0.026;
    final double SHOOTER_SPEED_OFFSET = 0.1;
    final double SHOOTER_SPEED_TO_VELOCITY = 5300;
    final double SHOOTER_TARGET_THRESHOLD = 0.8;

    final double STAGING_MOTOR_SPEED = 1.0;
    final double STAGING_MOTOR_ALIGNMENT_SPEED = 0.4;

    final double INTAKE_MOTOR_SPEED = 1.0;

    final double ARM_ABSOLUTE_ZERO = 0.228;
    final double ARM_MOTOR_SPEED_SCALE = 10.0;
    final double ARM_MOTOR_UP_CAP = -1.0;
    final double ARM_MOTOR_DOWN_CAP = 0.85;

    final double ARM_STAGING_POSITION = 0.73;
    final double ARM_AMP_POSITION = 0.463;
    final double ARM_TRAP_POSITION = 0.382;
    final double ARM_MAX_HEIGHT = 0.309;

    final double ARM_TOLERANCE = 1.0;
    final double ARM_SPEED_SCALE = 0.1;

    final double SHOOTER_LIMELIGHT_X_SCALE = 0.008;

    final double CLIMBER_MIN_HEIGHT = 0.0;
    final double CLIMBER_HANG_HEIGHT = 150;
    final double CLIMBER_MAX_HEIGHT = 280;
    final double CLIMBER_CENTER_CLIMB_HEIGHT = 240;
    final double CLIMBER_SPEED_SCALE = 0.05;

    // final double ARM_P = 8.5;
    // final double ARM_I = 0.0;
    // final double ARM_D = 0.4;
    final double ARM_P = 7.0;
    final double ARM_I = 0.06;
    final double ARM_D = 0.02;

    // final double DRIVE_P = 0.045;
    // final double DRIVE_I = 0.0;
    // final double DRIVE_D = 0.003;

    final double DRIVE_P_HIGH = 0.045;
    // final double DRIVE_P_LOW = 0.0375;
    final double DRIVE_P_LOW = 0.03;

    final double DRIVE_I = 0.0;
    final double DRIVE_D = 0.003;

    final double LEAD_SCREW_P = 0.055;
    final double LEAD_SCREW_I = 0.0;
    final double LEAD_SCREW_D = 0.005;

    final double FRONT_LEFT_SCALE = .985;
    final double BACK_LEFT_SCALE = .99;
    final double BACK_RIGHT_SCALE = 1.0;
    final double FRONT_RIGHT_SCALE = .995;

    // final double FRONT_LEFT_SCALE = 1.0;
    // final double BACK_LEFT_SCALE = 0.991;
    // final double BACK_RIGHT_SCALE = 0.992;
    // final double FRONT_RIGHT_SCALE = 0.979;

    final double AUTO_DRIVE_SPEED_SCALE = 0.05;
    final double 
    AUTO_ARC_ANGLE_TOLERANCE = 1.0;

    final double ENCODER_DISTANCE_SCALE = 0.565;
    // final double ENCODER_DISTANCE_SCALE = 0.542;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        frontLeftSwerve = new SwerveModule(1, 2, 0);
        backLeftSwerve = new SwerveModule(3, 4, 1);
        backRightSwerve = new SwerveModule(5, 6, 2);
        frontRightSwerve = new SwerveModule(7, 8, 3);
    
        setBrakeMode();

        try {
            gyro = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            System.out.println("Error instantiating navx: " + ex.getMessage());
        }

        gyro.reset();

        timer.start();

        intakeMotor.setOpenLoopRampRate(RAMP_RATE);
        stagingMotor.setOpenLoopRampRate(RAMP_RATE);
        shooterMotorTop.setOpenLoopRampRate(RAMP_RATE);
        shooterMotorBottom.setOpenLoopRampRate(RAMP_RATE);
        climberMotor.setOpenLoopRampRate(RAMP_RATE);
        armMotor.setOpenLoopRampRate(RAMP_RATE);
        leadScrewMotor.setOpenLoopRampRate(RAMP_RATE);

        intakeMotor.setSmartCurrentLimit(80);       // PDH 2
        stagingMotor.setSmartCurrentLimit(20);      // PDH 10
        shooterMotorTop.setSmartCurrentLimit(80);
        shooterMotorBottom.setSmartCurrentLimit(80);
        climberMotor.setSmartCurrentLimit(35);
        armMotor.setSmartCurrentLimit(80);
        leadScrewMotor.setSmartCurrentLimit(10);    // PDH 9

        leadScrewEncoder = leadScrewMotor.getEncoder();
        climberEncoder = climberMotor.getEncoder();
        shooterTopEncoder = shooterMotorTop.getEncoder();
        shooterBottomEncoder = shooterMotorBottom.getEncoder();;

        leadScrewEncoder.setPosition(0.0);
        climberEncoder.setPosition(0.0);

        armPID = new PIDController(ARM_P, ARM_I, ARM_D);
        drivePID = new PIDController(DRIVE_P_HIGH, DRIVE_I, DRIVE_D);

        leadScrewPID = leadScrewMotor.getPIDController();
        leadScrewPID.setP(LEAD_SCREW_P);
        leadScrewPID.setI(LEAD_SCREW_I);
        leadScrewPID.setD(LEAD_SCREW_D);

        shooterTable.getEntry("ledMode").setNumber(1);
        frontTable.getEntry("ledMode").setNumber(1);

        autoChooser.setDefaultOption("Six Piece", "Six Piece");
        autoChooser.addOption("Outside", "Outside");
        // autoChooser.addOption("Simple Left", "Simple Left");
        // autoChooser.addOption("Simple Right", "Simple Right");
        autoChooser.addOption("Backup", "Backup");

        SmartDashboard.putData("Auto Position", autoChooser);

        delayChooser.setDefaultOption("0", "0");
        delayChooser.addOption("1", "1");
        delayChooser.addOption("2", "2");
        delayChooser.addOption("3", "3");
        delayChooser.addOption("4", "4");
        delayChooser.addOption("5", "5");
        delayChooser.addOption("6", "6");
        delayChooser.addOption("7", "7");
        delayChooser.addOption("8", "8");
        delayChooser.addOption("9", "9");
        delayChooser.addOption("10", "10");

        SmartDashboard.putData("Auto Delay", delayChooser);
        shooterTable.getEntry("pipeline").setNumber(0); // near shot
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        frontLeftSwerve.printValues();
        backLeftSwerve.printValues();
        backRightSwerve.printValues();
        frontRightSwerve.printValues();

        shooterLimelightX = shooterTx.getDouble(0.0);
        shooterLimelightY = shooterTy.getDouble(0.0);
        shooterTidValue = shooterTID.getDouble(0.0);

        frontLimelightX = frontTx.getDouble(0.0);
        frontLimelightY = frontTy.getDouble(0.0);

        backNoteSensorHit = !backNoteSensor.get();
        frontNoteSensorHit = !frontNoteSensor.get();
        shooterNoteSensorHit = !shooterNoteSensor.get();

        SmartDashboard.putNumber("Auto Stage", autoStage);
        SmartDashboard.putNumber("Drive Encoder Average", getDriveEncoders());
        SmartDashboard.putNumber("Encoder Distance Inches", getDriveDistInches());

        SmartDashboard.putNumber("Gyro", getBoundedGyroAngle());

        SmartDashboard.putNumber("Left X", leftX);
        SmartDashboard.putNumber("Left Y", leftY);
        SmartDashboard.putNumber("Right X", rightX);
        SmartDashboard.putNumber("Secondary Left Y", secondaryLeftY);
        SmartDashboard.putNumber("Secondary Right Y", secondaryRightY);

        SmartDashboard.putNumber("Magnitude", magnitude);
        SmartDashboard.putNumber("Drive Angle", driveAngle);
        SmartDashboard.putNumber("Angle Diff", angleDiff);

        SmartDashboard.putNumber("Shooter Limelight X", shooterLimelightX);
        SmartDashboard.putNumber("Shooter Limelight Y", shooterLimelightY);

        SmartDashboard.putNumber("Front Limelight X", frontLimelightX);
        SmartDashboard.putNumber("Front Limelight Y", frontLimelightY);
        SmartDashboard.putNumber("Shooter TID", shooterTidValue);

        SmartDashboard.putNumber("Arm Target", armTarget);
        SmartDashboard.putBoolean("Arm Override", armOverride);
        SmartDashboard.putNumber("Arm Motor Speed", armMotorSpeed);
        SmartDashboard.putNumber("Arm Absolute Encoder", armAbsoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Arm Adjusted Absolute Encoder", getAdjustedAbsoluteEncoder());

        SmartDashboard.putNumber("Climber Encoder", climberEncoder.getPosition());
        SmartDashboard.putNumber("Lead Screw Encoder", leadScrewEncoder.getPosition());
        SmartDashboard.putNumber("Lead Screw Offset", leadScrewOffset);


        SmartDashboard.putNumber("Lead Screw Target Position", targetLeadScrewPosition);

        SmartDashboard.putString("Auto Position", autoChooser.getSelected());

        autoDelay = Double.parseDouble(delayChooser.getSelected());
        SmartDashboard.putNumber("Delay Timer", autoDelay);

        SmartDashboard.putBoolean("Back Note Sensor", backNoteSensorHit);
        SmartDashboard.putBoolean("Front Note Sensor", frontNoteSensorHit);
        SmartDashboard.putBoolean("Shooter Note Sensor", shooterNoteSensorHit);
        SmartDashboard.putBoolean("Upper Arm Limit", upperArmLimit.get());
        SmartDashboard.putBoolean("Lower Arm Limit", lowerArmLimit.get());

        SmartDashboard.putNumber("front left zero offset", zeroPositionOffset[0]);
        SmartDashboard.putNumber("back left zero offset", zeroPositionOffset[1]);
        SmartDashboard.putNumber("back right zero offset", zeroPositionOffset[2]);
        SmartDashboard.putNumber("front right zero offset", zeroPositionOffset[3]);

        SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
        SmartDashboard.putNumber("Shooter Power", secondaryDriver.getRightTriggerAxis() / 4);
        SmartDashboard.putNumber("Shooter Top Velocity", shooterTopEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Bottom Velocity", shooterBottomEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Target Speed", shooterTargetSpeed);

        SmartDashboard.putNumber("Shooter Speed Offset", shooterSpeedOffset);

        // SmartDashboard.putNumber("PDH 0", powerDistributionHub.getCurrent(0));
        // SmartDashboard.putNumber("PDH 1", powerDistributionHub.getCurrent(1));
        // SmartDashboard.putNumber("PDH 2", powerDistributionHub.getCurrent(2));
        // SmartDashboard.putNumber("PDH 3", powerDistributionHub.getCurrent(3));
        // SmartDashboard.putNumber("PDH 4", powerDistributionHub.getCurrent(4));
        // SmartDashboard.putNumber("PDH 5", powerDistributionHub.getCurrent(5));
        // SmartDashboard.putNumber("PDH 6", powerDistributionHub.getCurrent(6));
        // SmartDashboard.putNumber("PDH 7", powerDistributionHub.getCurrent(7));
        // SmartDashboard.putNumber("PDH 8", powerDistributionHub.getCurrent(8));
        // SmartDashboard.putNumber("PDH 9", powerDistributionHub.getCurrent(9));
        // SmartDashboard.putNumber("PDH 10", powerDistributionHub.getCurrent(10));
        // SmartDashboard.putNumber("PDH 11", powerDistributionHub.getCurrent(11));
        // SmartDashboard.putNumber("PDH 12", powerDistributionHub.getCurrent(12));
        // SmartDashboard.putNumber("PDH 13", powerDistributionHub.getCurrent(13));
        // SmartDashboard.putNumber("PDH 14", powerDistributionHub.getCurrent(14));
        // SmartDashboard.putNumber("PDH 15", powerDistributionHub.getCurrent(15));
        // SmartDashboard.putNumber("PDH 16", powerDistributionHub.getCurrent(16));
        // SmartDashboard.putNumber("PDH 17", powerDistributionHub.getCurrent(17));
        // SmartDashboard.putNumber("PDH 18", powerDistributionHub.getCurrent(18));
        // SmartDashboard.putNumber("PDH 19", powerDistributionHub.getCurrent(19));

        SmartDashboard.putString("Alliance Color", String.valueOf(DriverStation.getAlliance()));
    }

    @Override
    public void autonomousInit() {
        shooterTable.getEntry("pipeline").setNumber(0); // near shot
        gyro.reset();
        resetDriveEncoders();
        shooterAtTarget = false;
        leadScrewEncoder.setPosition(0);
        climberEncoder.setPosition(0);
        shooterSpeed = 0.6;
        aimingCounter = 0;
        startGyroAngle = getBoundedGyroAngle();
        magnitude = 0.0;
        driveAngle = 0.0;
        rightX = 0.0;
        autoTurnAngle = 0.0;

        autoStage = 1;
        startTime = timer.get();


        frontTable.getEntry("pipeline").setNumber(2);

        // if (autoChooser.getSelected() == "Six Piece") {
        //     frontTable.getEntry("pipeline").setNumber(2);
        // } else if (autoChooser.getSelected() == "Outside") {
        //     if (String.valueOf(DriverStation.getAlliance()).contains("Red")) { 
        //         frontTable.getEntry("pipeline").setNumber(0);
        //     } else {
        //         frontTable.getEntry("pipeline").setNumber(1);
        //     }
        // }

        autoDelay = Double.parseDouble(delayChooser.getSelected());
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        diffTime = timer.get() - startTime;
        shooterTargetSpeed = shooterSpeed * SHOOTER_SPEED_TO_VELOCITY;

        // if ((autoChooser.getSelected() == "Simple Left") ||
        //     (autoChooser.getSelected() == "Simple Right")) {

        //     switch (autoStage) {
        //         case 1:
        //             shooterSpeed = 0.6;
        //             targetLeadScrewPosition = 15;

        //             if (Math.abs(leadScrewEncoder.getPosition() - targetLeadScrewPosition) < 2) {
        //                 autoStage++;
        //             } 
        //             break;
        //         case 2:
        //             shooterSpeed = 0.5;
        //             targetLeadScrewPosition = 15;
        //             shootNote();
        //             break;
        //         case 3:
        //             shooterSpeed = 0.0;

        //             if (diffTime > autoDelay) {
        //                 autoDrive(0, 60, 0.1, -1, 0.0);
        //             }
                    
        //             break;
        //     }
        // } else 
        if (autoChooser.getSelected() == "Backup") {
            shooterSpeed = 0;
            if (diffTime > autoDelay) {
                autoDrive(0, 60, 0.5, -1, 0.0);
            }
        } else if (String.valueOf(DriverStation.getAlliance()).contains("Red")) {
            /*******************************
             * 
             *  RED ALLIANCE
             * 
             *******************************/

            // Six Piece
            if (autoChooser.getSelected() == "Six Piece") {
                switch (autoStage) {
                    case 1:
                        shooterSpeed = 0.6;
                        targetLeadScrewPosition = 15;
    
                        if ((Math.abs(leadScrewEncoder.getPosition() - targetLeadScrewPosition) < 4) ||
                            (diffTime > 1.0)) {

                            startTime = timer.get();
                            autoStage++;
                        } 
                        break;
                    case 2:
                        shooterSpeed = 0.6;
                        targetLeadScrewPosition = 15;
                        shootNote();
                        break;
                    case 3:
                        autoIntakeNote();
                        autoDrive(318, 60, 1.0, 336, 0.1);
                        targetLeadScrewPosition = 215;
                        shooterSpeed = 0.79;
                        break;
                    case 4:
                        autoIntakeNote();
                        targetLeadScrewPosition = 215;
                        shooterSpeed = 0.79;
                        shootNote();
                        break;
                    case 5:
                        autoIntakeNote();
                        if (getDriveDistInches() < 24) {
                            autoTurnAngle = 90;
                        } else {
                            autoTurnAngle = 0;
                        }
                        targetLeadScrewPosition = 180;
                        autoDrive(88, 37, 0.9, autoTurnAngle, 0);
                        break;
                    case 6:
                        turnToAngle(0.0, 0.8);
                        autoIntakeNote();
                        shooterSpeed = 0.77;
                        targetLeadScrewPosition = 180;
                        autoStage++;
                        break;
                    case 7:
                        turnToAngle(0.0, 0.8);
                        autoIntakeNote();
                        shooterSpeed = 0.77;
                        targetLeadScrewPosition = 180;
                        shootNote();
                        break;
                    case 8:
                        autoIntakeNote();
                        if (getDriveDistInches() < 22) {
                            autoTurnAngle = 90;
                        } else {
                            autoTurnAngle = 24;
                        }
                        targetLeadScrewPosition = 216;
                        autoDrive(87, 48, 0.9, autoTurnAngle, 0);
                        break;
                    case 9:
                        turnToAngle(24, 0.8);
                        autoIntakeNote();
                        shooterSpeed = 0.79;
                        targetLeadScrewPosition = 216;
                        autoStage++;
                        break;
                    case 10:
                        turnToAngle(24, 0.8);
                        autoIntakeNote();
                        shooterSpeed = 0.79;
                        targetLeadScrewPosition = 216;
                        shootNote();
                        break;
                    case 11:
                        autoIntakeNote();
                        shooterSpeed = 0.87;
                        targetLeadScrewPosition = 335;
                        autoDrive(7, 207, 1.0, 6, 0.0);
                        break;
                    case 12:
                        autoIntakeNote();
                        shooterSpeed = 0.87;
                        targetLeadScrewPosition = 335;
                        autoDrive(190, 150, 1.0, 18, 0.0);
                        break;
                    case 13:
                        autoIntakeNote();
                        shooterSpeed = 0.87;
                        targetLeadScrewPosition = 335;
                        turnToAngle(18, 0.5);
                        shootNote();
                        break;
                    case 14:
                        autoIntakeNote();
                        shooterSpeed = 0.91;
                        targetLeadScrewPosition = 330;
                        if (getDriveDistInches() < 90) {
                            autoDriveDirection = 346;
                        } else {
                            autoDriveDirection = 346 + (frontLimelightX * 0.5);
                        }
                        autoDrive(autoDriveDirection, 150, 1.0, 350, 0.0);
                        break;
                    case 15:
                        autoIntakeNote();
                        shooterSpeed = 0.91;
                        targetLeadScrewPosition = 330;
                        autoDrive(167, 165, 1.0, 15, 0.0);
                        break;
                    case 16:
                        autoIntakeNote();
                        shooterSpeed = 0.91;
                        targetLeadScrewPosition = 330;
                        turnToAngle(15, 0.5);
    
                        if (diffTime > 0.1 ||
                            (Math.abs(15 - getBoundedGyroAngle()) < 2)) {
                            autoStage++;
                            startTime = timer.get();
                        }
                        break;
                    case 17:
                        autoIntakeNote();
                        shooterSpeed = 0.91;
                        targetLeadScrewPosition = 330;
                        turnToAngle(15, 0.5);
                        shootNote();
                        break;
                } 

            // Outside
            } else if (autoChooser.getSelected() == "Outside") {
                switch (autoStage) {
                    case 1:
                        shooterSpeed = 0.75;
                        targetLeadScrewPosition = 135;
                        autoTurn(304, 0.5);
                        break;
                    case 2:
                        shooterSpeed = 0.75;
                        targetLeadScrewPosition = 135;
                        if ((Math.abs(leadScrewEncoder.getPosition() - targetLeadScrewPosition) < 5) ||
                            (diffTime > 1.5)) {
                                
                            shootNote();
                        }
                        break;
                    case 3:
                        autoIntakeNote();
                        shooterSpeed = 0.83;
                        targetLeadScrewPosition = 295;

                        if (Math.abs(frontLeftSwerve.getDriveEncoder()) < 115 ||
                            frontNoteSensorHit) {

                            autoDriveDirection = 337;
                        } else {
                            autoDriveDirection = 337 + (frontLimelightX * 0.5);
                        }

                        autoDrive(autoDriveDirection, 280, 1.0, 339, 0);
                        break;
                    case 4:
                        autoIntakeNote();
                        shooterSpeed = 0.83;
                        targetLeadScrewPosition = 310;
                        autoDrive(160, 220, 1.0, 314, 0);
                        break;
                    case 5:
                        turnToAngle(314, 0.5);
                        shooterSpeed = 0.83;
                        targetLeadScrewPosition = 310;
                        shootNote();
                        break;
                    case 6:
                    
                        autoIntakeNote();

                        if (getDriveDistInches() < 90) {
                            autoDriveDirection = 336;
                            autoTurnAngle = 336;
                        } else if (getDriveDistInches() < 140 ||
                                   frontNoteSensorHit) {
                            autoDriveDirection = 13;
                            autoTurnAngle = 13;
                        } else {
                            autoDriveDirection = 13 + (frontLimelightX * 0.5);
                            autoTurnAngle = 13;
                        }
                        targetLeadScrewPosition = 305;
                        autoDrive(autoDriveDirection, 200, 1.0, autoTurnAngle, 0);
                        
                        break;
                    case 7:
                        autoIntakeNote();
                        shooterSpeed = 0.9;
                        targetLeadScrewPosition = 305;
                        autoDrive(194, 90, 1.0, 13, 0);
                        break;
                    case 8:
                        autoIntakeNote();
                        shooterSpeed = 0.9;
                        targetLeadScrewPosition = 300;
                        autoDrive(156, 118, 1.0, 320, 0);
                        break;
                    case 9:
                        turnToAngle(320, 0.5);
                        shooterSpeed = 0.9;
                        targetLeadScrewPosition = 305;
                        shootNote();
                        break;
                    case 10:
                        autoIntakeNote();
                        shooterSpeed = 0.83;
                        targetLeadScrewPosition = 299;

                        if (Math.abs(frontLeftSwerve.getDriveEncoder()) < 80 ||
                            frontNoteSensorHit) {

                            autoDriveDirection = 10;
                        } else {
                            autoDriveDirection = 10 + (frontLimelightX * 0.5);
                        }

                        autoDrive(autoDriveDirection, 202, 1.0, 10, 0);
                        break;
                    case 11:
                        autoIntakeNote();
                        shooterSpeed = 0.84;
                        targetLeadScrewPosition = 299;
                        autoDrive(156, 155, 1.0, 351, 0);
                        break;
                    case 12:
                        turnToAngle(351, 0.5);
                        shooterSpeed = 0.84;
                        targetLeadScrewPosition = 299;
                        shootNote();
                        break;

                }
            }
        } else {
            /*******************************
             * 
             *  BLUE ALLIANCE
             * 
             *******************************/

            // Six Piece
            if (autoChooser.getSelected() == "Six Piece") {
                switch (autoStage) {
                    case 1:
                        shooterSpeed = 0.6;
                        targetLeadScrewPosition = 15;
    
                        if ((Math.abs(leadScrewEncoder.getPosition() - targetLeadScrewPosition) < 4) ||
                            (diffTime > 1.0)) {
                        // if (diffTime > 3.0) {
                                
                            startTime = timer.get();
                            autoStage++;
                        } 
                        break;
                    case 2:
                        shooterSpeed = 0.6;
                        targetLeadScrewPosition = 15;
                        shootNote();
                        break;
                    case 3:
                        autoIntakeNote();
                        autoDrive(42, 60, 1.0, 24, 0.1);
                        targetLeadScrewPosition = 215;
                        shooterSpeed = 0.79;
                        break;
                    case 4:
                        autoIntakeNote();
                        targetLeadScrewPosition = 215;
                        shooterSpeed = 0.79;
                        shootNote();
                        break;
                    case 5:
                        autoIntakeNote();
                        if (getDriveDistInches() < 26) {
                            autoTurnAngle = 270;
                        } else {
                            autoTurnAngle = 0;
                        }
                        targetLeadScrewPosition = 180;
                        autoDrive(273, 37, 0.9, autoTurnAngle, 0);
                        break;
                    case 6:
                        turnToAngle(0.0, 0.8);
                        autoIntakeNote();
                        shooterSpeed = 0.77;
                        targetLeadScrewPosition = 180;
                        autoStage++;
                        break;
                    case 7:
                        turnToAngle(0.0, 0.8);
                        autoIntakeNote();
                        shooterSpeed = 0.77;
                        targetLeadScrewPosition = 180;
                        shootNote();
                        break;
                    case 8:
                        autoIntakeNote();
                        if (getDriveDistInches() < 22) {
                            autoTurnAngle = 270;
                        } else {
                            autoTurnAngle = 336;
                        }
                        targetLeadScrewPosition = 216;
                        autoDrive(275, 34, 0.9, autoTurnAngle, 0);
                        break;
                    case 9:
                        turnToAngle(336, 0.8);
                        autoIntakeNote();
                        shooterSpeed = 0.79;
                        targetLeadScrewPosition = 216;
                        autoStage++;
                        break;
                    case 10:
                        turnToAngle(336, 0.8);
                        autoIntakeNote();
                        shooterSpeed = 0.79;
                        targetLeadScrewPosition = 216;
                        shootNote();
                        break;
                    case 11:
                        autoIntakeNote();
                        shooterSpeed = 0.79;
                        targetLeadScrewPosition = 216;
                        autoDrive(353, 210, 1.0, 354, 0.0);
                        break;
                    case 12:
                        autoIntakeNote();
                        shooterSpeed = 0.87;
                        targetLeadScrewPosition = 322;
                        autoDrive(170, 153, 1.0, 342, 0.0);
                        break;
                    case 13:
                        autoIntakeNote();
                        shooterSpeed = 0.87;
                        targetLeadScrewPosition = 322;
                        turnToAngle(342, 0.5);
                        shootNote();
                        break;
                    case 14:
                        autoIntakeNote();
                        shooterSpeed = 0.91;
                        targetLeadScrewPosition = 315;
                        if (getDriveDistInches() < 90) {
                            autoDriveDirection = 12;
                        } else {
                            autoDriveDirection = 12 + (frontLimelightX * 0.5);
                        }
                        autoDrive(autoDriveDirection, 150, 1.0, 10, 0.0);
                        break;
                    case 15:
                        autoIntakeNote();
                        shooterSpeed = 0.91;
                        targetLeadScrewPosition = 315;
                        autoDrive(193, 165, 1.0, 345, 0.0);
                        break;
                    case 16:
                        autoIntakeNote();
                        shooterSpeed = 0.91;
                        targetLeadScrewPosition = 315;
                        turnToAngle(345, 0.5);
    
                        if (diffTime > 0.1 ||
                            (Math.abs(15 - getBoundedGyroAngle()) < 2)) {

                            autoStage++;
                            startTime = timer.get();
                        }
                        break;
                    case 17:
                        autoIntakeNote();
                        shooterSpeed = 0.91;
                        targetLeadScrewPosition = 315;
                        turnToAngle(345, 0.5);
                        shootNote();
                        break;
                }
            // Outside
            } else if (autoChooser.getSelected() == "Outside") {
                switch (autoStage) {
                    case 1:
                        shooterSpeed = 0.75;
                        targetLeadScrewPosition = 135;
                        autoTurn(56, 0.5);
                        break;
                    case 2:
                        shooterSpeed = 0.75;
                        targetLeadScrewPosition = 135;
                        if ((Math.abs(leadScrewEncoder.getPosition() - targetLeadScrewPosition) < 5) ||
                            (diffTime > 1.5)) {
                                
                            shootNote();
                        }
                        break;
                    case 3:
                        autoIntakeNote();
                        shooterSpeed = 0.83;
                        targetLeadScrewPosition = 295;

                        if (Math.abs(frontLeftSwerve.getDriveEncoder()) < 115 ||
                            frontNoteSensorHit) {

                            autoDriveDirection = 22;
                        } else {
                            autoDriveDirection = 22 + (frontLimelightX * 0.5);
                        }

                        autoDrive(autoDriveDirection, 280, 1.0, 21, 0);
                        break;
                    case 4:
                        autoIntakeNote();
                        shooterSpeed = 0.83;
                        targetLeadScrewPosition = 305;
                        autoDrive(200, 220, 1.0, 46, 0);
                        break;
                    case 5:
                        turnToAngle(46, 0.5);
                        shooterSpeed = 0.83;
                        targetLeadScrewPosition = 305;
                        shootNote();
                        break;
                    case 6:
                    
                        autoIntakeNote();

                        if (getDriveDistInches() < 90) {
                            autoDriveDirection = 24;
                            autoTurnAngle = 24;
                        } else if (getDriveDistInches() < 140 ||
                                   frontNoteSensorHit) {
                            autoDriveDirection = 347;
                            autoTurnAngle = 347;
                        } else {
                            autoDriveDirection = 347 + (frontLimelightX * 0.5);
                            autoTurnAngle = 347;
                        }
                        targetLeadScrewPosition = 305;
      
                        autoDrive(autoDriveDirection, 205, 1.0, autoTurnAngle, 0);
                        
                        break;
                    case 7:
                        autoIntakeNote();
                        shooterSpeed = 0.9;
                        targetLeadScrewPosition = 305;
                        autoDrive(167, 90, 1.0, 347, 0);
                        
                        break;
                    case 8:
                        autoIntakeNote();
                        shooterSpeed = 0.9;
                        targetLeadScrewPosition = 305;
                        autoDrive(204, 118, 1.0, 40, 0);
                        break;
                    case 9:
                        turnToAngle(40, 0.5);
                        shooterSpeed = 0.9;
                        targetLeadScrewPosition = 305;
                        shootNote();
                        break;
                    case 10:
                        autoIntakeNote();
                        shooterSpeed = 0.83;
                        targetLeadScrewPosition = 299;

                        if (Math.abs(frontLeftSwerve.getDriveEncoder()) < 80 ||
                            frontNoteSensorHit) {

                            autoDriveDirection = 350;
                        } else {
                            autoDriveDirection = 350 + (frontLimelightX * 0.5);
                        }

                        autoDrive(autoDriveDirection, 190, 1.0, 350, 0);
                        break;
                    case 11:
                        autoIntakeNote();
                        shooterSpeed = 0.84;
                        targetLeadScrewPosition = 299;
                        autoDrive(194, 155, 1.0, 9, 0);
                        break;
                    case 12:
                        turnToAngle(9, 0.5);
                        shooterSpeed = 0.84;
                        targetLeadScrewPosition = 299;
                        shootNote();
                        break;

                }
            }
        }

       

        leadScrewPID.setReference(targetLeadScrewPosition, ControlType.kPosition);

        if (shooterSpeed != 0) {
            shooterMotorTop.set(shooterSpeed);
            shooterMotorBottom.set(shooterSpeed - SHOOTER_SPEED_OFFSET);
        } else {
            shooterMotorTop.set(0);
            shooterMotorBottom.set(0);
        }

        armMotor.set(0.02);

        // Drive each swerve module
        frontLeftSwerve.drive(magnitude * FRONT_LEFT_SCALE, driveAngle, rightX, false, false);
        backLeftSwerve.drive(magnitude * BACK_LEFT_SCALE, driveAngle, rightX, false, false);
        backRightSwerve.drive(magnitude * BACK_RIGHT_SCALE, driveAngle, rightX, false, false);
        frontRightSwerve.drive(magnitude * FRONT_RIGHT_SCALE, driveAngle, rightX, false, false);
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        magnitude = 0;
        driveAngle = 0;
        armOverride = true;
        armMotorSpeed = 0;
        armTarget = ARM_STAGING_POSITION;
        gamePieceStaged = false;
        shooterAtTarget = false;
        gamePieceStagedTimer = -1;
        timer.reset();

        frontTable.getEntry("pipeline").setNumber(2);
        shooterTable.getEntry("pipeline").setNumber(0); // near shot

        resetEncoderCounter = 0;
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        // // Apply Offset to Abolute Encoder Zero for testing purposes
        // if (primaryDriver.getLeftBumper()) {
        //     if (primaryDriver.getPOV() == 0) {
        //         zeroPositionOffset[0] += 0.0001;
        //     } else if (primaryDriver.getPOV() == 90) {
        //         zeroPositionOffset[1] += 0.0001;
        //     } else if (primaryDriver.getPOV() == 180) {
        //         zeroPositionOffset[0] -= 0.0001;
        //     } else if (primaryDriver.getPOV() == 270) {
        //         zeroPositionOffset[1] -= 0.0001;
        //     }
        // } else if (primaryDriver.getRightBumper()) {
        //     if (primaryDriver.getPOV() == 0) {
        //         zeroPositionOffset[2] += 0.0001;
        //     } else if (primaryDriver.getPOV() == 90) {
        //         zeroPositionOffset[3] += 0.0001;
        //     } else if (primaryDriver.getPOV() == 180) {
        //         zeroPositionOffset[2] -= 0.0001;
        //     } else if (primaryDriver.getPOV() == 270) {
        //         zeroPositionOffset[3] -= 0.0001;
        //     }
        // }

/**********************************************
 * 
 * PRIMARY DRIVER
 * 
//  **********************************************/

        if (primaryDriver.getPOV() == 0) { // up on the dpad
            leftX = 0;
            leftY = -SLOW_SPEED;
        } else if (primaryDriver.getPOV() == 90) { // right
            leftX = SLOW_SPEED;
            leftY = 0;
        } else if (primaryDriver.getPOV() == 180) { // down
            leftX = 0;
            leftY = SLOW_SPEED;
        } else if (primaryDriver.getPOV() == 270) { // left
            leftX = -SLOW_SPEED;
            leftY = 0;
        } else {
            leftX = primaryDriver.getLeftX();
            leftY = primaryDriver.getLeftY();
        }

        if (primaryDriver.getRightTriggerAxis() > DEAD_ZONE) { // align to the speaker
            if (shooterTidValue == -1) {
                leftX += (frontLimelightX / 50);
                if (Math.abs(primaryDriver.getRightX()) > DEAD_ZONE) { 
                    rightX = primaryDriver.getRightX();
                } else {
                    rightX = 0;
                }
            } else {
                // calcLimelightOffsetX();
                // rightX = (shooterLimelightX + limelightOffsetX) * SHOOTER_LIMELIGHT_X_SCALE;
                rightX = shooterLimelightX * SHOOTER_LIMELIGHT_X_SCALE;

                if (Math.abs(rightX) > 0.025) {
                    if (ledLightCounter == 0) {
                        ledLightCounter = 10;
                    } else {
                        ledLightCounter--;
                    }

                    if (ledLightCounter > 5) {
                        powerDistributionHub.setSwitchableChannel(true);
                    } else {
                        powerDistributionHub.setSwitchableChannel(false);
                    }
                } else {
                    powerDistributionHub.setSwitchableChannel(true);
                }
            }

            
        // } else if (primaryDriver.getLeftTriggerAxis() > DEAD_ZONE) {
        //     if (String.valueOf(DriverStation.getAlliance()).contains("Red")) {
        //         calcRightX(298);
        //     } else {
        //         calcRightX(62);
        //     }
        } else if (primaryDriver.getLeftBumper()) { // face left stage
            calcRightX(60);
        } else if (primaryDriver.getRightBumper()) { // face right stage
            calcRightX(300);
        } else if (primaryDriver.getYButton()) { // face away from the driver station
            calcRightX(0);
        } else if (primaryDriver.getBButton()) { // face left
            calcRightX(90);
        } else if (primaryDriver.getAButton()) { // face towards the driver station / center stage
            calcRightX(180);
        } else if (primaryDriver.getXButton()) { // face right
            calcRightX(270);
        } else if (primaryDriver.getLeftStickButton()) { // face right
            if (String.valueOf(DriverStation.getAlliance()).contains("Red")) { 
                calcRightX(330);
            } else {
                calcRightX(30);
            }
        } else {
            if (Math.abs(primaryDriver.getRightX()) > DEAD_ZONE) {
                if (primaryDriver.getRightX() < 0) {
                    // rightX = -Math.pow(primaryDriver.getRightX(), 2.0);
                    rightX = primaryDriver.getRightX();
                } else {
                    // rightX = Math.pow(primaryDriver.getRightX(), 2.0);
                    rightX = primaryDriver.getRightX();
                }
            } else {
                rightX = 0;
            }

            ledLightCounter = 0;
            powerDistributionHub.setSwitchableChannel(false);
        }

        magnitude = MathUtil.clamp(Math.hypot(leftX, leftY), 0, 1);

        if (magnitude < DEAD_ZONE) {
            magnitude = 0;
        } else {
            magnitude = Math.pow((magnitude - DEAD_ZONE) / (1 - DEAD_ZONE), 2.0);
        }

        if (primaryDriver.getLeftTriggerAxis() > DEAD_ZONE) {
            magnitude *= 0.5;
            rightX *= 0.5;
        }

        // Get the direction of the drive angle based on the X and Y input
        if (magnitude != 0) {
            driveAngle = 180 + (Math.atan2(-leftX, leftY) * (180 / Math.PI));

            // If not currently in auto align mode, apply a gyro offset to the drive angle
            // for field oriented control
            // if (((primaryDriver.getRightTriggerAxis() < DEAD_ZONE) ||
            if (primaryDriver.getPOV() == -1) {
                driveAngle -= getBoundedGyroAngle();

                // Keep angle between 0 and 360
                if (driveAngle < 0) {
                    driveAngle += 360;
                }
            }
        }

        if (primaryDriver.getRightStickButton()) {
            useMaxSpin = true;
        } else {
            useMaxSpin = false;
        }

        frontLeftSwerve.drive(magnitude * FRONT_LEFT_SCALE, driveAngle, rightX, false, useMaxSpin);
        backLeftSwerve.drive(magnitude * BACK_LEFT_SCALE, driveAngle, rightX, false, useMaxSpin);
        backRightSwerve.drive(magnitude * BACK_RIGHT_SCALE, driveAngle, rightX, false, useMaxSpin);
        frontRightSwerve.drive(magnitude * FRONT_RIGHT_SCALE, driveAngle, rightX, false, useMaxSpin);

        // Reset the encoders periodically when not moving
        if (magnitude == 0 &&
            rightX == 0 &&
            resetEncoderCounter > 50) {

            frontLeftSwerve.resetEncoders(zeroPositionOffset);
            backLeftSwerve.resetEncoders(zeroPositionOffset);
            backRightSwerve.resetEncoders(zeroPositionOffset);
            frontRightSwerve.resetEncoders(zeroPositionOffset);

            resetEncoderCounter = 0;
        }

        resetEncoderCounter++;

        // Manually reset the gyro and encoders
        if (primaryDriver.getBackButton() &&
            primaryDriver.getStartButton()) {

            gyro.reset();
            resetDriveEncoders();
        }

/*********************************************
 * 
 * SECONDARY DRIVER
 * 
 *********************************************/

        if (Math.abs(secondaryDriver.getLeftY()) > DEAD_ZONE) {
            secondaryLeftY = -secondaryDriver.getLeftY();
        } else {
            secondaryLeftY = 0;
        }

        if (Math.abs(secondaryDriver.getRightY()) > DEAD_ZONE) {
            secondaryRightY = -secondaryDriver.getRightY();
        } else {
            secondaryRightY = 0;
        }

        // Arm Control
        if (secondaryDriver.getRightTriggerAxis() < DEAD_ZONE) {
            if (secondaryDriver.getPOV() == 180) {

                armTarget = ARM_STAGING_POSITION;
                armOverride = false;
            } else if ((secondaryDriver.getPOV() == 90) &&
                        frontNoteSensorHit &&
                        backNoteSensorHit) {

                armTarget = ARM_AMP_POSITION;
                armOverride = false;
            } else if (secondaryDriver.getPOV() == 0) {
                
                armTarget = ARM_MAX_HEIGHT;
                armOverride = false;
            } else if (secondaryDriver.getPOV() == 270) {

                armTarget = ARM_TRAP_POSITION;
                armOverride = false;
            } else if (armOverride == false &&
                    secondaryDriver.getLeftTriggerAxis() < DEAD_ZONE) {
            } else {
                // if ((secondaryRightY <= 0) &&
                //     lowerArmLimit.get()) {

                //     armMotorSpeed = 0.01;
                // } else 
                if ((secondaryRightY > 0) &&
                            upperArmLimit.get()) {

                    armMotorSpeed = -0.001;
                } else if ((secondaryRightY == 0) &&
                            //(lowerArmLimit.get() == false) &&
                            (upperArmLimit.get() == false)) {

                    armMotorSpeed = -0.01;
                } else if (secondaryDriver.getLeftTriggerAxis() > DEAD_ZONE) {
                    armMotorSpeed = -secondaryRightY;
                    armOverride = true;
                }
            }
        }

        // if (armOverride == false &&
        //     lowerArmLimit.get() && 
        //     armTarget == ARM_STAGING_POSITION &&
        //     secondaryDriver.getLeftTriggerAxis() < DEAD_ZONE) {

        //     armMotorSpeed = 0.02;
        if (armOverride == false) {
            armMotorSpeed = MathUtil.clamp(armPID.calculate(getAdjustedAbsoluteEncoder(), armTarget), ARM_MOTOR_UP_CAP, ARM_MOTOR_DOWN_CAP);

            if (armMotorSpeed > 0 &&
                getAdjustedAbsoluteEncoder() < ARM_TRAP_POSITION) {

                armMotorSpeed *= 0.5;
            }

            if (armMotorSpeed < 0.02 &&
                armTarget == ARM_STAGING_POSITION &&
                secondaryDriver.getLeftTriggerAxis() < DEAD_ZONE) {

                armMotorSpeed = 0.02;
            }
        } 

        armMotor.set(armMotorSpeed);
        


        // if (primaryDriver.getPOV() == 0) {
        //     shooterSpeedOffset += 0.001;
        // } else if (primaryDriver.getPOV() == 180) {
        //     shooterSpeedOffset -= 0.001;
        // }

        // 15 clicks off zero is subwoofer

        if (secondaryDriver.getLeftTriggerAxis() > DEAD_ZONE && 
            secondaryDriver.getRightTriggerAxis() > DEAD_ZONE) {

            shooterSpeed = 0.6;
            targetLeadScrewPosition = 15;

            // leadScrewSpeed = (targetLeadScrewPosition - leadScrewEncoder.getPosition()) / 20;
            leadScrewPID.setReference(targetLeadScrewPosition, ControlType.kPosition);
        } else if (secondaryDriver.getRightTriggerAxis() > DEAD_ZONE) { // spin up the shooter
            
            calcShooterSpeed();
            calcLeadScrewPosition();

            if (secondaryDriver.getPOV() == 0) {
                if (leadScrewOffsetPressed == false) {
                    leadScrewOffset += 5;
                    leadScrewOffsetPressed = true;
                } 
            } else if (secondaryDriver.getPOV() == 180) {
                if (leadScrewOffsetPressed == false) {
                    leadScrewOffset -= 5;
                    leadScrewOffsetPressed = true;
                }
            } else if (secondaryDriver.getPOV() == 90) {
                shooterTable.getEntry("pipeline").setNumber(1); // far shot
            } else if (secondaryDriver.getPOV() == 270) {
                shooterTable.getEntry("pipeline").setNumber(0); // near shot
            } else {
                leadScrewOffsetPressed = false;
            }

            targetLeadScrewPosition -= leadScrewOffset;

            // leadScrewSpeed = (targetLeadScrewPosition - leadScrewEncoder.getPosition()) / 20;
            leadScrewPID.setReference(targetLeadScrewPosition, ControlType.kPosition);
        } else if ((secondaryRightY != 0) &&
                   (secondaryDriver.getLeftTriggerAxis() < DEAD_ZONE)) {

            leadScrewSpeed = -secondaryRightY;
            if ((leadScrewEncoder.getPosition() < 50) &&
                 leadScrewSpeed < 0) {

                leadScrewSpeed *= 0.25;
            }
            leadScrewMotor.set(leadScrewSpeed);
        } else {
            shooterSpeed = 0;
            // leadScrewSpeed = 0;
            leadScrewMotor.set(0);
        }

        // intake with A, reverse it with X, as long as the arm is down
        if (secondaryDriver.getAButton()) {
                // && Math.abs(armPositionEncoder.getAbsolutePosition() - ARM_STAGING_POSITION) < ARM_TOLERANCE) {
            if (secondaryDriver.getXButton()) {
                intakeMotor.set(INTAKE_MOTOR_SPEED);
            } else {
                intakeNote();
            }
        } else if ((secondaryDriver.getRightTriggerAxis() > DEAD_ZONE) && 
                   (secondaryDriver.getBButton() == false)) {
            if (backNoteSensorHit && 
                frontNoteSensorHit) {
                    
                stagingMotor.set(-STAGING_MOTOR_SPEED);
            } else {
                stagingMotor.set(0);
            }
        } else if (secondaryDriver.getBButton()) {
            stagingMotor.set(-STAGING_MOTOR_SPEED);
        } else if (secondaryDriver.getXButton()) {
            shooterSpeed = -0.2;
            stagingMotor.set(STAGING_MOTOR_SPEED);
        } else {
            intakeMotor.set(0);
            stagingMotor.set(0);

            gamePieceStaged = false;
        }

        if (timer.get() - gamePieceStagedTimer < 1) {
            primaryDriver.setRumble(RumbleType.kBothRumble, 1.0);
            secondaryDriver.setRumble(RumbleType.kBothRumble, 1.0);
        } else {
            primaryDriver.setRumble(RumbleType.kBothRumble, 0.0);
            secondaryDriver.setRumble(RumbleType.kBothRumble, 0.0);
        }

        if (shooterSpeed != 0) {
            shooterMotorTop.set(shooterSpeed);
            shooterMotorBottom.set(shooterSpeed - SHOOTER_SPEED_OFFSET);
        } else {
            shooterMotorTop.set(0);
            shooterMotorBottom.set(0);
        }

        // feed the shooter with B, score in amp/trap with X
        

        // Climber
        if (secondaryDriver.getLeftBumper() &&
            secondaryDriver.getRightBumper()) {

            if (climberEncoder.getPosition() < 1) {
                climberMotor.set(-0.2);
            } else if (climberEncoder.getPosition() <= 5) {
                climberMotor.set(-0.5);
            } else {
                climberMotor.set(-1.0);
            }
        } else if (secondaryDriver.getRightBumper() &&
           (climberEncoder.getPosition() < CLIMBER_CENTER_CLIMB_HEIGHT)) {

            climberMotor.set(1.0);
        } else if (secondaryDriver.getLeftBumper() &&
                   climberEncoder.getPosition() > CLIMBER_HANG_HEIGHT) {
            climberMotor.set(-1.0);
        } else if (secondaryLeftY != 0) { // manual climber control, if not controlling the arm
            if (secondaryLeftY < 0 &&
                climberEncoder.getPosition() <= 0) {

                climberMotor.set(secondaryLeftY / 5);
            } else if (climberEncoder.getPosition() <= 5 && secondaryLeftY < 0) {
                climberMotor.set(secondaryLeftY * 0.5);
            } else {
                climberMotor.set(secondaryLeftY);
            }            
        } else {
            climberMotor.set(0);
        }

        // Manually reset the climer and lead screwencoders
        if (secondaryDriver.getBackButton() &&
            secondaryDriver.getStartButton()) {

            leadScrewEncoder.setPosition(0);
            climberEncoder.setPosition(0);
        }
    }

    public void autoIntakeNote() {
        if (!noteStaged) {
            if (backNoteSensorHit && frontNoteSensorHit) {
                stagingMotor.set(-STAGING_MOTOR_SPEED * 0.25);
                intakeMotor.set(0);
            } else if (backNoteSensorHit && !frontNoteSensorHit) {
                noteStaged = true; // tell shootNote it has control of the staging motor
                stagingMotor.set(0);
                // stop the intake motor when a note is detected
                intakeMotor.set(0);
            } else if (frontNoteSensorHit) {
                stagingMotor.set(-STAGING_MOTOR_SPEED * 0.9);
            } else {
                intakeMotor.set(-INTAKE_MOTOR_SPEED);
                stagingMotor.set(-STAGING_MOTOR_SPEED);
            }
        }
        
    }

    public void intakeNote() {
        if (gamePieceStaged &&
            !backNoteSensorHit && 
            !frontNoteSensorHit) {

            intakeMotor.set(0);
            stagingMotor.set(STAGING_MOTOR_ALIGNMENT_SPEED);
        } else if (backNoteSensorHit && frontNoteSensorHit) {
            stagingMotor.set(0); // stop the staging motor when a note is detected
            intakeMotor.set(0);
            gamePieceStaged = true;
        } else if (frontNoteSensorHit) {
            intakeMotor.set(-INTAKE_MOTOR_SPEED);
            stagingMotor.set(-STAGING_MOTOR_ALIGNMENT_SPEED);
            gamePieceStagedTimer = timer.get();
        } else if (backNoteSensorHit) {
            intakeMotor.set(0);
            stagingMotor.set(STAGING_MOTOR_ALIGNMENT_SPEED);
        } else {
            intakeMotor.set(-INTAKE_MOTOR_SPEED);
            stagingMotor.set(-STAGING_MOTOR_SPEED);
        }
        
    }

    public void shootNote() {
        // if (((Math.abs(shooterLimelightX) > 4) ||
        //     (Math.abs(leadScrewEncoder.getPosition() - targetLeadScrewPosition) > 4)) &&
        //     (aimingCounter < 75) &&
        //     (shooterAtTarget == false)) {

        //     startTime = timer.get();
        //     aimingCounter++;
        // } else if ((diffTime < 1.2) &&
        //            !shooterNoteSensorHit) {
        if ((diffTime < 1.0) &&
                !shooterNoteSensorHit) {
            stagingMotor.set(-STAGING_MOTOR_SPEED);
            intakeMotor.set(-INTAKE_MOTOR_SPEED);
            shooterAtTarget = true;
        } else {
            stagingMotor.set(0);
            intakeMotor.set(0);
            gamePieceStaged = false;
            shooterAtTarget = false;
            noteStaged = false;
            aimingCounter = 0;
            resetDriveEncoders();
            autoStage++;
            startTime = timer.get();
            startGyroAngle = getBoundedGyroAngle();
        }
    }

    // // vertical and horizontal distance are relative to a top-down view of the robot at the start of it's turn
    //  // not necessary WIP, Clayton can ignore
    // public void autoArc(double verticalDistance, double horizontalDistance, double maxSpeed, double endAngle) {
    //     angleDiff = endAngle - getBoundedGyroAngle();
    //     while (angleDiff < 0) angleDiff += 360;

    //     frontLeftArcLength = calcArcLength(verticalDistance - SwerveModule.Y_POSITIONS[0] * Math.sin(Math.toRadians(angleDiff)) - 
    //                                                           SwerveModule.X_POSITIONS[0] * Math.sin(Math.toRadians(angleDiff)),
    //                                         horizontalDistance + SwerveModule.Y_POSITIONS[0] * Math.sin(Math.toRadians(angleDiff)) - 
    //                                                              SwerveModule.X_POSITIONS[0] * Math.sin(Math.toRadians(angleDiff)), endAngle);
    //     frontRightArcLength = calcArcLength(verticalDistance - SwerveModule.Y_POSITIONS[3] * Math.sin(Math.toRadians(angleDiff)) - 
    //                                                            SwerveModule.X_POSITIONS[3] * Math.sin(Math.toRadians(angleDiff)),
    //                                         horizontalDistance + SwerveModule.Y_POSITIONS[3] * Math.sin(Math.toRadians(angleDiff)) - 
    //                                                              SwerveModule.X_POSITIONS[3] * Math.sin(Math.toRadians(angleDiff)), endAngle);

    //     frontLeftDistDiff = frontLeftArcLength - frontLeftSwerve.getDriveEncoder() * ENCODER_DISTANCE_SCALE;
    //     frontRightDistDiff = frontRightArcLength - frontRightSwerve.getDriveEncoder() * ENCODER_DISTANCE_SCALE;

    //     speedScale = horizontalDistance > 0 ? frontRightDistDiff / frontLeftDistDiff : frontLeftDistDiff / frontRightDistDiff;

    //     double fociY = verticalDistance > horizontalDistance ? Math.pow(verticalDistance, 2) - Math.pow(horizontalDistance, 2) :
    //                                                            Math.pow(horizontalDistance, 2) - Math.pow(verticalDistance, 2);

    //     double driveAngle = Math.atan(fociY / calcArcLength(verticalDistance, horizontalDistance, endAngle));
        
    //     if (Math.abs(angleDiff) < AUTO_ARC_ANGLE_TOLERANCE) {
    //         autoStage++;
    //     }
    // }

    public void calcShooterSpeed() {
        if (shooterTidValue == -1) {
                if (primaryDriver.getLeftStickButton()) {
                    shooterSpeed = 0.65;
                } else {
                    shooterSpeed = 0.75;
                }
                
        } else {
            if (shooterLimelightY < 1) {
                shooterSpeed = 0.90 ;
            } else {
                shooterSpeed = 0.85 - (shooterLimelightY / 100.0);
            }
        }
    }

    public void calcLeadScrewPosition() {
        if (shooterTidValue == -1) {
            targetLeadScrewPosition = 160.0;
        } else {
            if (String.valueOf(DriverStation.getAlliance()).contains("Red")) { 
                targetLeadScrewPosition = (shooterLimelightY * -17.3667) + 335;
            } else {
                targetLeadScrewPosition = (shooterLimelightY * -17.3667) + 335;
            }
            // targetLeadScrewPosition = (shooterLimelightY * -17.3667) + 320; comp offset
        }

        if (targetLeadScrewPosition < 0) {
            targetLeadScrewPosition = 0.0;
        } else if (targetLeadScrewPosition > 320) {
            targetLeadScrewPosition = 320.0;
        }
    }

    public void turnToSpeaker() {
        if (shooterTidValue != -1) {
            rightX = shooterLimelightX * SHOOTER_LIMELIGHT_X_SCALE;
        } else {
            rightX = 0;
        }
    }

    public void autoMainRoutine() {
        calcShooterSpeed();
        calcLeadScrewPosition();
        autoIntakeNote();
        turnToSpeaker();
    }

    public void calcLimelightOffsetX() { 
        // robot goes back -> -Vy
        // robot goes left -> +Vx
        if (shooterTidValue == 0) {
            limelightOffsetX = 0;
            return;
        }
        double Vx = gyro.getVelocityX() * Math.cos(getBoundedGyroAngle()) + 
                    gyro.getVelocityY() * Math.sin(getBoundedGyroAngle());

        if (Vx < 0.2) {
            limelightOffsetX = 0;
            return;
        }
        double yDist = 32; // measured

        double xDist = Math.tan((59.6 * shooterLimelightX/160) * Math.PI / 180) * yDist;

        // double yDist = 1/Math.tan((49.7 * shooterLimelightY/120) * Math.PI / 180) * 32.25;
        // double elevationAngle = 10; // also measured

        double zDist = (shooterLimelightY - 16.5) * 25/6.7 + 58;

        double shooterAngle = (leadScrewEncoder.getPosition() - 166.5) * 2/27.4 + 50;
        double noteVz = Math.round(Math.cos(shooterAngle * Math.PI / 180) * 
                    shooterTopEncoder.getVelocity() * 4 * Math.PI / 100) * 100;

        double shootTime = Math.abs(zDist / noteVz);
        double timeNeeded = Math.abs(xDist / Vx);

        double offsetSpeed = 1;

        if (timeNeeded < shootTime) { // aim closer
            if (xDist < 0) { // april tag is left of robot
                limelightOffsetX += offsetSpeed;
            } else {
                limelightOffsetX -= offsetSpeed;
            }
        } else { // aim farther
            if (xDist < 0) { // april tag is right of robot
                limelightOffsetX -= offsetSpeed;
            } else {
                limelightOffsetX += offsetSpeed;
            }
        }
    }

    public double getBoundedGyroAngle() {
        double gyroAngle;

        // if (autoChooser.getSelected() == "Simple Left") {
        //     gyroOffset = -60;
        // } else if (autoChooser.getSelected() == "Simple Right") {
        //     gyroOffset = 60;
        // } else {
        //     gyroOffset = 0;
        // }

        // gyroAngle = (gyro.getAngle() + gyroOffset) % 360;
        gyroAngle = gyro.getAngle() % 360;

        if (gyroAngle < 0) {
            gyroAngle += 360;
        }

        return gyroAngle;
    }

    // public double calcArcLength(double xDist, double yDist, double endAngle) {
    //     double arcLength = (Math.PI * Math.sqrt(2 * (Math.pow(yDist, 2) + Math.pow(xDist, 2))) + Math.PI * (xDist + yDist)) / 2.0;
    //     arcLength *= (endAngle - getBoundedGyroAngle()) / 360.0;

    //     while (arcLength < 0) arcLength += 360;

    //     return arcLength;
    // }

    public void calcRightX(double targetAngle) {
        angleDiff = getBoundedGyroAngle() - targetAngle;

        if (angleDiff < -180) {
            angleDiff += 360;
        } else if (angleDiff > 180) {
            angleDiff -= 360;
        }

        // if (Math.abs(angleDiff) < 30) {
        //     rightX = MathUtil.clamp(-(angleDiff / RIGHTX_SCALE), -0.3, 0.3);
        // } else {
            rightX = MathUtil.clamp(-(angleDiff / RIGHTX_SCALE), -0.8, 0.8);
        // }

        // Prevent wobble while auto aligning
        if ((magnitude > 0) &&
                (Math.abs(rightX) < DEAD_ZONE)) {
            rightX = 0.0;
        }
    }

    // Used to calculate the position that the lead screw for the shooter needs to be in
    public double calcShooterPosition(double limelightY) {
        return (limelightY * LEAD_SCREW_SCALE - leadScrewEncoder.getPosition()) * LEAD_SCREW_SPEED_SCALE;
    }

    // Adjust absolute range of through bore encoder
    public double getAdjustedAbsoluteEncoder() {
        double absoluteEncoderPosition = armAbsoluteEncoder.getAbsolutePosition();
        absoluteEncoderPosition += 0.5;

        if (absoluteEncoderPosition > 1.0) {
            absoluteEncoderPosition -= 1.0;
        }

        return absoluteEncoderPosition;
    }

    // Set drive motors to brake mode
    public void setBrakeMode() {
        frontLeftSwerve.setBrakeMode();
        backLeftSwerve.setBrakeMode();
        backRightSwerve.setBrakeMode();
        frontRightSwerve.setBrakeMode();
    }

    // Set drive motors to coast mode
    public void setCoastMode() {
        frontLeftSwerve.setCoastMode();
        backLeftSwerve.setCoastMode();
        backRightSwerve.setCoastMode();
        frontRightSwerve.setCoastMode();
    }

    // Reset all of the drive encoders
    public void resetDriveEncoders() {
        frontLeftSwerve.resetDriveEncoder();
        backLeftSwerve.resetDriveEncoder();
        backRightSwerve.resetDriveEncoder();
        frontRightSwerve.resetDriveEncoder();
    }

    // Drive in a direction in Auto Mode
    public void autoDrive(double direction, double distance, double maxSpeed, double turnAngle, double beginTurn) {
        if (distance <= 60) {
            drivePID.setP(DRIVE_P_HIGH);
        } else if (distance >= 180) {
            drivePID.setP(DRIVE_P_LOW);
        } else {
            drivePID.setP(DRIVE_P_LOW + ((DRIVE_P_HIGH - DRIVE_P_LOW) * ((180 - distance) / 120.0)));
        }

        // double targetAngle = currentAngle;

        // driveAngle = direction;
        driveAngle = direction -= getBoundedGyroAngle();

        // Get the magnitude of the drive based on which direction the encoder is climbing
        // if (frontLeftSwerve.getDriveEncoder() < 0) {
        //     magnitude = (Math.abs(frontLeftSwerve.getDriveEncoder() + (distance * ENCODER_DISTANCE_SCALE))) * 0.05;

        //     // Back up if driving passed setpoint
        //     if ((frontLeftSwerve.getDriveEncoder() + (distance * ENCODER_DISTANCE_SCALE)) < 0) {
        //         driveAngle += 180;
        //     }
        // } else {
        //     magnitude = (Math.abs(frontLeftSwerve.getDriveEncoder() - (distance * ENCODER_DISTANCE_SCALE))) * 0.05;

        //     // Back up if driving passed setpoint
        //     if ((frontLeftSwerve.getDriveEncoder() - (distance * ENCODER_DISTANCE_SCALE)) > 0) {
        //         driveAngle += 180;
        //     }
        // }
        // magnitude = (getDriveEncoders() - (distance * ENCODER_DISTANCE_SCALE)) * 0.05;

        // Back up if driving passed setpoint
        // if ((getDriveEncoders() - (distance * ENCODER_DISTANCE_SCALE)) > 0) {
        //     driveAngle += 180;
        // }


        // if (magnitude < 0) {
        //     driveAngle += 180;
        //     driveAngle %= 360;
        // }

        if (turnAngle != -1) {
            if (Math.abs(getBoundedGyroAngle() - startGyroAngle) < 180) {
                magnitude = drivePID.calculate(getDriveEncoders(), (distance + Math.abs(getBoundedGyroAngle() - startGyroAngle) * Math.PI / 180 * Math.hypot(10.25, 10.25)) * ENCODER_DISTANCE_SCALE);
            } else {
                magnitude = drivePID.calculate(getDriveEncoders(), (distance + (360 - Math.abs(getBoundedGyroAngle() - startGyroAngle)) * Math.PI / 180 * Math.hypot(10.25, 10.25)) * ENCODER_DISTANCE_SCALE);
            }
            if (getDriveEncoders() > (distance * ENCODER_DISTANCE_SCALE * beginTurn)) {

                calcRightX(turnAngle);

                if (Math.abs(prevRightX - rightX) < 0.15) {
                    prevRightX = rightX;
                } else if (prevRightX > rightX) {
                    prevRightX -= 0.15;
                    rightX = prevRightX;
                } else if (prevRightX < rightX) {
                    prevRightX += 0.15;
                    rightX = prevRightX;
                }

                driveAngle -= (rightX * 12);
            } else {
                rightX = 0.0;
            }
        } else {
            magnitude = drivePID.calculate(getDriveEncoders(), distance * ENCODER_DISTANCE_SCALE);
        }

        // Keep angle between 0 and 360
        if (driveAngle < 0) {
            driveAngle += 360;
        }

        if (driveAngle > 360) {
            driveAngle -= 360;
        }

        // Cap magnitude at max speed
        if (magnitude > maxSpeed) {
            magnitude = maxSpeed;
        }

        // targetAngle = currentAngle - getBoundedGyroAngle();

        // if (targetAngle > 180) {
        //     targetAngle -= 360;
        // } else if (targetAngle < -180) {
        //     targetAngle += 360;
        // }

        // rightX = -targetAngle * 5;

        // Move onto next stage if reasonably close to target
        if (magnitude < 0.03) {
            autoStage++;
            resetDriveEncoders();
            startGyroAngle = getBoundedGyroAngle();
            noteStaged = false;
            shooterAtTarget = false;

            magnitude = 0;
            driveAngle = 0;
            rightX = 0;
            prevRightX = 0;

            startTime = timer.get();
        }
    }

    // Turn a set amount in Auto Mode
    public void autoTurn(double targetAngle, double maxSpeed) {
        magnitude = 0;
        driveAngle = 0;

        calcRightX(targetAngle);

        // Cap Turn speed
        if (rightX > maxSpeed) {
            rightX = maxSpeed;
        } else if (rightX < -maxSpeed) {
            rightX = -maxSpeed;
        }

        // Move onto next stage if reasonably close to target angle
        if (Math.abs(angleDiff) < 3.0) {
            autoStage++;
            rightX = 0;
            resetDriveEncoders();
            currentAngle = targetAngle;
            startTime = timer.get();
            startGyroAngle = getBoundedGyroAngle();
            shooterAtTarget = false;
            noteStaged = false;
        }
    }

    public void turnToAngle(double targetAngle, double maxSpeed) {
        magnitude = 0;
        driveAngle = 0;

        calcRightX(targetAngle);

        // Cap Turn speed
        if (rightX > maxSpeed) {
            rightX = maxSpeed;
        } else if (rightX < -maxSpeed) {
            rightX = -maxSpeed;
        }
    }

    public double getDriveEncoders() {
        return (Math.abs(frontLeftSwerve.getDriveEncoder()) + 
                Math.abs(backLeftSwerve.getDriveEncoder()) + 
                Math.abs(backRightSwerve.getDriveEncoder()) + 
                Math.abs(frontRightSwerve.getDriveEncoder())) / 4;
    }
    public double getDriveDistInches() {
        return getDriveEncoders() / ENCODER_DISTANCE_SCALE;
    }
}