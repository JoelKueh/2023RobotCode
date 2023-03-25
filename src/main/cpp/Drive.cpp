// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drive.h"

Drive::Drive()
{
    frontLeftMotor.SetInverted(true);
    backLeftMotor.SetInverted(true);
    frontLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    frontRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    backLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    backRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    myMecanumDrive = new frc::MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

    autotimer.Start();
    
    autoTP = new frc::TrapezoidProfile<units::feet>(
        frc::TrapezoidProfile<units::feet>::Constraints{2_fps, 1_fps_sq},
        frc::TrapezoidProfile<units::feet>::State{3_ft, 0_fps},
        frc::TrapezoidProfile<units::feet>::State{0_ft, 0_fps}
    );
    gyroTP = new frc::TrapezoidProfile<units::radians>(
        frc::TrapezoidProfile<units::radians>::Constraints{4_rad_per_s, 5_rad_per_s / 1_s},
        frc::TrapezoidProfile<units::radians>::State{(units::radian_t)(deg_to_rad(166)), 0_rad_per_s},
        frc::TrapezoidProfile<units::radians>::State{0_rad, 0_rad_per_s}
    );
}   

void Drive::MecanumDrive(double drivePower, double strafePower, double turnPower)
{
    myMecanumDrive->DriveCartesian(drivePower, strafePower, turnPower);
}

void Drive::autoReset()
{
    delete autoTP;
    autotimer.Reset();
    gyro.Reset();
    frontRightEncoder.SetPosition(0);
    autoTP = new frc::TrapezoidProfile<units::feet>(
        frc::TrapezoidProfile<units::feet>::Constraints{2_fps, 1_fps_sq},
        frc::TrapezoidProfile<units::feet>::State{3_ft, 0_fps},
        frc::TrapezoidProfile<units::feet>::State{0_ft, 0_fps}
    );
}

void Drive::autoReset2()
{
    delete autoTP;
    autotimer.Reset();
    gyro.Reset();
    frontRightEncoder.SetPosition(0);
    autoTP = new frc::TrapezoidProfile<units::feet>(
        frc::TrapezoidProfile<units::feet>::Constraints{2_fps, 2_fps_sq},
        frc::TrapezoidProfile<units::feet>::State{-3_ft, 0_fps},
        frc::TrapezoidProfile<units::feet>::State{0_ft, 0_fps}
    );
}

void Drive::autoReset3()
{
    delete autoTP;
    autotimer.Reset();
    gyro.Reset();
    frontRightEncoder.SetPosition(0);
    autoTP = new frc::TrapezoidProfile<units::feet>(
        frc::TrapezoidProfile<units::feet>::Constraints{4_fps, 4_fps_sq},
        frc::TrapezoidProfile<units::feet>::State{16_ft, 0_fps},
        frc::TrapezoidProfile<units::feet>::State{0_ft, 0_fps}
    );
    armrunonce = true;
}

// void Drive::autoReset4()
// {
//     delete gyroTP;
//     autotimer.Reset();
//     gyro.Reset();
//     frontRightEncoder.SetPosition(0);
//     if(autoswitch.Get())
//     {
//         gyroTP = new frc::TrapezoidProfile<units::radians>(
//         frc::TrapezoidProfile<units::radians>::Constraints{4_rad_per_s, 5_rad_per_s / 1_s},
//         frc::TrapezoidProfile<units::radians>::State{(units::radian_t)(deg_to_rad(166)), 0_rad_per_s},
//         frc::TrapezoidProfile<units::radians>::State{0_rad, 0_rad_per_s}
//     );
//     }
//     else
//     {
//         gyroTP = new frc::TrapezoidProfile<units::radians>(
//         frc::TrapezoidProfile<units::radians>::Constraints{4_rad_per_s, 5_rad_per_s / 1_s},
//         frc::TrapezoidProfile<units::radians>::State{(units::radian_t)(deg_to_rad(-166)), 0_rad_per_s},
//         frc::TrapezoidProfile<units::radians>::State{0_rad, 0_rad_per_s}
//     );
//     }
// }

// void Drive::autoReset5()
// {
//     delete autoTP;
//     autotimer.Reset();
//     gyro.Reset();
//     frontRightEncoder.SetPosition(0);
//     autoTP = new frc::TrapezoidProfile<units::feet>(
//         frc::TrapezoidProfile<units::feet>::Constraints{6_fps, 4_fps_sq},
//         frc::TrapezoidProfile<units::feet>::State{-13.25_ft, 0_fps},
//         frc::TrapezoidProfile<units::feet>::State{0_ft, 0_fps}
//     );
// }

// void Drive::autoReset6()
// {
//     delete gyroTP;
//     autotimer.Reset();
//     gyro.Reset();
//     frontRightEncoder.SetPosition(0);
//     if(autoswitch.Get())
//     {
//         gyroTP = new frc::TrapezoidProfile<units::radians>(
//         frc::TrapezoidProfile<units::radians>::Constraints{4_rad_per_s, 5_rad_per_s / 1_s},
//         frc::TrapezoidProfile<units::radians>::State{(units::radian_t)(deg_to_rad(-173)), 0_rad_per_s},
//         frc::TrapezoidProfile<units::radians>::State{0_rad, 0_rad_per_s}
//     );
//     }
//     else
//     {
//         gyroTP = new frc::TrapezoidProfile<units::radians>(
//         frc::TrapezoidProfile<units::radians>::Constraints{4_rad_per_s, 5_rad_per_s / 1_s},
//         frc::TrapezoidProfile<units::radians>::State{(units::radian_t)(deg_to_rad(173)), 0_rad_per_s},
//         frc::TrapezoidProfile<units::radians>::State{0_rad, 0_rad_per_s}
//     );
//     }
// }

// void Drive::autoReset7()
// {
//     delete autoTP;
//     autotimer.Reset();
//     gyro.Reset();
//     frontRightEncoder.SetPosition(0);
//     autoTP = new frc::TrapezoidProfile<units::feet>(
//         frc::TrapezoidProfile<units::feet>::Constraints{8_fps, 9_fps_sq},
//         frc::TrapezoidProfile<units::feet>::State{-16_ft, 0_fps},
//         frc::TrapezoidProfile<units::feet>::State{0_ft, 0_fps}
//     );
// }

double Drive::MDPID()
{
    auto processVar = autoTP->Calculate(autotimer.Get());
    double position = frontRightEncoder.GetPosition();
    double output = mecanumPID.Calculate(position, feet_to_rot(processVar.position.value()));
    frc::SmartDashboard::PutNumber("autoProcessVar", processVar.position.value());
    frc::SmartDashboard::PutNumber("autoPosition", position);
    frc::SmartDashboard::PutNumber("output", output);
    if(output > 0.25)
    {
        return 0.25;
    }
    else if(output < -0.25)
    {
        return -0.25;
    }
    else
    {
        return output;
    }
}

// double Drive::MDPID2()
// {
//     auto processVar2 = gyroTP->Calculate(autotimer.Get());
//     auto position2 = gyro.GetAngle();
//     double output2 = gyroPID.Calculate(deg_to_rad(position2.value()), processVar2.position.value());
//     frc::SmartDashboard::PutNumber("Gyro Angle", deg_to_rad(position2.value()));
//     frc::SmartDashboard::PutNumber("Gyro ProcessVar", processVar2.position.value());
//     frc::SmartDashboard::PutNumber("Gyro Output", output2);
//     if(output2 > 0.5)
//     {
//         return 0.5;
//     }
//     else if(output2 < -0.5)
//     {
//         return -0.5;
//     }
//     else
//     {
//         return output2;
//     }
// }

void Drive::checkAutoState()
{
    if(autostate == 0)
    {
        if(frontRightEncoder.GetPosition() > 2 && armrunonce)
        {
            arm4 = true;
            armrunonce = false;
        }
        if(frontRightEncoder.GetPosition() > 20.3)
        {
            autostate = 1;
        }
    }
    else if(autostate == 1)
    {
        if(frontRightEncoder.GetPosition() < -20.26)
        {
            autostate = 2;
            runonce = true;
        }
    }
    else if(autostate == 2)
    {
        if(frontRightEncoder.GetPosition() > 5 && armrunonce)
        {
            arm1 = true;
            armrunonce = false;
        }
    }
    // else if(autostate == 3)
    // {
    //     if(autoswitch.Get())
    //     {
    //         if(gyro.GetAngle().value() > rad_to_deg(2.83))
    //         {
    //             autostate = 4;
    //             runonce = true;
    //         }
    //     }
    //     else
    //     {
    //         if(gyro.GetAngle().value() < rad_to_deg(-2.83))
    //         {
    //             autostate = 4;
    //             runonce = true;
    //         }
    //     }
    // }
    // else if(autostate == 4)
    // {
    //     if(frontRightEncoder.GetPosition() < -88.6)
    //     {
    //         autostate = 5;
    //         runonce = true;
    //     }
    // }
    // else if(autostate == 5)
    // {
    //     if(autoswitch.Get())
    //     {
    //         if(gyro.GetAngle().value() < rad_to_deg(-2.95))
    //         {
    //             autostate = 6;
    //             runonce = true;
    //         }
    //     }
    //     else
    //     {
    //         if(gyro.GetAngle().value() > rad_to_deg(2.95))
    //         {
    //             autostate = 6;
    //             runonce = true;
    //         }
    //     }
    // }
}