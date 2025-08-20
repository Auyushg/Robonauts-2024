/*******************************************************************************
 *
 * File: Intake.h
 *
 * Written by:
 * 	The Robonauts
 * 	FRC Team 118
 * 	NASA, Johnson Space Center
 * 	Clear Creek Independent School District
 *
 ******************************************************************************/
#pragma once

#include "wpi/sendable/Sendable.h"
#include "wpi/sendable/SendableBuilder.h"
#include "FlightBase/RSubsystem.h"
#include "frc/Preferences.h"
#include <frc/filter/Debouncer.h>
#include "RobonautsLibrary/RSpeedController.h"
#include "RobonautsLibrary/OIButton.h"
#include "RobonautsLibrary/OIAxis.h"

#include "RobonautsLibrary/RDigitalInput.h"

#include <vector>

class Intake : public RSubsystem, public wpi::Sendable
{
  public:
    Intake(std::string name);
    ~Intake(void);

    void rollIn();
    void rollOut();
    void rollOff();
    void rollAtSpeed(double speed);
    void intakeNote();

    bool hasPiece();

    void setSpeed(double speed);
    void setCurrentLimits(double supplyCurrentLimit, double supplyTriggerCurrent, double supplyThresholdTime, double statorCurrentLimit, double statorTriggerCurrent, double statorThresholdTime);
    void setSensorSpeedReduction(double sensorSpeedReduction);
    void addNoteSensors(int leftChannel, int rightChannel, int topChannel);
    void setGains(double kp, double ki, double kd, double kf);
    void SetBrakeMode(bool brake_mode);
    void setSetpoints(double left_sp, double right_sp);
    void setCyclesBeforeStop(unsigned int cycles) {m_cycles_to_wait_after_sensor = cycles;};
    void setMaxTimeWithTwoNotes(double max_seconds_with_two_notes) {m_two_note_max_clock = (int)max_seconds_with_two_notes/0.02;};

    void setSlowCoef(double coef);

    bool getBeamSensor1(){return beam_sensor_left->Get();};
    bool getBeamSensor2(){return beam_sensor_right->Get();};
    bool getBeamSensor3(){return beam_sensor_top->Get();};


  protected:
    void RobotInit(void) override;      // Called once upon creation of control
    void RobotPeriodic(void) override;  // called every cycle

    void DisabledInit(void) override;   // called once when system transitions from enabled to disabled
    void DisabledPeriodic(void) override;   // called every cycle before RobotPeriodic when in disabled

    void AutonomousInit(void) override; // called once when system transitions from disabled to enabled in autonomous mode
    void AutonomousPeriodic(void) override; // called every cycle before RobotPeriodic when in autonomous

    void TeleopInit(void) override;     // called once when system transitions from disabled to enabled in teleop mode
    void TeleopPeriodic(void) override;     // called every cycle before RobotPeriodic when in teleop

    void TestInit(void) override;     // called once when system transitions from disabled to enabled in test mode

    void initPreferences();
    void readPreferences();
    void addLogVars();
    void sensorOutputAuto();
    void sensorOutputManual();

    virtual void InitSendable(wpi::SendableBuilder &builder) override;

    void handleOI();

  private:
    void setCurrentLimit(double limit, double peak, double time, RSpeedController *motor);
    void applyVelocityLimits();
    void readSensors();
    void writeEffectors();
    void storeState();

    bool isIntaking(bool left_btn, bool right_btn, bool force);   // force declares that it's intaking (separate from buttons)

    RSpeedController * m_left_motor;
    RSpeedController * m_right_motor;

    OIButtonSet m_open_close_button{"open_close"};

    OIButtonSet m_left_in_btn{"left_in"};
    OIButtonSet m_right_in_btn{"right_in"};

    OIButtonSet m_left_in_stop_btn{"left_in_stop"};
    OIButtonSet m_right_in_stop_btn{"right_in_stop"};
    OIButtonSet m_left_out_btn{"left_out"};
    OIButtonSet m_right_out_btn{"right_out"};
    OIAxis m_left_axis{"left_axis"};
    OIAxis m_right_axis{"right_axis"};

    OIButtonSet m_left_in_slow_btn{"left_in_slow"};
    OIButtonSet m_right_in_slow_btn{"right_in_slow"};
    OIButtonSet m_left_out_slow_btn{"left_out_slow"};
    OIButtonSet m_right_out_slow_btn{"right_out_slow"};

    OIButtonSet m_left_right_pass_btn{"left_right_pass"};
    OIButtonSet m_right_left_pass_btn{"right_left_pass"};

    OIButtonSet m_in_no_stop_btn{"in_no_stop"};

    OIButtonSet m_dyad_in_btn{"dyad_in"};

    frc::DigitalInput* beam_sensor_left = nullptr;
    frc::DigitalInput* beam_sensor_right = nullptr;
    frc::DigitalInput* beam_sensor_top = nullptr;

    double m_cmd {0};
    double m_running_left{0};
    double m_running_right{0};

    double m_left_cmd{50.0};    // cmd is nominal command (in motor rev/s) 50 is approx 50% duty cycle
    double m_left_cmd_in{0};  // in command for running roller in 
    double m_left_cmd_out{0}; // out command is for running roller out
    double m_right_cmd{50.0};
    double m_right_cmd_in{0};
    double m_right_cmd_out{0};

    double m_left_cmd_in_slow{0};
    double m_left_cmd_out_slow{0};
    double m_right_cmd_in_slow{0};
    double m_right_cmd_out_slow{0};
    double m_slow_coef{0};

    double m_left_motor_velocity{0};
    double m_left_motor_curr{0};
    double m_left_motor_dc{0};
    double m_right_motor_velocity{0};
    double m_right_motor_curr{0};
    double m_right_motor_dc{0};

    double m_joystick_left_scale{0};
    double m_joystick_right_scale{0};
    double m_sensor_speed_reduction{0.3};   // value to run rollers when note is in an intake

    bool m_left_sees_note{false};
    bool m_right_sees_note{false};
    static bool m_exit_sees_note;  // should create a static getter, not direct access to this variable

    bool m_is_intaking{false};
    bool m_intake_note{false};
    bool m_note_in_lower_intake{false};

    bool m_left_sees_note_last{false};
    bool m_right_sees_note_last{false};
    bool m_exit_sees_note_last{false};
    bool m_note_in_lower_intake_last{false};

    unsigned int m_cycles_to_wait_after_sensor{5};
    int m_cycles_count{0};

    bool m_was_chambering_last_period{false};
    bool m_had_piece_on_chamber{false};

    int m_two_note_max_clock{static_cast<int>(2.0/0.02)};
    int m_two_note_clock{0};

};

void luaRegisterIntake();


