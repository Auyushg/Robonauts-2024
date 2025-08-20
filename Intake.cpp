/*******************************************************************************
 *
 * File: Intake.cpp
 *
 * Written by:
 *     The Robonauts
 *     FRC Team 118
 *     NASA, Johnson Space Center
 *     Clear Creek Independent School District
 *
 ******************************************************************************/

#include "gsu/Advisory.h"
#include "FlightBase/LuaState.h"
#include "Intake.h"
#include "gsu/Advisory.h"
#include "RobonautsLibrary/RobotUtil.h"
#include "RobonautsLibrary/StateData2.h"
#include "frc/shuffleboard/Shuffleboard.h"

bool Intake::m_exit_sees_note = false;

/* Create and initialize all of the elements of the subsystem */
Intake::Intake(std::string ctrl_name)
    : RSubsystem(ctrl_name)
    , m_left_motor(nullptr)
    , m_right_motor(nullptr)
{
    Advisory::pinfo("========================= Creating SubSystem [%s] =========================\n", ctrl_name.c_str());

    addRSpeedController("left_motor", &m_left_motor);
    addRSpeedController("right_motor", &m_right_motor);
    addOI(&m_left_axis);
    addOI(&m_right_axis);
    addOI(&m_left_in_btn);
    addOI(&m_right_in_btn);
    addOI(&m_left_in_stop_btn);
    addOI(&m_right_in_stop_btn);
    addOI(&m_left_out_btn);
    addOI(&m_right_out_btn);
    addOI(&m_left_in_slow_btn);
    addOI(&m_right_in_slow_btn);
    addOI(&m_left_out_slow_btn);
    addOI(&m_right_out_slow_btn);
    addOI(&m_in_no_stop_btn);
    addOI(&m_left_right_pass_btn);
    addOI(&m_right_left_pass_btn);
    addOI(&m_dyad_in_btn);

}

/*  Destructor, releases any resources allocated by this class */
Intake::~Intake(void) {}

bool Intake::hasPiece()
{
    return (m_note_in_lower_intake);
}

/* Initializes preferences that appear on the ShuffleBoard

   In the example below, button_cmd is a double. Doubles, floats, integers, strings, and booleans can be used with
   preferences

   if (!frc::Preferences::ContainsKey("Intake/button_cmd")) {
        frc::Preferences::SetDouble("Intake/button_cmd", button_cmd);
   }
*/
void Intake::initPreferences() {
    if (!frc::Preferences::ContainsKey("Intake/left_cmd")) {
        frc::Preferences::SetDouble("Intake/left_cmd", m_left_cmd);
    }
    if (!frc::Preferences::ContainsKey("Intake/right_cmd")) {
        frc::Preferences::SetDouble("Intake/right_cmd", m_right_cmd);
    }
    if (!frc::Preferences::ContainsKey("Intake/joystick_left_scale")) {
        frc::Preferences::SetDouble("Intake/joystick_left_scale", m_joystick_left_scale);
    }
    if (!frc::Preferences::ContainsKey("Intake/joystick_right_scale")) {
        frc::Preferences::SetDouble("Intake/joystick_right_scale", m_joystick_right_scale);
    }
    if (!frc::Preferences::ContainsKey("Intake/sensor_control")) {    // sensor control should to be renamed, name is not clear
        frc::Preferences::SetDouble("Intake/sensor_control", m_sensor_speed_reduction);
    }

}

/* Reads preferences that appear on the ShuffleBoard

   In this example the preference "Intake/button_cmd" is read.  If the value does not exist, the value of
   button_cmd is returned.  Doubles, floats, integers, strings, and booleans can be used with preferences.

   button_cmd = frc::Preferences::GetDouble("Intake/button_cmd", button_cmd);

*/
void Intake::readPreferences() {
//    m_left_cmd = frc::Preferences::GetDouble("Intake/left_cmd", m_left_cmd);
//    m_right_cmd = frc::Preferences::GetDouble("Intake/right_cmd", m_right_cmd);
    m_joystick_left_scale = frc::Preferences::GetDouble("Intake/joystick_left_scale", m_joystick_left_scale);
    m_joystick_right_scale = frc::Preferences::GetDouble("Intake/joystick_right_scale", m_joystick_right_scale);
    m_sensor_speed_reduction = frc::Preferences::GetDouble("Intake/sensor_control", m_sensor_speed_reduction);   // sensor_control should be renamed see above

    // set left in and out based on left nominal value (which should always be positive)
    // m_left_cmd_in = -m_left_cmd;
    // m_left_cmd_out = m_left_cmd;
    // // set right in and out based on right nominal value (which should always be positive)
    // m_right_cmd_in = m_right_cmd;
    // m_right_cmd_out = -m_right_cmd;

}

/* Take care of any initialization that needs to be done after all controls have been created. */
void Intake::RobotInit() {
    if ( m_left_motor ) {
        m_left_motor->SetControlMode(RSpeedController::VELOCITY);
//        m_left_motor->SetControlMode(RSpeedController::DUTY_CYCLE);
    }
    if ( m_right_motor ) {
        m_right_motor->SetControlMode(RSpeedController::VELOCITY);
//        m_right_motor->SetControlMode(RSpeedController::DUTY_CYCLE);
    }
    frc::Shuffleboard::GetTab("Intake").Add("intake", *this).WithSize(3, 4).WithPosition(0, 0);
}

//************************************************************************
// method that reads sensors from the intake system
//************************************************************************
void Intake::readSensors()
{
    if ( m_left_motor) {  // combining to simplify
        m_left_motor_velocity = m_left_motor->GetSpeed();
        m_left_motor_curr = m_left_motor->GetOutputCurrent();
        m_left_motor_dc = m_left_motor->GetMotorOutputPercent();
    }

    if (m_right_motor) {
        m_right_motor_velocity = -m_right_motor->GetSpeed();   // invert so left and right are the same
        m_right_motor_curr = m_right_motor->GetOutputCurrent();
        m_right_motor_dc = m_right_motor->GetMotorOutputPercent();
    }

    if(beam_sensor_left != nullptr){
        m_left_sees_note = !beam_sensor_left->Get();
        m_right_sees_note = !beam_sensor_right->Get();
        m_exit_sees_note = !beam_sensor_top->Get();
    }
    m_note_in_lower_intake = (m_left_sees_note == true || m_right_sees_note == true);

}

// method to check if the robot is intaking
bool Intake::isIntaking(bool left, bool right, bool force)
{
    bool ret = false;
    if(force == true)
    {
        ret = true;
    }
    else if( left == true && right == true) 
    {
        ret = true;
    }
    else if(left == false && right == false) {
        ret = false;
    }

    StateData2::GetInstance().set<bool>(name + "/has_piece", m_note_in_lower_intake);

    return ret;
}

//************************************************************************
// method that reads sensors from the intake system
//************************************************************************
void Intake::sensorOutputAuto()
{
    if(m_left_sees_note == true || m_right_sees_note == true){
        if(m_left_sees_note == true) {
            m_running_left = m_sensor_speed_reduction * m_left_cmd;
            m_running_right = m_sensor_speed_reduction * m_right_cmd;
        }
        if(m_right_sees_note == false) {
            m_running_left = m_sensor_speed_reduction * m_left_cmd;
            m_running_right = m_sensor_speed_reduction * m_right_cmd;
        }
        if(m_right_sees_note == true && m_left_sees_note == true) {
            m_running_right = m_sensor_speed_reduction * m_left_cmd;
            m_running_left = m_sensor_speed_reduction * m_right_cmd;
        }
        if (m_exit_sees_note == true) {
            //m_running_left = 0;
            //m_running_right = 0;
        }
    }

}

void Intake::sensorOutputManual()
{
//    if(m_note_in_lower_intake == true && m_is_intaking == true)  
//    {
//        m_running_left = m_sensor_speed_reduction * m_left_cmd_in;   // sensor_control is maybe slow down when note in intak if set less than 1
//        m_running_right = m_sensor_speed_reduction * m_right_cmd_in;
//    }

    // comment need logic for seeing top sensor, which might replace this logic (once we have that sensor)
    // for when it turns off on exit of note
    // if(m_note_in_lower_intake == false && m_note_in_lower_intake_last == true)  // falling edge
    // {
    //     m_running_left = m_running_right = 0;
    // }
    // for when it turns off on entrance of note
    if(m_note_in_lower_intake == true && m_note_in_lower_intake_last == false)  // falling edge
    {
        m_cycles_count = m_cycles_to_wait_after_sensor; // start timer
    }
    if(m_cycles_count >= 0)
    {
        m_cycles_count--;
    }
    if(m_cycles_count == 0)
    {
        m_intake_note = false;
        m_running_left = m_running_right = 0;
    }



}

//************************************************************************
// saves robot state for next time
//************************************************************************
void Intake::storeState()
{
    m_left_sees_note_last = m_left_sees_note;  // sensor response is not very descriptive, should be changed
    m_right_sees_note_last = m_right_sees_note;  // sensor response is not very descriptive, should be changed
    m_exit_sees_note_last = m_exit_sees_note;  // sensor response is not very descriptive, should be changed
    m_note_in_lower_intake_last = m_note_in_lower_intake;
}

//************************************************************************
// method that writes the outputs to the motors
//************************************************************************
void Intake::writeEffectors()
{
    if (m_left_motor) {
        // set the motor to the speed of cmd
        m_left_motor->Set(m_running_left);
    }
    if(m_right_motor) {
        m_right_motor->Set(m_running_right);
    }
}

void Intake::rollIn() {
    m_running_left = m_left_cmd_in;
    m_running_right = m_right_cmd_in;
}

void Intake::rollOut() {
    m_running_left = m_left_cmd_out;
    m_running_right = m_right_cmd_out;
}

void Intake::rollOff() {
    m_running_left = 0;
    m_running_right = 0;
}


void Intake::intakeNote() {
    m_intake_note = true;
    rollIn(); 
}

//************************************************************************
// method that reads the joysticks for the intake system
//************************************************************************
void Intake::handleOI(void) {
    bool shooter_has_piece = StateData2::GetInstance().get<bool>("shooter_control/has_piece", false);

    // read left side button, in, out or off
    if(m_left_in_btn.GetButton()) {
        m_running_left = m_left_cmd_in;
    }
    if(m_left_out_slow_btn.GetButton()) {
        m_running_left = m_left_cmd_out_slow;
    }
    if(m_left_out_btn.GetButton()) {
        m_running_left = m_left_cmd_out;
    }
    if(m_left_in_slow_btn.GetButton()) {
        m_running_left = m_left_cmd_in_slow;
    }

    // if(m_left_in_stop_btn.GetButtonPressed() && !m_note_in_lower_intake && m_had_piece_on_chamber) {
    //     m_running_left = m_left_cmd_in_slow;
    // } else if(m_left_in_stop_btn.GetButtonReleased()) {
    //     m_cycles_count = -1;
    // }

    if(m_right_in_btn.GetButton()) {
        m_running_right = m_right_cmd_in;
    }
    if(m_right_out_slow_btn.GetButton()) {
        m_running_right = m_right_cmd_out_slow;
    }
    if(m_right_out_btn.GetButton()) {
        m_running_right = m_right_cmd_out;
    }
    if(m_right_in_slow_btn.GetButton()) {
        m_running_right = m_right_cmd_in_slow;
    }
    // if(m_right_in_stop_btn.GetButtonPressed() && !m_note_in_lower_intake && m_had_piece_on_chamber) {
    //     m_running_right = m_right_cmd_in_slow;
    // } else if(m_right_in_stop_btn.GetButtonReleased()) {
    //     m_cycles_count = -1;
    // }

     if(m_in_no_stop_btn.GetButton()) {
        m_running_left = m_left_cmd_in;
        m_running_right = m_right_cmd_in;
    }

    if(m_dyad_in_btn.GetButton() && !m_had_piece_on_chamber && !shooter_has_piece) {
        m_running_left = m_left_cmd_in;
        m_running_right = m_right_cmd_in;
    } else if (m_dyad_in_btn.GetButton() && !m_had_piece_on_chamber && shooter_has_piece) {
        m_running_left = m_left_cmd_out;
        m_running_right = m_right_cmd_out;
    }

    if(m_dyad_in_btn.GetButton() && !m_note_in_lower_intake && m_had_piece_on_chamber) {
        m_running_right = m_right_cmd_in;
        m_running_left = m_left_cmd_in;
    } else if (m_note_in_lower_intake && shooter_has_piece) {
        m_running_left = 0;
        m_running_right = 0;
    }
    else if(m_dyad_in_btn.GetButtonReleased()) {
        m_cycles_count = -1;
    }


    if (!m_left_in_btn.GetButton() && !m_in_no_stop_btn.GetButton() && !m_left_in_stop_btn.GetButton() && !m_left_out_btn.GetButton() 
                && !m_left_in_slow_btn.GetButton() && !m_left_out_slow_btn.GetButton() && !m_dyad_in_btn.GetButton()) {
        m_running_left = 0.0;
    }
    if (!m_right_in_btn.GetButton() && !m_in_no_stop_btn.GetButton() && !m_right_in_stop_btn.GetButton() && !m_right_out_btn.GetButton() 
                && !m_right_in_slow_btn.GetButton() && !m_right_out_slow_btn.GetButton() && !m_dyad_in_btn.GetButton()) {
        m_running_right = 0.0;
    }


    // if(shooter_has_piece){
    //     if (!m_note_in_lower_intake && !m_right_in_stop_btn.GetButton()){
    //         m_running_left = m_left_cmd_out_slow;
    //         m_running_right = m_right_cmd_out_slow;
    //     }

    // }

    if (m_dyad_in_btn.GetButton()) {
        if (m_was_chambering_last_period == false) {
            m_had_piece_on_chamber = shooter_has_piece;
            m_was_chambering_last_period = true;
        }
    } else {
        m_was_chambering_last_period = false;
    }

    if (m_note_in_lower_intake && shooter_has_piece) {
        m_two_note_clock++;
    } else {
        m_two_note_clock=0;
    }

    if (m_two_note_clock >= m_two_note_max_clock) {
        m_running_left = m_left_cmd_out;
        m_running_right = m_right_cmd_out;
    }

    if (m_left_sees_note && m_right_sees_note) {
        m_running_left = m_left_cmd_out;
        m_running_right = m_right_cmd_out;
    }

    m_is_intaking = isIntaking(m_right_in_stop_btn.GetButton(),m_left_in_stop_btn.GetButton(), false);

    if(m_left_right_pass_btn.GetButton()) {
        m_running_left = m_left_cmd_in;
        m_running_right = m_right_cmd_out;
    }

    if(m_right_left_pass_btn.GetButton()) {
        m_running_left = m_left_cmd_out;
        m_running_right = m_right_cmd_in;
    }
}


/**********************************************************************
 *
 * Runs on a clock, separate from main class at a period specified in RoboControl.lua
 * 1. Read any sensors
 * 2. Run logic based on sensor and user inputs (coming in through setAnalog/setDigital)
 * 3. Write to effectors (either motors or relays ...)
 *
 **********************************************************************/
void Intake::RobotPeriodic(void)
{
    readSensors();
         // get sensor values from motors (or others)
    if (!IsAutonomous()) {
        handleOI();
        if(m_is_intaking || m_intake_note) {
            sensorOutputManual();
        }
    }
    writeEffectors();

    storeState();
}

/* Set the robot into a safe mode */
void Intake::DisabledInit(void) {}

/* Run Disabled specific commands before RobotPeriodic */
void Intake::DisabledPeriodic(void) {}

/* Prepare for Autonomous Operations */
void Intake::AutonomousInit(void) 
{
    m_cycles_count = -1;
}

/* Run Autonomous specific commands before RobotPeriodic */
void Intake::AutonomousPeriodic(void) {}

void Intake::setSpeed(double speed) {
    m_left_cmd = m_right_cmd = RobotUtil::limit(-1.0, 1.0, speed);
}

// set close loop gains, both set the same (for now)
void Intake::setGains(double kp, double ki, double kd, double kf) 
{
    if(m_left_motor != nullptr)
    {
        m_left_motor->SetP(kp);
        m_left_motor->SetI(ki);
        m_left_motor->SetD(kd);
        m_left_motor->SetF(kf);
    }
    if(m_right_motor != nullptr)
    {
        m_right_motor->SetP(kp);
        m_right_motor->SetI(ki);
        m_right_motor->SetD(kd);
        m_right_motor->SetF(kf);
    }

}

//set slow coefficient
void Intake::setSlowCoef(double coef){
    m_slow_coef = coef;

    // set left in and out based on left nominal value (which should always be positive)
    m_left_cmd_in_slow = -m_left_cmd * m_slow_coef;//hardcoded to 10 pct
    m_left_cmd_out_slow = m_left_cmd * m_slow_coef;//hardcoded to 10 pct
    // set right in and out based on right nominal value (which should always be positive)
    m_right_cmd_in_slow = m_right_cmd * m_slow_coef;//hardcoded to 10 pct
    m_right_cmd_out_slow = -m_right_cmd * m_slow_coef;//hardcoded to 10 pct
}

// set close loop gains, both set the same (for now)
void Intake::setSetpoints(double left_sp, double right_sp) 
{
    m_left_cmd = left_sp;
    m_right_cmd = right_sp;

    // set left in and out based on left nominal value (which should always be positive)
    m_left_cmd_in = -m_left_cmd;
    m_left_cmd_out = m_left_cmd;
    // set right in and out based on right nominal value (which should always be positive)
    m_right_cmd_in = m_right_cmd;
    m_right_cmd_out = -m_right_cmd;
}

void Intake::setCurrentLimits(double supplyCurrentLimit, double supplyTriggerCurrent, double supplyThresholdTime, double statorCurrentLimit, double statorTriggerCurrent, double statorThresholdTime) {
    m_left_motor->SetCurrentLimit(supplyCurrentLimit,supplyTriggerCurrent,supplyThresholdTime);
    m_right_motor->SetCurrentLimit(supplyCurrentLimit,supplyTriggerCurrent,supplyThresholdTime);

    m_left_motor->SetStatorCurrentLimit(statorCurrentLimit,statorCurrentLimit,supplyThresholdTime);
    m_right_motor->SetStatorCurrentLimit(statorCurrentLimit,statorCurrentLimit,supplyThresholdTime);

}

void Intake::SetBrakeMode(bool brake_mode){
    m_right_motor->SetBrakeMode(brake_mode);
    m_left_motor->SetBrakeMode(brake_mode);
}

void Intake::setSensorSpeedReduction(double sensorSpeedReduction) {
    m_sensor_speed_reduction = sensorSpeedReduction;
}

void Intake::addNoteSensors(int leftChannel, int rightChannel, int topChannel) {
    beam_sensor_left = new frc::DigitalInput(leftChannel);
    beam_sensor_right = new frc::DigitalInput(rightChannel);
    beam_sensor_top = new frc::DigitalInput(topChannel);
}

/* Prepare for Teleop Operations */
void Intake::TeleopInit(void) 
{
    m_cycles_count = -1;
}

/* Run Teleop specific commands before RobotPeriodic */
void Intake::TeleopPeriodic(void) {}

void Intake::TestInit(void) {}

/* addLogVars is used to initialize the variables to log

   Use the call addLogVar(std::string, variable).  The variable can be a floating point (double or float), integer
   or boolean.

   addLogVar("value", value);
*/
void Intake::addLogVars() {
}


/* InitSendable is used to initialize the variables to send to the Shuffleboard

   AddBooleanProperty and AddDoubleProperty are the 2 most common methods to use to add varaibles to the shuffleboard.

   void AddBooleanProperty(std::string_view key, std::function< bool()> getter, std::function< void(bool)> setter);
   void AddDoubleProperty(std::string_view key, std::function< double()> getter, std::function< void(double)> setter);

   Functions must be used to access variables.  These can be getters and setters to variables. If a getter/setter does
   not exist, then a c++ lambda function can be used.  For example, if we want to display the variable "value" that has
   no getter and we don't want to allow the value to be set from the shuffleboard.

   builder.AddDoubleProperty("value", [this]{return value;}, nullptr);
*/
void Intake::InitSendable(wpi::SendableBuilder &builder) {
    builder.AddBooleanProperty("01. Left sees note", [this]{ return m_right_sees_note;}, nullptr);
    builder.AddBooleanProperty("02. Right sees note", [this]{ return m_left_sees_note;}, nullptr);
    builder.AddBooleanProperty("03. Top sees note", [this]{ return m_exit_sees_note;}, nullptr);
    builder.AddDoubleProperty("04. Left dc cmd", [this]{ return m_running_left;}, nullptr);
    builder.AddDoubleProperty("05. Left dc", [this]{ return m_left_motor_dc;}, nullptr);
    builder.AddDoubleProperty("06. Left vel", [this]{ return m_left_motor_velocity;}, nullptr);
    builder.AddDoubleProperty("07. Left curr", [this]{ return m_left_motor_curr;}, nullptr);
    builder.AddDoubleProperty("08. Right dc cmd", [this]{ return m_running_right;}, nullptr);
    builder.AddDoubleProperty("09. Right dc", [this]{ return m_right_motor_dc;}, nullptr);
    builder.AddDoubleProperty("10. Right vel", [this]{ return m_right_motor_velocity;}, nullptr);
    builder.AddDoubleProperty("11. Right curr", [this]{ return m_right_motor_curr;}, nullptr);
    builder.AddDoubleProperty("12. cmd while inside", [this]{ return m_sensor_speed_reduction;}, nullptr);
    builder.AddDoubleProperty("13. left nom cmd", [this]{ return m_left_cmd;}, nullptr);
    builder.AddDoubleProperty("14. right nom cmd", [this]{ return m_right_cmd;}, nullptr);
    builder.AddDoubleProperty("14. slow coef", [this]{ return m_slow_coef;}, nullptr);
    builder.AddBooleanProperty("15. chambering last period", [this]{return m_was_chambering_last_period;}, nullptr);    
    builder.AddBooleanProperty("16. had piece on chamber", [this]{return m_had_piece_on_chamber;}, nullptr);
    
}


/* luaRegister is where to tell Lua which functions to include in RobotControl.lua and autons

   luaRegister has a chained list of calls that define the Subsystem and adds function definitions to Lua.  The line
   that starts "luabridge::getGlobalNamespace" through ".endNamespace()" is technically one line of code.
   To add functions use .addFunction("lua_name",&Intake::method) after the "addConstructor" line in the chain.
 */
void luaRegisterIntake() {
    Advisory::pinfo("registering Intake with lua");
    lua_State * L = getLuaState();
    luabridge::getGlobalNamespace(L)
     .beginNamespace("robonauts")
      .deriveClass<Intake, RSubsystem>("Intake")
      .addFunction("rollIn",&Intake::rollIn)
      .addFunction("rollOut",&Intake::rollOut)
      .addFunction("rollOff",&Intake::rollOff)
      .addFunction("intakeNote",&Intake::intakeNote)
      .addFunction("setSpeed",&Intake::setSpeed)
      .addFunction("setSlowCoef",&Intake::setSlowCoef)
      .addFunction("setCurrentLimits",&Intake::setCurrentLimits)
      .addFunction("setSensorSpeedReduction",&Intake::setSensorSpeedReduction)
      .addFunction("addNoteSensors",&Intake::addNoteSensors)
      .addFunction("getBeamSensor1",&Intake::getBeamSensor1)   // should be get sensor with an argument
      .addFunction("getBeamSensor2",&Intake::getBeamSensor2)
      .addFunction("getBeamSensor3",&Intake::getBeamSensor3)
      .addFunction("SetBrakeMode",&Intake::SetBrakeMode)       // should be Set or set, but not both
      .addFunction("setGains",&Intake::setGains)
      .addFunction("setSetpoints",&Intake::setSetpoints)   // setpoint in motor rev/s
      .addFunction("setCyclesBeforeStop",&Intake::setCyclesBeforeStop)   // setpoint in motor rev/s
      .addFunction("setMaxTimeWithTwoNotes",&Intake::setMaxTimeWithTwoNotes)
      .addFunction("hasPiece", &Intake::hasPiece)
      .addConstructor<void(*)(std::string)>()
      .endClass()
     .endNamespace();
}

