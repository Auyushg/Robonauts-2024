
-- function to initialize a map between a name and hardware joystick
function add_joysticks()
    -- instantiate an OIMap in the application.
    oi_map = robonauts.OIMap("oi_map")
    oi_map:addJoystick("pilot", 0)
end

function add_macro_control()
    macro_control = robonauts.MacroLuaControl("macro_control")
    -- macro_control:addAuton("name", "auton_file")
end

function add_intake()
    Intake = robonauts.Intake("Intake")
    

    Intake:assignRSpeedControllerTalonFXP6("front_motor", 51, false)
    Intake:assignRSpeedControllerTalonFXP6("back_motor", 52, false)
    -- Intake:assignRSpeedControllerTalonFX("roller_motor", 2, false)
    Intake:assignAxis("front_axis", "pilot", 1, 1.0, 0.05)
    Intake:assignAxis("back_axis", "pilot", 3, 1.0, 0.05)
    
    -- Push into intake
    Intake:assignButton("front_button", "pilot", 5)
    Intake:assignButton("back_button", "pilot", 5)
    --Eject from intake
    Intake:assignButton("front_reverse", "pilot", 7)
    Intake:assignButton("back_reverse", "pilot", 7)
    --front to back
    Intake:assignButton("front_button", "pilot", 6)
    Intake:assignButton("back_reverse", "pilot", 6)
    --back to front
    Intake:assignButton("front_reverse", "pilot", 8)
    Intake:assignButton("back_button", "pilot", 8)

    Intake:setCurrentLimits(80,80,1,80,80,1)

    Intake:SetBrakeMode(false)






   

    -- assignRSpeedControllerTalonFX( std::string name_of_motor, int CAN_ID, bool invert)
    -- assignAxis(std::string axis_name, std::string joystick_name, int axis_number, double scale, double deadband)
    -- assignButton(std::string button_name, std::string joystick_name, int button_number)
    -- assignPOV(std::string pov_name, std::string joystick_name, int channel)
end

-- start of script main program
add_joysticks()
add_macro_control()
add_intake()

