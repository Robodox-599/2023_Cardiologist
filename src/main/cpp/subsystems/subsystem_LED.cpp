// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_LED.h"

subsystem_LED::subsystem_LED(): m_CANdle{LEDConstants::CANdleID}, m_LEDTimer{} {

    m_LEDTimer.Start();
    
    m_CANdle.ConfigLEDType(ctre::phoenix::led::LEDStripType::RGB);
    m_CANdle.ConfigBrightnessScalar(0.5);
    m_CANdle.ConfigLOSBehavior(true);
}


void subsystem_LED::SetPurpleLED(){
    m_LEDState = LEDConstants::LEDState::Purple;
    m_TriggerState = LEDConstants::LEDState::Purple;
    // m_LEDTimer.Reset();
}

void subsystem_LED::SetYellowLED(){
    // m_LEDTimer.Reset();
    m_LEDState = LEDConstants::LEDState::Yellow;
    m_TriggerState = LEDConstants::LEDState::Yellow;

}

void subsystem_LED::SetStandbyLED(){
    m_LEDState = LEDConstants::LEDState::Standby;

}

void subsystem_LED::SetIntakedLED(){

    //m_CANdle.Animate(IntakedAnimation, 0);

    m_LEDState = LEDConstants::LEDState::Intaked;

}

void subsystem_LED::SetErrorLED(){
    m_LEDState = LEDConstants::LEDState::Error;

}

void subsystem_LED::SetOffLED(){
    m_LEDState = LEDConstants::LEDState::Off;
}

void subsystem_LED::SetTriggerStateLED(){
    m_LEDState = m_TriggerState;
}


// This method will be called once per scheduler run
void subsystem_LED::Periodic() {
        switch(m_LEDState){
        case(LEDConstants::LEDState::Standby):
        // green red blue
            m_CANdle.SetLEDs(255, 0, 0);
            break;
        case(LEDConstants::LEDState::Purple):
            m_CANdle.SetLEDs(0, 255, 255);
            break;
        case(LEDConstants::LEDState::Yellow):
            m_CANdle.SetLEDs(255, 255, 0);
            break;
        case(LEDConstants::LEDState::Intaked):

            // m_CANdle.Animate(IntakedAnimation, 0);
            // if( std::fmod(m_LEDTimer.Get().value(), 10) < 5 ){
            //     m_CANdle.SetLEDs(255,0,0);
            // }else{
            //     m_CANdle.SetLEDs(255,255,255);
            // }
            break;
        case(LEDConstants::LEDState::Error):
            // m_CANdle.SetLEDs(0, 255, 0);
            break;
        
        case(LEDConstants::LEDState::Off):
            m_CANdle.SetLEDs(0, 0, 255);
            break;
    }
}
