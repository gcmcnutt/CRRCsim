/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008-2009 Jan Reucker (original author)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */
  
#ifndef EVENT_H_
#define EVENT_H_

//#define DEBUG_CLASS_EVENT

/** Base class for events
 *
 *  All events are instances of classes that are derived
 *  from this base class. Common features for events:
 *
 *  - Each event belongs to an event group. Event receivers
 *    may choose only to receive events from certain groups.
 *  - Each event has a certain type, e.g. the group of
 *    Joystick events may contain JoystickButton events,
 *    JoystickMovement events and so on.
 */
class Event
{
  public:
    typedef enum
    {
      Generic     = 0x00000001UL,
      Input       = 0x00000002UL,
      FDM         = 0x00000004UL,
      Keyboard    = 0x00000008UL,
      Joystick    = 0x00000010UL,
      Mouse       = 0x00000020UL,
      Logging     = 0x00000040UL,
      All         = 0xFFFFFFFFUL
    } EventGroup;

    typedef enum
    {
      CrashEvent  = 0x00000001UL
    } GenericEvents;
    
    /// Create an event
    /// \param group    Event group
    /// \param type     Event type
    Event(EventGroup group, unsigned long type);
    
    /// Query the event's group
    /// \return Event group to which this event belongs
    EventGroup getGroup() const {return ev_group;};
    
    /// Query the event's type
    /// \return The event's group-specific type code
    unsigned long getType() const {return ev_type;};
    
    /// Destroy the event
    virtual ~Event();

  protected:
    EventGroup ev_group;      ///< Stores the event's group
    unsigned long ev_type;    ///< Stores the event's type
};


/**
 * A joystick event base class.
 */
class JoystickEvent : public Event
{
  public:
    /// Typedef for possible event types in the Joystick event group
    typedef enum
    {
      Button,
      Axis
    } Type;
    
    /// Create a JoystickEvent
    JoystickEvent(unsigned long type) 
    : Event(Event::Joystick, type)
    {
    }
};


/**
 * A joystick button event.
 *
 * This event is generated for every change to the state of a joystick
 * button.
 *
 * \todo This class is just an example.
 */
class JoystickButtonEvent : public JoystickEvent
{
  public:
    JoystickButtonEvent(unsigned long button, bool boPressed)
    : JoystickEvent(JoystickEvent::Button),
      m_Button(button), m_IsPressed(boPressed)
    {
    }
    
    /// Query the number of the button that caused the event
    /// \return button number
    unsigned long getButton() const {return m_Button;}
    
    /// Set the number of the button that caused the event
    /// \param ulButton   button number
    void setButton(unsigned long ulButton) {m_Button = ulButton;}
    
    /// Query the state of the button that caused the event
    /// \retval true    Button is pressed
    /// \retval false   Button is not pressed
    bool isPressed() const {return m_IsPressed;}
    
    /// Set the state of the button that caused the event
    void setPressed(bool boPressed) {m_IsPressed = boPressed;}
    
    /// Set button number and state
    /// \param button     button number
    /// \param boPressed  button state
    void set(unsigned long button, bool boPressed)
    {
      m_Button = button;
      m_IsPressed = boPressed;
    }
    
  private:
    unsigned long m_Button;       ///< button number
    bool          m_IsPressed;    ///< button state (true: pressed)
    
};


/** A log message event
 *
 *  This event should be used to notify the different
 *  logging subsystems to log a new string message.
 *
 *  The event group "logging" currently only contains
 *  one event, so the event type is always set to 0.
 */
class LogMessageEvent : public Event
{
  public:
    LogMessageEvent(std::string msg = "")
    : Event(Event::Logging, 0),
      m_Msg(msg)
    {
    }
    
    inline void set(std::string msg)
    {
      m_Msg = msg;
    }
    
    inline std::string get() const
    {
      return m_Msg;
    }
  
  private:
    std::string m_Msg;
};


/** Input value update event
 *
 *  This event is raised whenever the input device
 *  axis values have changed.
 */
class AxisUpdateEvent : public Event
{
  public:
    AxisUpdateEvent()
    : Event(Event::Input, 0)
    {
    }
    
    void set( const float& aileron, const float& elevator,
              const float& rudder,  const float& throttle,
              const float& flap,    const float& spoiler,
              const float& retract, const float& pitch)
    {
      m_aileron = aileron;
      m_elevator = elevator;
      m_rudder = rudder;
      m_throttle = throttle;
      m_flap = flap;
      m_spoiler = spoiler;
      m_retract = retract;
      m_pitch = pitch;
    }
    
    float getAileron() const  {return m_aileron;}
    float getElevator() const {return m_elevator;}
    float getRudder() const   {return m_rudder;}
    float getThrottle() const {return m_throttle;}
    float getFlap() const     {return m_flap;}
    float getSpoiler() const  {return m_spoiler;}
    float getRetract() const  {return m_retract;}
    float getPitch() const    {return m_pitch;}
    

  private:
    float m_aileron;    ///< aileron input,          -0.5 ... 0.5
    float m_elevator;   ///< elevator input,         -0.5 ... 0.5
    float m_rudder;     ///< rudder input,           -0.5 ... 0.5
    float m_throttle;   ///< throttle input,          0.0 ... 1.0
    float m_flap;       ///< flap input,              0.0 ... 1.0
    float m_spoiler;    ///< spoiler input,           0.0 ... 1.0
    float m_retract;    ///< retractable gear input,  0.0 ... 1.0
    float m_pitch;      ///< rotor/propeller pitch,  -0.5 ... 0.5

};


/** 
 * CrashEvent - emitted when the plane crashes
 */
class CrashEvent : public Event
{
  public:
    CrashEvent()
    : Event(Event::Generic, Event::CrashEvent)
    {
    }
  
};

#endif // EVENT_H_
