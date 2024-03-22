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
  
/**
 *  \file EventDispatcher.h
 *
 *  \brief Declaration of the EventListener and EventDispatcher classes
 *
 *  This file declares the classes for distributing and receiving events.
 *  The EventDispatcher is a singleton that receives events from different
 *  sources and distributes them to all registered EventListeners.
 */

#ifndef EVENT_DISPATCHER_H_
#define EVENT_DISPATCHER_H_

#include <deque>
#include <iostream>

#include "Event.h"


/**
 *  \brief Abstract base class for event receivers
 *
 *  This base class declares the common interface for all event
 *  receivers and handles the (de-)registration with the central
 *  EventDispatcher.
 *
 *  There are two approaches for creating a concrete receiver:
 *  1.) derive directly from this class and implement 
 *      void operator()(const Event*) which is then called by the
 *      EventDispatcher
 *  2.) use the template class EventAdapter if deriving directly
 *      from EventListener or implementing void operator()(const Event*)
 *      isn't feasible
 */
class EventListener
{
  public:
    /// Create a listener and register it with the EventDispatcher
    EventListener(unsigned long ulGroupMask);
    /// Unregister and destroy a listener
    virtual ~EventListener();
    /// This is the actual callback function that is called if
    /// an event is received.
    virtual void operator()(const Event* ev) = 0;
};


/**
 *  \brief Template for building EventListener adapter classes
 *
 *  This template creates a class acting as an adapter to the EventListener
 *  hierarchy. Use it if direct inheritance from EventListener or
 *  implementing void operator()(const Event*) is not feasible.
 *
 *  Basic usage:
 *  - Add a callback method to your class that will be called for
 *    each incoming event and receives a const pointer to the event.
 *  - Add a member of type EventAdapter<myClass> to your class.
 *  - Initialize the EventAdapter with a pointer to the current instance
 *    of your class, the name of the callback method, a pointer to the
 *    event dispatcher and maybe an event mask if you don't want to receive
 *    all events.
 *
 *  This should result in some code similar to this example:
 *
 *    \code
 *    #include <iostream>
 *    #include "EventDispatcher.h"
 *
 *    class FooEventReceiver
 *    {
 *      public:
 *        FooEventReceiver(unsigned long ulGroupMask = Event::Group::All);
 *        void myCallback(const Event* ev);
 *
 *      private:
 *        EventAdapter<FooEventReceiver> m_EventAdapter;
 *    };
 *
 *    FooEventReceiver::FooEventReceiver(unsigned long ulGroupMask)
 *    : m_EventAdapter(this, &FooEventReceiver::myCallback, ulGroupMask)
 *    {
 *    }
 *
 *    void FooEventReceiver::myCallback(const Event* ev)
 *    {
 *      std::cout << "Received an event!" << std::endl;
 *    }
 *    \endcode
 */
template < class Class >
class EventAdapter : public EventListener
{
   public:
    typedef void (Class::*CallBackMethod)(const Event* ev);

    EventAdapter(Class* instance, CallBackMethod method,
                 unsigned long ulGroupMask = Event::All)
    : EventListener(ulGroupMask)
    {
       instance_  = instance;
       callback_  = method;
    };

    void operator()(const Event* ev)
    {
       return (instance_->*callback_)(ev);
    };

   private:
    Class*          instance_;
    CallBackMethod  callback_;
};



/**
 *  \brief The EventDispatcher singleton class.
 *
 *  This class represents the central event dispatcher. Event sources
 *  will pass their events to this class. EventListeners will
 *  register with the EventDispatcher in order to receive these events.
 *  
 *  The EventDispatcher is implemented as a singleton, so there's only
 *  one single instance that will automatically be initialized on
 *  first use. It provides a static method to access the single
 *  instance.
 */
class EventDispatcher
{
  private:
    typedef struct tagListenerContainer
    {
      EventListener*      theListener;
      unsigned long       ulGroupMask;
    } T_ListenerContainer;
    typedef std::deque< T_ListenerContainer > ListenerQueue;
    ListenerQueue Listeners;
    
    /// Default constructor, not accessible
    EventDispatcher(){};
    /// Copy constructor, not accessible
    EventDispatcher(EventDispatcher const&){};

  public:
    /// Create the only instance of this object and return a pointer to it
    static EventDispatcher* getInstance(); 

    bool registerListener(EventListener *Listener, unsigned long ulGroups = Event::All);
    bool unregisterListener(EventListener *Listener);
    void raise(const Event* ev);
};

#endif // EVENT_DISPATCHER_H_
