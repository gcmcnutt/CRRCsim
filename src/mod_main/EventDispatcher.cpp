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
  
#include "EventDispatcher.h"

#include <algorithm>
#include <iostream>

//#define DEBUG_EVENT_DISPATCHER

EventDispatcher*  EventDispatcher::getInstance()
{ 
  static EventDispatcher instance; 
  return &instance; 
} 

EventListener::EventListener(unsigned long ulGroupMask)
{
  EventDispatcher::getInstance()->registerListener(this, ulGroupMask);
}

EventListener::~EventListener()
{
  EventDispatcher::getInstance()->unregisterListener(this);
}


bool EventDispatcher::registerListener(EventListener* Listener, unsigned long ulGroups)
{
  bool boSuccess = false;

  ListenerQueue::const_iterator foundListener;
  for (foundListener = Listeners.begin(); foundListener != Listeners.end(); foundListener++)
  {
    if ((*foundListener).theListener == Listener)
    {
      break;
    }
  }

  if (foundListener == Listeners.end())
  {
    T_ListenerContainer LC;
    LC.theListener = Listener;
    LC.ulGroupMask = ulGroups;
    Listeners.push_back(LC);
    boSuccess = true;
  }
  
  #ifdef DEBUG_EVENT_DISPATCHER
  std::cout << "EventDispatcher::registerListener(" << Listener;
  std::cout << ", 0x" << std::hex << ulGroups << std::dec << ") ";
  std::cout << (boSuccess ? "ok" : "FAILED") << std::endl;
  #endif

  return boSuccess;
}

bool EventDispatcher::unregisterListener(EventListener* Listener)
{
  bool boSuccess = false;
  
  ListenerQueue::iterator foundListener;
  for (foundListener = Listeners.begin(); foundListener != Listeners.end(); foundListener++)
  {
    if ((*foundListener).theListener == Listener)
    {
      break;
    }
  }

  if (foundListener != Listeners.end())
  {
    Listeners.erase(foundListener);
    boSuccess = true;
  }

  #ifdef DEBUG_EVENT_DISPATCHER
  std::cout << "EventDispatcher::unregisterListener(" << Listener;
  std::cout << ") " << (boSuccess ? "ok" : "FAILED") << std::endl;
  #endif

  return boSuccess;
}

void EventDispatcher::raise(const Event* ev)
{
  #ifdef DEBUG_EVENT_DISPATCHER
  std::cout << "EventDispatcher::raise: event " << ev << ", type 0x";
  std::cout << std::hex << ev->getGroup() << std::dec << std::endl;
  #endif

  ListenerQueue::const_iterator allListeners = Listeners.begin();
  while (allListeners != Listeners.end())
  {
    if (((*allListeners).ulGroupMask & ev->getGroup()) != 0)
    {
      (*(*allListeners).theListener)(ev);
    }
    allListeners++;
  }
}
