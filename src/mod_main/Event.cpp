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
  
#include <iostream>

#include "Event.h"

Event::Event(EventGroup group, unsigned long type)
: ev_group(group), ev_type(type)
{
  #ifdef DEBUG_CLASS_EVENT
  std::cout << "Created group|type " << ev_group << "|";
  std::cout << ev_type << " event " << this << std::endl;
  #endif
}

Event::~Event()
{
  #ifdef DEBUG_CLASS_EVENT
  std::cout << "Destroyed group|type " << ev_group << "|";
  std::cout << ev_type << " event " << this << std::endl;
  #endif
}


