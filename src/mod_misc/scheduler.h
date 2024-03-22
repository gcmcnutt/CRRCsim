/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008 Olivier Bordes (original author)
 * Copyright (C) 2008 Jan Reucker
 * Copyright (C) 2008 Jens Wilhelm Wulf
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
 
// scheduler
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <list>
using namespace std;

class Scheduler 
{
  public:
    Scheduler();
    ~Scheduler();
    void Register( void *);
    void UnRegister( void *);
    void Run();

  private:
    list< void *> lObject;
};

class RunnableObject
{
  public:
    // there should always be a virtual default constructor
    virtual ~RunnableObject() {}
    virtual void Run()=0;
    Scheduler *myScheduler;
};

#endif

