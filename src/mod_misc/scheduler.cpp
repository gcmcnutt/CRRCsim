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
  

#include <iostream>
#include "scheduler.h"
using namespace std;

Scheduler::Scheduler() {};
Scheduler::~Scheduler() {};

void Scheduler::Register( void *object)
{
  cout << "Scheduler::Register("<< object << ")" << endl;
  lObject.push_back( object);
}

void Scheduler::UnRegister( void *object)
{
  lObject.remove( object);
  cout << "Scheduler::UnRegister("<< object << ")" << endl;
}

void Scheduler::Run()
{
  list<void *>::iterator _it;
  // cout << "Scheduler::Run" << endl;
  for( _it= lObject.begin(); _it!= lObject.end(); _it++)
  {
    ((RunnableObject *)(*_it))-> Run();
  }
}

#ifdef TEST_SCHEDULER

class Test1: public RunnableObject
{
  public:
  Test1(Scheduler *_s) {
    cout << "registering "<< this  << endl;
    _s->Register(this);}
  void Run() {cout << "running " << this << endl;} 
};

int main (int argc, char ** argv)
{
  Scheduler s;
  Test1 t1(&s);
  Test1 t11(&s);
  s.Run();
}

#endif
