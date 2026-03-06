/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2008 Jan Reucker
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
  

// configure module mod_fdm
#ifndef MOD_FDM_CONFIG_H
#define MOD_FDM_CONFIG_H

#include <crrc_config.h>


#define MOD_FDM_USE_LARCSIM     1
#define MOD_FDM_USE_002         0   // should not be used anymore, since aero model is out-of-date
#define MOD_FDM_USE_DISPLAYMODE 1
#define MOD_FDM_USE_TESTMODE    1
#define MOD_FDM_USE_HELI01      1
#define MOD_FDM_USE_MCOPTER01   1

#define MOD_FDM_INFOSTR  "CRRCSim " PACKAGE_VERSION


#endif // MOD_FDM_CONFIG_H

