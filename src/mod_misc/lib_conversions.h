/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004, 2005, 2008 Jens Wilhelm Wulf (original author)
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

/*
 * Copyright (C) 2004  Jens Wilhelm Wulf, j dot w dot wulf at gmx dot net
 * Licensed under the GNU General Public License version 2.
 */

#ifndef JWW_CONVERSIONS_H
#define JWW_CONVERSIONS_H

#include <string>
#include <math.h>

/**
 * to make something computer readable
 */
std::string doubleToString(double dVal);

/**
 * to make something nice human readable
 */
std::string itoStr(long int nr, char fill, unsigned char digits, bool prettyneg = true);

/**
 * to make something nice human readable
 */
std::string ftoStr(double nr, unsigned char digitspre, unsigned char digitspost, bool prettyneg = true, bool fComma = false);

/**
 * Erwartet nur die hex-Ziffern im String, also kein führendes 0x oder so!
 */
int hex2int(std::string hexIn);
std::string strU(std::string in);
std::string trim(std::string in);
std::string itoHexStr(long unsigned int nr);

void float2MulShift(float fl, int& mul, int& shift);

#endif // JWW_CONVERSIONS_H
