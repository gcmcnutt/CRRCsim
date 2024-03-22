/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004, 2005, 2006, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2004 Jan Edward Kansky
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

/*
 * Copyright (C) 2004  Jens Wilhelm Wulf, j dot w dot wulf at gmx dot net
 * Licensed under the GNU General Public License version 2.
 */

#include "lib_conversions.h"
#include <ctype.h>
#include <math.h>

#define DEBUG 0

#include <stdio.h>

std::string doubleToString(double dVal)
{
  char tmp[30];
  int  nStart = 0;

  sprintf(tmp, "%2.8g", dVal);
  while (tmp[nStart] == ' ')
    nStart++;

  return(&(tmp[nStart]));
}

std::string itoStr(long int nr, char fill, unsigned char digits, bool prettyneg)
{
  std::string  tmp = "";
  char    digit;
  bool    neg = false;
  int     Cnt;

  if (nr < 0)
  {
    neg = true;
    nr *= -1;
  }


  if (nr == 0)
  {
    for (int Cnt = 0; Cnt < digits-1; Cnt++) tmp += fill;
    tmp += '0';
  }
  else
  {
    Cnt = 0;
    while (Cnt < digits || nr > 0 || neg)
    {
      if (nr > 0)
        digit = (nr % 10)+48;
      else
      {
        if (neg)
        {
          if (prettyneg) digit = '-';
          else
          {
            tmp = '-' + tmp;
            digit = fill;
          }
          neg = false;
        }
        else digit = fill;
      }

      tmp = digit + tmp;
      nr /= 10;
      Cnt++;
    }
  }

  return(tmp);
}

long int mypow10(int exponent)
// ich will int als output, nicht double
{
  int tmp = 1;

  for (int Cnt=0; Cnt<exponent; Cnt++) tmp *= 10;

  return(tmp);
}

std::string ftoStr(double        nr,
                   unsigned char digitspre,
                   unsigned char digitspost,
                   bool          prettyneg,
                   bool          fComma)
{
  std::string   tmp;
  int      pre;
  int      post;
  bool     neg = false;


  if (nr < 0)
  {
    neg = true;
    nr *= -1;
  }

  pre = int(floor(nr));
  post = int(floor((nr - pre) * mypow10(digitspost)));

  tmp = itoStr(pre, ' ', digitspre);
  if (fComma)
    tmp += ",";
  else
    tmp += ".";
  tmp += itoStr(post, '0', digitspost);


  if (neg)
  {
    int pos;

    pos = tmp.length()-1;
    while ((tmp[pos] != ' ') && (pos > 0)) pos--;

    if (pos == 0 && tmp[pos] != ' ') tmp = ' ' + tmp; // nur für den Notfall

    if (prettyneg) tmp[pos] = '-';
    else tmp = tmp.substr(0, pos+1) + '-' + tmp.substr(pos+1, tmp.length()-pos);
  }

  return(tmp);
}


int hex2int(std::string hexIn)
{
  int tmp = 0;
  int wert1 = 1;
  int wert2;

  for (unsigned int n= 0; n< hexIn.length(); n++)
  {
    wert2 = hexIn[hexIn.length()-n-1];
    if (wert2 >=97 ) wert2-= 87;
    if (wert2 >=65 ) wert2-= 55;
    if (wert2 >= 48) wert2-= 48;
    tmp+= wert2 * wert1;
    wert1*= 16;
  }
  return(tmp);
}


std::string strU(std::string in) /*{{{*/
{
  std::string tmp = "";

  for (unsigned int uCnt = 0; uCnt < in.length(); uCnt++)
  {
    tmp += toupper(in[uCnt]);
  }
  return(tmp);
}
/*}}}*/

/**
 * Removes leading and trailing whitespace
 */
std::string trim(std::string in)
{
  int nL  = in.length();
  int nCnt;

  // Die Anzahl der führenden Zeichen ermitteln
  for (nCnt=0; nCnt < nL && isspace(in[nCnt]); nCnt++)
  {
  }

  // Die führenden Zeichen löschen
  in.replace(0, nCnt, "");

  nL = in.length();
  if (nL > 0)
  {
    // Die Anzahl der abschliessenden Zeichen ermitteln
    for (nCnt = nL-1; nCnt > 0 && isspace(in[nCnt]); nCnt--)
    {
    }
    // Die abschliessenden Zeichen löschen
    in.replace(nCnt+1, nL-nCnt, "");
  }

  return(in);
}

std::string itoHexStr(long unsigned int nr)
{
  std::string tmp = "";
  char        ch;

  while (nr > 0)
  {
    ch = 48 + (nr & 0x0F);
    if (ch > 57)
      ch += 7;
    tmp = ch + tmp;

    nr >>= 4;
  }

  return(tmp);
}

void float2MulShift(float fl, int& mul, int& shift)
{
  int          nMulBits = 15;
  long long    lMaxNum  = (1l << nMulBits)-1;
  long long    lDiv;

  // Dieses sollte auf jeden Fall möglich sein:
  mul   = (long long)(fl);
  shift = 0;

#if DEBUG == 1
  printf("lMaxNum = %i\n", lMaxNum);
#endif

  // Nun was gescheites berechnen:
  lDiv = (long long)(lMaxNum / fl);

#if DEBUG == 1
  printf("lDiv = %i\n", lDiv);
#endif

  while ((1l<<shift) < lDiv)
  {
    shift++;
  }
  shift--;

#if DEBUG == 1
  printf("shift = %i\n", shift);
#endif

  mul = (int)floor(  (fl * (1l<<shift)) +0.5 );

#if DEBUG == 1
  printf("mul = %i\n", mul);
#endif

  // Solange mul eine gerade Zahl ist kann man sie und den Schiebefaktor
  // halbieren ohne das Ergebnis ungenauer zu machen.
  while ((mul & 1) == 0 && shift > 0)
  {
    mul >>= 1;
    shift--;
  }
}
