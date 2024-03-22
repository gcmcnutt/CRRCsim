/**
 * Create thermal profile, characteristics of the profile can be adapted.
 * Writes <tt>thermalprofile.h</tt> and prints values to stdout, you can view
 * them using gnuplot or something else.
 *
 * 
 * Jens Wilhelm Wulf
 */


/**
 * Was habe ich mir dabei gedacht?
 * Für den inneren Teil wollte ich eine Funktion dritter Ordnung haben, damit
 * die folgenden Forderungen erfüllbar sind: 
 *   1. Steigung=0 bei x=0
 *   2. Nullstelle bei x=r
 *   3. Einstellbare Steigung bei x=r, damit es einen stetigen Übergang zur
 *      äußeren Funktion gibt.
 * 
 * Daher schreibe ich mal
 *    f_i(x) = (x-r)(x-p)(x+A)
 * Nun soll aber auch gelten: f_i(x=0) = 0. Daher braucht man einen Nenner und
 * erhält
 *    f_i(x) = (x-r)(x-p)(x+A) / (r*p*A)
 * Der Parameter r ist nicht frei, den er gibt das Ende des inneren Teils an.
 * Dann hat man noch zwei freie Parameter (p, A). Die erste Ableitung von 
 * f_i(x) ist
 *    f_i'(x) = [3x^2 + 2x(A-p-r) -pA -Ar +pr] / (pAr)
 * Sie soll bei x=0 null sein. Daraus ergibt sich
 *    0 = -pA -Ar +pr
 *    A = pr/(p+r)
 * Die erste Ableitung bei x=r ergibt sich damit zu
 *    f_i'(x=r) = [r^2 + Ar - pr - pA] / (pAr)
 *    f_i'(x=r) = [r^2 + pr - 2p^2] / (p^2 r)
 * Zunächst wird p mit p>r vorgegeben. Damit wird die Ableitung am Rand berechnet und das Volumen des 
 * inneren Teils.
 * 
 * Der äußere Teil muss folgende Bedingung erfüllen:
 *   1. Funktionswert=0 bei x_e=1-r
 *   2. Steigung=0      bei x_e
 *   3. Funktionswert=0 bei x=0
 *   4. Eine bestimmte Steigung bei x=0. Die Steigung wird durch die innere Funktion vorgegeben.
 *   5. Sie muss in Y-Richtung skalierbar sein.
 * Damit kommt man schnell zu der Grundfunktion
 *    f_a(x) = m x (x-x_e)^2 (x+c)
 * Wobei man c noch so bestimmen muss, dass die Steigung bei x=0 den Wert s hat. Die Ableitung ist
 *    f_a'(x) = m * [4x^3 + 3x^2(c-2x_e) +2x(x^2+2 x_e c) + (x_e)^2 c]
 * Somit muss gelten
 *    c = s / [m (x_e)^2]
 * 
 * 
 * Nun würde ich gerne auch ein Profil machen, was dem der alten Simulation etwas näher kommt.
 * Wenn das mit einfachen Formeln zu beschreiben sein soll, muss ich mir den stetigen 
 * Steigungsverlauf an der Trennstelle abschminken.
 * In der Mitte benutze ich einfach x^10. Es gelten die Bedingungen:
 *   1. Steigung=0 bei x=0
 *   2. Nullstelle bei x=r
 * Daraus ergibt sich 
 *   f_i(x)    = 1 - (x^10 / r^10)
 *   f_i'(x)   = -10/r^10  x^9
 *   f_i'(x=r) = -10/r
 */

#include <iostream>
#include <fstream>

#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif


// Define PROFTYPE to 0 for the original new style thermal.
// Define PROFTYPE to 1 for something which is similar to an old style thermal.
#define PROFTYPE 0


/**
 * compute formula for inner part of thermal
 */
double getVelocityCenter(double dDist,
                         double dRadiusCenter,
                         double dPar)
{
#if (PROFTYPE == 0)  
  double A;

  A = dRadiusCenter * dPar / (dRadiusCenter + dPar);
  
  return(
         (dDist-dRadiusCenter) * (dDist-dPar) * (dDist + A)
         /
         (dRadiusCenter*dPar*A)
         );
#endif
#if (PROFTYPE == 1)
  return(1 - (pow(dDist, 10)/pow(dRadiusCenter, 10)) );
#endif
}

/**
 * compute first derivation of inner part at its end
 * (dDist = dRadiusCenter)
 */
double getCenterDer(double dRadiusCenter,
                    double dPar)
{
#if (PROFTYPE == 0)  
  return(
         (dRadiusCenter*dRadiusCenter + dPar*dRadiusCenter - 2*dPar*dPar)
         /
         (dPar*dPar*dRadiusCenter)
         );
#endif
#if (PROFTYPE == 1)
  return(-10/dRadiusCenter);
#endif
}

/**
 * compute formula for outer part of thermal
 */
double getVelocityOuter(double dDist,
                        double dRadiusCenter,
                        double dMul,
                        double dDer)
{
  double dXe = 1 - dRadiusCenter;
  double dX  = dDist - dRadiusCenter;
  
  return(
         dMul * dX * (dX - dXe) * (dX - dXe) * (dX + 
                                                ( dDer / (dMul*dXe*dXe) )
                                                )
         );
}

int main()
{
  /** Adjust to your needs: */
  
  // size of center:
  double dRadiusCenter = 0.35;
  // affects transition (algorithm may need to adjust this value)
  double dPar          = dRadiusCenter * 10; // multiplier has to be >= 1
  // size of table
  int    nBits = 6;
  
  /** There is nothing to edit below ! */
  
  double dStep = 1.0/(1<<nBits);
  double dDer;
  double dMul;
    
  dDer = getCenterDer(dRadiusCenter, dPar);
  
  // Calculate volume of inner part
  double dVI = 0;
  for (double x=0; x<dRadiusCenter; x+= dStep)
  {
    double dY = getVelocityCenter(x, dRadiusCenter, dPar);
    dVI += M_PI * dY * (2*x*dStep + dStep*dStep);
  }
  
  // Adjust dMul to get inner volume = outer volume
  dMul = -1;
  double dVO;
  do
  {
    std::cerr << "Steigung = " << dDer << "\n";
    
    // Calculate volume of outer part
    dVO = 0;
    for (double x=dRadiusCenter; x<1; x+= dStep)
    {
      double dY = getVelocityOuter(x, dRadiusCenter, dMul, dDer);
      dVO -= M_PI * dY * (2*x*dStep + dStep*dStep);
    }
    dMul = dMul * dVI/dVO;
    
    /*
    std::cerr << "inner volume: " << dVI << " "  
      << "dMul = " << dMul << " outer volume = " << dVO << "\n";
     */
    
    if (fabs(dMul) < 1.0E-20)
    {
      std::cerr << "...had to adjust...\n";
#if (PROFTYPE == 0)  
      dPar *= 0.8;
      if (dPar < dRadiusCenter)
        dPar = dRadiusCenter;
      dDer = getCenterDer(dRadiusCenter, dPar);
#endif      
#if (PROFTYPE == 1)
      dDer = dDer / 2;
#endif
      dMul = -1;
    }
  }
  while (fabs((dVI - dVO)/dVI) > 0.01);

  
  std::ofstream outfile;
  
  outfile.open("thermalprofile.h");
  outfile << "#ifndef THERMALPROFILE_H\n";
  outfile << "#define THERMALPROFILE_H\n";

  outfile    << "// PROFTYPE = " << (int)(PROFTYPE) << "\n";
  std::cout  << "# PROFTYPE = " << (int)(PROFTYPE) << "\n";
  
  outfile   << "const double ThermalRadius       = " << dRadiusCenter << ";\n";
  std::cout << "# const double ThermalRadius       = " << dRadiusCenter << ";\n";
  
  outfile << "const int    ThermalProfile_bits = " << nBits << ";\n";
  outfile << "const double ThermalProfile[]    = {\n";

  double mx = 1;
  double my = 1;
  
//  double mx = 2.3;
//  double my = 1.5;
//  double mx = 1.5;
//  double my = 2.5;

  
  for (double x=0; x<dRadiusCenter; x+= dStep)
  {
    double dTmp = getVelocityCenter(x, dRadiusCenter, dPar);
    std::cout << (mx*x/dRadiusCenter) << " " << (my*dTmp) << "\n";
    outfile << "   " << dTmp << ",\n";
  }    
  for (double x=dRadiusCenter; x<1; x+= dStep)
  {
    double dTmp = getVelocityOuter(x, dRadiusCenter, dMul, dDer);
    std::cout << (mx*x/dRadiusCenter) << " " << (my*dTmp) << "\n";
    outfile << "   " << dTmp << ",\n";
  }
  
  // one more value to get a faster interpolation code
  outfile << "   0};\n";
  
  outfile << "#endif\n";  
}


