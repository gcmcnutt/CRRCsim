/**
 * Create thermal profile according to
 * 
 * "Das Thermikbuch für Modellflieger", written by Markus Lisken and
 * Ulf Gerber, Verlag für Technik und Handwerk, Baden-Baden.
 *
 * 
 * Jens Wilhelm Wulf
 */


#include <iostream>
#include <fstream>

#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif


double getY(double x)
{
  const double a0 = 0.4256;
  const double a1 = 0.5743;
  const double a2 = 2*M_PI;
  const double b0 = -0.0743;
  const double b1 =  0.0743;
  const double b2 = 2*M_PI;
  
  if (x < 0.5)
    return(a0 + a1*cos(a2*x));
  else
    return(b0 + b1*cos(b2*x));
}


int main()
{
  // size of table
  int    nBits = 6;
  
  /** There is nothing to edit below ! */
  
  double dStep = 1.0/(1<<nBits);
  double dRadiusCenter = 0.383;
  
  std::ofstream outfile;
  
  outfile.open("thermalprofile.h");
  outfile << "#ifndef THERMALPROFILE_H\n";
  outfile << "#define THERMALPROFILE_H\n";

  outfile   << "const double ThermalRadius       = " << dRadiusCenter << ";\n";
  std::cout << "# const double ThermalRadius       = " << dRadiusCenter << ";\n";
  
  outfile << "const int    ThermalProfile_bits = " << nBits << ";\n";
  outfile << "const double ThermalProfile[]    = {\n";

  double mx = 1.0;
  double my = 1.0;
  double v  = 0;
  double xa, ya;
  
  for (double x=0; x<1; x+= dStep)
  {
    double dTmp = getY(x);
    
    if (x > 0)
    {
      v += M_PI*(x*x - xa*xa) * 0.5*(dTmp+ya);
    }
                
    std::cout << (mx*x/dRadiusCenter) << " " << (my*dTmp) << "\n";
//    std::cout << (mx*x/dRadiusCenter) << " " << (v) << "\n";
    outfile << "   " << dTmp << ",\n";
        
    xa = x;
    ya = dTmp;
  }    
  
  // one more value to get a faster interpolation code
  outfile << "   0};\n";
  
  outfile << "#endif\n";  
}


