// -*- mode: c; mode: fold -*-
/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2006, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2006, 2007, 2008 - Jan Reucker
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
//
#include "airtoxml.h"

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <limits>
#include <cstdlib>
#include <cstring>

#include "../../mod_fdm_config.h"
#include "../../mod_misc/lib_conversions.h"
#include "../../mod_misc/SimpleXMLTransfer.h"
#include "../../mod_misc/filesystools.h"
#include "../../mod_math/vector3.h"
#include "../../mod_math/matrix33.h"
#include "../power/power.h"
# include "../../mod_misc/ls_constants.h"

# include "../ls_types.h"

/*****************************************************************************/

/** \brief Read a string parameter from an .air file.
 *
 *  \param datafile file name
 *  \param param    parameter name
 *  \return parameter value as a std::string
 */
std::string getParamAsString(const char *datafile, const char *param)
{
  std::ifstream airdata(datafile, std::ios::in);
  std::string par = "";
  std::string value = "";

  if (!airdata.is_open())
  {
    std::cerr << "Unable to open airplane specification file ";
    std::cerr << datafile << std::endl;
  }
  else
  {
    bool found = false;
    while ((airdata >> par) && !found)
    {
      if (par == param)
      {
        airdata >> value;
        found = true;
      }
      // skip to end of line
      airdata.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    airdata.close();
  }
  return value;
}

/** \brief Read a long parameter from an .air file.
 *
 *  Reads a parameter from an .air file and places the parameter's
 *  value in the variable pointed to by <code>value</code>.
 *
 *  If the parameter name is found in the file, the returned
 *  status will always be <code>true</code>, even if no valid
 *  value was found behind the parameter name. In this case,
 *  the content of <code>value</code> will be undefined.
 *
 *  \param datafile file name
 *  \param param    parameter name
 *  \param value    parameter value as read from the file
 *  \return true if the parameter was found in the file, false otherwise
 */
bool getParamAsLong(const char *datafile, const char *param, long *value)
{
  bool ret = false;
  std::string stringval = getParamAsString(datafile, param);
  
  if (stringval != "")
  {
    *value = strtol(stringval.c_str(), NULL, 10);
    ret = true;
  }
  
  return ret;
}


/*****************************************************************************/
/**
 * This class is a stripped-down copy of CRRC_AirplaneSim_Larcsim,
 * which does only include things needed to load an .air-file.
 * There is a small number of additions.
 * 
 * @author Jens Wilhelm Wulf
 */
class DotAirLoader
{
   
  public:
   
   /**
    * read from file
    */
   DotAirLoader(const char* filename);

   virtual ~DotAirLoader();
         
   /// @name Aerodynamic data
   //@{
   SCALAR  C_ref;    //  reference chord (ft)
   SCALAR  B_ref;    //  reference span  (ft)
   SCALAR  S_ref;    //  reference area (ft^2)
   SCALAR  U_ref;    //  reference speed for Re-scaling of CD_prof  (ft/s)
   
   SCALAR  Alpha_0;  //  baseline alpha (rad)   
   SCALAR  Cm_0   ;  //  baseline Cm at Alpha_0
   SCALAR  CL_0   ;  //  baseline CL at Alpha_0
   
   SCALAR  CL_max ;  //  positive stall limit
   SCALAR  CL_min ;  //  negative stall limit
   SCALAR  CD_prof;  //  profile CD at U_ref
   
   SCALAR  Uexp_CD;  //  for Re-scaling of CD_prof  ~ (U/U_ref)^Uexp_CD
   
   SCALAR  CL_a;     // lift-force   / alpha    ~  2 pi / (1 + 2/AR)
   SCALAR  Cm_a;     // pitch-moment / alpha    (pitch stability)
   SCALAR  CY_b;     // side-force  / sideslip
   SCALAR  Cl_b;     // roll-moment / sideslip (crucial for rudder-only turns)   
   SCALAR  Cn_b;     //  yaw-moment  / sideslip (yaw stability)
   
   SCALAR  CL_q;     //  lift-force   / pitch-rate
   SCALAR  Cm_q;     //  pitch-moment / pitch-rate  (pitch damping)
   SCALAR  CY_p;     //  side-force  / roll-rate
   SCALAR  Cl_p;     //  roll-moment / roll-rate   (roll damping)
   SCALAR  Cn_p;     //  yaw-moment  / roll-rate   (yaw-roll coupling)   
   SCALAR  CY_r;     //  side-force  / yaw-rate
   SCALAR  Cl_r;     //  roll-moment / yaw-rate
   SCALAR  Cn_r;     //  yaw-moment  / yaw-rate  (yaw damping)
   
   SCALAR  CL_de;    //  lift-force   / elevator
   SCALAR  Cm_de;    //  pitch-moment / elevator
   SCALAR  CY_dr;    // side-force  / rudder
   SCALAR  Cl_dr;    // roll-moment / rudder
   SCALAR  Cn_dr;    // yaw-moment  / rudder
   SCALAR  CY_da;    // side-force  / aileron
   SCALAR  Cl_da;    // roll-moment / aileron
   SCALAR  Cn_da;    // yaw-moment  / aileron
   
   SCALAR eta_loc;
   SCALAR CG_arm;
   SCALAR CL_drop;
   SCALAR CD_stall;
   
   SCALAR span_eff;  // span efficiency: Effective span  0.95 for most planes, 0.85 flying wing
   SCALAR CL_CD0;    // CL at minimum profile CD: 0.30 for 7037, 0.15 MH32, 0.0 RG15, AGxx, power
   SCALAR CD_CLsq;   // d(CD)/d(CL^2),  curvature of parabolic profile polar: 0.01 composites, 0.015 saggy ships, 0.02 beat up ship
   SCALAR CD_AIsq;   // d(CD)/d(aileron^2) , curvature of ail. CD influence: 0.01/(max_aileron)^2
   SCALAR CD_ELsq;   // d(CD)/d(elevator^2), curvature of ele. CD influence: 0.01/(max_elevator)^2 for Zagi otherwise 0
   //@}

   /// @name Mass and inertia
   //@{
   SCALAR Mass;  // inertia
   SCALAR I_xx;  // inertia
   SCALAR I_yy;  // inertia
   SCALAR I_zz;  // inertia
   SCALAR I_xz;  // inertia
   //@}
      
   /// @name Gear and ground interaction
   //@{

   /**
    * This struct holds information about a hard point/wheel on the airplane.
    * todo: steering is unused
    */
   typedef struct
   {
     /**
      * body axes: x,y,z
      */
     CRRCMath::Vector3 v_P;
     
     double            spring_constant;
     double            spring_damping;
     
     int    steering_mapping;  // Indicates which RC channel controls the steering
     int    steering_max_angle;  // Indicates maximum anglee of steering wheel
     double percent_brake;
     double caster_angle_rad;          
   } Wheel;
   
   /**
    * Vector containing all hard points/wheels
    */
   std::vector<Wheel> wheels;
         
   //@}
      
   /// @name written by constructor
   //@{
   
   /**
    * Propulsion system: batteries, shafts, engines, propellers.
    */
   Power::Power* power;
   
   /**
    * Longest distance of any of the airplanes points to the cg
    */
   double dMaxSize;
   
   /**
    * Velocity in trimmed flight; dead air.
    */
   float trimmedFlightVelocity;
   
   //@}
   
   /// @name Misc
   //@{
   std::string model;
   std::string engine_sound;
   int         engine_type;
   double      engine_sound_pitchfactor;
   double      max_thrust;
   SimpleXMLTransfer* power_descr;
   //@}
      
};

void Check_fscanf(int nRead, int nExp)
{
  if (nRead < nExp)
  {
    std::string msg = "Error reading airplane specification file ";
    throw std::runtime_error(msg);
  }
}

DotAirLoader::DotAirLoader(const char* filename) /*{{{*/
{
  FILE *airplane_data;
  char parameter_name[256]; //  Name of parameter
  char trailing_comment[256];
  double parameter_value;
  bool fSectionParsingDone;
  int loop;

  /**
   * Tracks wingspan [m]
   */
  double span = 0;

  power = (Power::Power*)0;
  
  C_ref = 0;
  B_ref = 0;
  S_ref = 0;
  U_ref = 0;

  Alpha_0 = 0;

  Cm_0 = 0;
  CL_0 = 0;
  CL_max = 0;
  CL_min = 0;
  CD_prof = 0;

  Uexp_CD = 0;

  CL_a = 0;
  Cm_a = 0;
  CY_b = 0;
  Cl_b = 0;
  Cn_b = 0;

  CL_q = 0;
  Cm_q = 0;
  CY_p = 0;
  Cl_p = 0;
  Cn_p = 0;
  CY_r = 0;
  Cl_r = 0;
  Cn_r = 0;

  CL_de = 0;
  Cm_de = 0;
  CY_dr = 0;
  Cl_dr = 0;
  Cn_dr = 0;
  CY_da = 0;
  Cl_da = 0;
  Cn_da = 0;

  eta_loc = 0;
  CG_arm = 0;
  CL_drop = 0;
  CD_stall = 0;

  span_eff=1.0; // Effective span  0.95 for most planes, 0.85 flying wing
  CL_CD0=0.0; // 0.30 for 7037, 0.15 MH32, 0.0 RG15, AGxx, power
  CD_CLsq=0.0; // 0.01 composites, 0.015 saggy ships, 0.02 beat up ship
  CD_AIsq=0.0; // 0.01/(max_aileron)^2
  CD_ELsq=0.0; // 0.01/(max_elevator)^2 for Zagi otherwise 0

  Mass = 0;
  I_xx = 0;
  I_yy = 0;
  I_zz = 0;
  I_xz = 0;

  max_thrust = 0;

  engine_sound_pitchfactor=0;
  engine_type=0;
  
  airplane_data = fopen(filename, "r");
  if (airplane_data == NULL)
  {
    std::string msg = "Unable to open airplane specification file ";
    msg += filename;

    throw std::runtime_error(msg);
  }
  else
  {
    while (fscanf(airplane_data,"%s %lf %[^\n]",(char *)parameter_name,
                  &parameter_value,trailing_comment)!=EOF)
    {
      int nRead;
      
      if (strcasecmp(parameter_name,"Mass")==0)
        Mass=parameter_value;
      else if (strcasecmp(parameter_name,"I_xx")==0)
        I_xx=parameter_value;
      else if (strcasecmp(parameter_name,"I_yy")==0)
        I_yy=parameter_value;
      else if (strcasecmp(parameter_name,"I_zz")==0)
        I_zz=parameter_value;
      else if (strcasecmp(parameter_name,"I_xz")==0)
        I_xz=parameter_value;
      else if (strcasecmp(parameter_name,"C_ref")==0)
        C_ref=parameter_value;
      else if (strcasecmp(parameter_name,"B_ref")==0)
        B_ref=parameter_value;
      else if (strcasecmp(parameter_name,"S_ref")==0)
        S_ref=parameter_value;
      else if (strcasecmp(parameter_name,"U_ref")==0)
        U_ref=parameter_value;
      else if (strcasecmp(parameter_name,"Alpha_0")==0)
        Alpha_0=parameter_value;
      else if (strcasecmp(parameter_name,"Cm_0")==0)
        Cm_0=parameter_value;
      else if (strcasecmp(parameter_name,"CL_0")==0)
        CL_0=parameter_value;
      else if (strcasecmp(parameter_name,"CL_max")==0)
        CL_max=parameter_value;
      else if (strcasecmp(parameter_name,"CL_min")==0)
        CL_min=parameter_value;
      else if (strcasecmp(parameter_name,"CD_prof")==0)
        CD_prof=parameter_value;
      else if (strcasecmp(parameter_name,"Uexp_CD")==0)
        Uexp_CD=parameter_value;
      else if (strcasecmp(parameter_name,"CL_a")==0)
        CL_a=parameter_value;
      else if (strcasecmp(parameter_name,"Cm_a")==0)
        Cm_a=parameter_value;
      else if (strcasecmp(parameter_name,"CY_b")==0)
        CY_b=parameter_value;
      else if (strcasecmp(parameter_name,"Cl_b")==0)
        Cl_b=parameter_value;
      else if (strcasecmp(parameter_name,"Cn_b")==0)
        Cn_b=parameter_value;
      else if (strcasecmp(parameter_name,"CL_q")==0)
        CL_q=parameter_value;
      else if (strcasecmp(parameter_name,"Cm_q")==0)
        Cm_q=parameter_value;
      else if (strcasecmp(parameter_name,"CY_p")==0)
        CY_p=parameter_value;
      else if (strcasecmp(parameter_name,"Cl_p")==0)
        Cl_p=parameter_value;
      else if (strcasecmp(parameter_name,"Cn_p")==0)
        Cn_p=parameter_value;
      else if (strcasecmp(parameter_name,"CY_r")==0)
        CY_r=parameter_value;
      else if (strcasecmp(parameter_name,"Cl_r")==0)
        Cl_r=parameter_value;
      else if (strcasecmp(parameter_name,"Cn_r")==0)
        Cn_r=parameter_value;
      else if (strcasecmp(parameter_name,"CL_de")==0)
        CL_de=parameter_value;
      else if (strcasecmp(parameter_name,"Cm_de")==0)
        Cm_de=parameter_value;
      else if (strcasecmp(parameter_name,"CY_dr")==0)
        CY_dr=parameter_value;
      else if (strcasecmp(parameter_name,"Cl_dr")==0)
        Cl_dr=parameter_value;
      else if (strcasecmp(parameter_name,"Cn_dr")==0)
        Cn_dr=parameter_value;
      else if (strcasecmp(parameter_name,"CY_da")==0)
        CY_da=parameter_value;
      else if (strcasecmp(parameter_name,"Cl_da")==0)
        Cl_da=parameter_value;
      else if (strcasecmp(parameter_name,"Cn_da")==0)
        Cn_da=parameter_value;
      else if (strcasecmp(parameter_name,"eta_loc")==0)
        eta_loc=parameter_value;
      else if (strcasecmp(parameter_name,"span_eff")==0)
        span_eff=parameter_value;
      else if (strcasecmp(parameter_name,"CL_CD0")==0)
        CL_CD0=parameter_value;
      else if (strcasecmp(parameter_name,"CD_CLsq")==0)
        CD_CLsq=parameter_value;
      else if (strcasecmp(parameter_name,"CD_AIsq")==0)
        CD_AIsq=parameter_value;
      else if (strcasecmp(parameter_name,"CD_ELsq")==0)
        CD_ELsq=parameter_value;
      else if (strcasecmp(parameter_name,"CG_arm")==0)
        CG_arm=parameter_value;
      else if (strcasecmp(parameter_name,"CL_drop")==0)
        CL_drop=parameter_value;
      else if (strcasecmp(parameter_name,"CD_stall")==0)
        CD_stall=parameter_value;
      else if (strcasecmp(parameter_name,"max_thrust")==0)
        max_thrust=parameter_value;
      else if (strcasecmp(parameter_name,"engine_sound_pitchfactor")==0)
        engine_sound_pitchfactor=parameter_value;
      else if (strcasecmp(parameter_name,"engine_type")==0)
        engine_type=(int)(floor(parameter_value+0.5));
      else if (strcasecmp(parameter_name,"gear")==0)
      {
        bool  done_parsing_gear;
        float p1,p2,p3;      // Temporary variables used to read in file
        float dist;
        int   num_wheels;
        Wheel wheel;

        printf("In gear\n");

        // let's assume that there is nothing distant from the CG:
        dMaxSize = 0;

        num_wheels=(int)parameter_value;
        wheels.clear();
        for (loop=0; loop<num_wheels; loop++)
          wheels.push_back(wheel);
        
        done_parsing_gear=false;
        while(!done_parsing_gear)
        {
          nRead = fscanf(airplane_data,"%s %[^\n]",(char *)parameter_name,
                         trailing_comment);
          Check_fscanf(nRead, 1);
          
          if (strcasecmp(parameter_name,"locations")==0)
          {
            for (loop=0;loop<num_wheels;loop++)
            {
              nRead = fscanf(airplane_data,"%f %f %f",&p1,&p2,&p3);
              Check_fscanf(nRead, 3);
              wheels[loop].v_P = CRRCMath::Vector3((double)p1, (double)p2, (double)p3);
              
              // track wingspan
              if (span < (SCALAR)p2)
                span = (SCALAR)p2;
              // far away (Z distance is assumed to be low)?
              dist = p1*p1+p2*p2;
              if (dist > dMaxSize)
                dMaxSize = dist;
            }
            dMaxSize = sqrt(dMaxSize);
            span     = 2*span*FT_TO_M;
          }
          else if (strcasecmp(parameter_name,"springiness")==0)
          {
            for (loop=0;loop<num_wheels;loop++)
            {
              nRead = fscanf(airplane_data,"%f",&p1);
              Check_fscanf(nRead, 1);
              wheels[loop].spring_constant = (SCALAR)p1;
            }
          }
          else if (strcasecmp(parameter_name,"damping")==0)
          {
            for (loop=0;loop<num_wheels;loop++)
            {
              nRead = fscanf(airplane_data,"%f",&p1);
              Check_fscanf(nRead, 1);
              wheels[loop].spring_damping=(SCALAR)p1;
            }
          }
          else if (strcasecmp(parameter_name,"braking")==0)
          {
            for (loop=0;loop<num_wheels;loop++)
            {
              nRead = fscanf(airplane_data,"%f",&p1);
              Check_fscanf(nRead, 1);
              wheels[loop].percent_brake=(SCALAR)p1;
            }
          }
          else if (strcasecmp(parameter_name,"steerability")==0)
          {
            for (loop=0;loop<num_wheels;loop++)
            {
              nRead = fscanf(airplane_data,"%f %f",&p1,&p2);
              Check_fscanf(nRead, 2);
              wheels[loop].steering_mapping   = (int)p1;
              wheels[loop].steering_max_angle = (int)p2;
              wheels[loop].caster_angle_rad   = 0;
            }
          }
          else if (strcasecmp(parameter_name,"end_gear")==0)
          {
            done_parsing_gear=true;
          }
        }
      }
      else if (strcasecmp(parameter_name,"triangles")==0)
      {
        fSectionParsingDone = false;
        while (!fSectionParsingDone)
        {
          nRead = fscanf(airplane_data,"%s %[^\n]",(char *)parameter_name,
                         trailing_comment);
          Check_fscanf(nRead, 1);
          fSectionParsingDone = (strcasecmp(parameter_name,"end_triangles")==0);
        }
      }
      else if (strcasecmp(parameter_name,"sphere")==0)
      {
        fSectionParsingDone = false;
        while (!fSectionParsingDone)
        {
          nRead = fscanf(airplane_data,"%s %[^\n]",(char *)parameter_name,
                         trailing_comment);
          Check_fscanf(nRead, 1);
          fSectionParsingDone = (strcasecmp(parameter_name,"end_sphere")==0);
        }
      }
      else if (strcasecmp(parameter_name,"cylinder")==0)
      {
        fSectionParsingDone = false;
        while (!fSectionParsingDone)
        {
          nRead = fscanf(airplane_data,"%s %[^\n]",(char *)parameter_name,
                         trailing_comment);
          Check_fscanf(nRead, 1);
          fSectionParsingDone = (strcasecmp(parameter_name,"end_cylinder")==0);
        }
      }
      else if (strcasecmp(parameter_name,"extrusion")==0)
      {
        fSectionParsingDone = false;
        while (!fSectionParsingDone)
        {
          nRead = fscanf(airplane_data,"%s %[^\n]",(char *)parameter_name,
                         trailing_comment);
          Check_fscanf(nRead, 1);
          fSectionParsingDone = (strcasecmp(parameter_name,"end_extrusion")==0);
        }
      }
    }
    fclose(airplane_data);

    {
      // crrc_aero says: Cm = Cm_0 + Cm_a*(Alpha-Alpha_0)
      // So Cm is zero at
      float alpha = ((Cm_a * Alpha_0) - Cm_0 ) / Cm_a;
      // crrc_aero says: CL  = CL_0 + CL_a*(Alpha - Alpha_0);
      float cl = CL_0 + CL_a * (alpha - Alpha_0);

      // sanity check: currently this method does not really work for biplane2.air, as
      // a negative cl is computed. ???
      if (cl < 0.2)
      {
        cl = 0.2;
        printf("airplane: Cm_a=%f   Alpha_0=%f  Cm_0=%f\n", Cm_a, Alpha_0, Cm_0);
        printf("airplane: CL_a=%f               CL_0=%f\n", CL_a,          CL_0);
        printf("airplane: alpha=%f  C_L=%f\n", alpha, cl);
      }

      // v = sqrt(m * g * 2 / (F * CL * rho))
      // I'll use conversions to SI units, just to understand the numbers...
      const float foot = 0.3048;               // m
      const float slug = 14.5939041995;        // kg
      float m    = Mass * slug;                // kg
      float g    = 9.81;                       // m/s^2
      float F    = S_ref * (foot*foot);        // m^2
      float rho  = 1.225;                      // kg/m^3
      float v = sqrt(m * g * 2 / (F * cl * rho));
      //      std::cout << "trimmedFlightVelocity = " << v << " m/s\n";
      trimmedFlightVelocity = v / foot; // ft/s
    }

    std::cout << "DotAirLoader::DotAirLoader(\"" << filename << "\") done\n";
  }

  model        = getParamAsString(filename, "model");
  engine_sound = getParamAsString(filename, "engine_sound");
    
  {    
    // try to load new style config and evaluate it    
    std::string name = filename;
    if (name.find(".air") != std::string::npos && name.find(".air") == name.length()-4)
    {
      name = name.substr(0, name.length()-4);
      name += ".xml";
    }
    try
    {
      SimpleXMLTransfer* xml = new SimpleXMLTransfer(name.c_str());
      power = new Power::Power(xml);
      // 
      power_descr = xml;
    }
    catch (XMLException e)
    {
      power = (Power::Power*)0;
      power_descr = (SimpleXMLTransfer*)0;
      std::cout << e.what() << "\n";
    }

    // has power been loaded from new style config?
    if (power == (Power::Power*)0)
    {
      // Do we need a power system because of entry in old style config?
      if (max_thrust > 0)
      {
        SimpleXMLTransfer* xml = new SimpleXMLTransfer();
        
        xml->setAttribute("power.automagic.F", doubleToString(max_thrust*LBF_TO_N));
        xml->setAttribute("power.automagic.V", doubleToString(trimmedFlightVelocity*FT_TO_M));
        
        xml->setAttribute("power.automagic.battery.automagic.T", "420");
        
        xml->setAttribute("power.automagic.battery.shaft.J",     "0");
        
        xml->setAttribute("power.automagic.battery.shaft.propeller.D", doubleToString(span*0.2));
        xml->setAttribute("power.automagic.battery.shaft.propeller.H", doubleToString(span*0.2*0.7));
        xml->setAttribute("power.automagic.battery.shaft.propeller.J",      "0");
        
        if (engine_type == 0)
        {
          xml->setAttribute("power.automagic.battery.throttle_min",                   "0.2");
          xml->setAttribute("power.automagic.battery.shaft.brake",                    "0");
          xml->setAttribute("power.automagic.battery.shaft.propeller.n_fold",         "-1");
        }
        else
        {
          xml->setAttribute("power.automagic.battery.throttle_min",                   "0");
          xml->setAttribute("power.automagic.battery.shaft.brake",                    "1");
          xml->setAttribute("power.automagic.battery.shaft.propeller.n_fold",         "5");
        }
        xml->setAttribute("power.automagic.battery.shaft.engine.automagic.omega_p", "2827");
        xml->setAttribute("power.automagic.battery.shaft.engine.automagic.eta_opt", "0.78");
        xml->setAttribute("power.automagic.battery.shaft.engine.automagic.eta",     "0.7");
                
        // 
        power_descr = xml->getChild("power");        
      }
      else
      {
        power_descr = (SimpleXMLTransfer*)0;
      }
    }
  }
}
/*}}}*/

DotAirLoader::~DotAirLoader()
{
  delete power;
}

/** 
 * Returns name of airplane
 */
std::string getBasename(std::string filename)
{
  if (strU(filename).find(".AIR") == filename.length()-4)
  {
    std::string::size_type pos;
    std::string            outname = filename.substr(0, filename.length()-4);
    
    pos  = outname.rfind('/');
    if (pos != std::string::npos)
      outname = outname.substr(pos+1);
    
    return(outname);
  }
  else
    return(""); 
}


/**
 * description: see header
 */
std::string air_to_xml_file_load(std::string filename)
{
  int nSuc = 0;
  
  nSuc = air_to_xml_file(filename);
  
  if (nSuc == 1)
    return(getXMLFilename(filename));
  else
    return(filename);
}

/**
 * description: see header
 */
int air_to_xml_file(std::string filename)
{
  int nSuccess = 0;
  
  std::string outname = getXMLFilename(filename);
  
  if (outname.length() > 0)
  {    
    std::cout << "Trying to convert " << filename << " to " << outname << "\n";
    if (FileSysTools::fileExists(outname))
    {
      std::cout << "File exists; aborting\n";
      nSuccess = 1;
    }
    else
    {        
      DotAirLoader*      air = new DotAirLoader(filename.c_str());
      SimpleXMLTransfer* xml = new SimpleXMLTransfer();
      SimpleXMLTransfer* i1;
      SimpleXMLTransfer* i2;
      SimpleXMLTransfer* i3;
      SimpleXMLTransfer* conf;
      unsigned int       uSize;
      
      xml->setName("CRRCSim_airplane");
      xml->setAttribute("version", "2");

      // description
      i1 = new SimpleXMLTransfer();
      i1->setName("description");
      i2 = new SimpleXMLTransfer();
      i2->setName("en");
      i2->setContent("This plane has been automatically converted from " + getBasename(filename) + ".air. Please update this text if you know more about it.");
      i1->addChild(i2);
      xml->addChild(i1);
    
      // changelog: erster Eintrag
      i1 = new SimpleXMLTransfer();
      i1->setName("changelog");
      i2 = new SimpleXMLTransfer();
      i2->setName("change");
      
      i3 = new SimpleXMLTransfer();
      i3->setName("date");      
      i3->setContent("Unknown");
      i2->addChild(i3);
      
      i3 = new SimpleXMLTransfer();
      i3->setName("author");
      i3->setContent(MOD_FDM_INFOSTR);
      i2->addChild(i3);
      
      i3 = new SimpleXMLTransfer();
      i3->setName("en");
      i3->setContent("Automatically converted from .air file.");
      i2->addChild(i3);
      
      i1->addChild(i2);
            
      // changelog: Beispiel
      i2 = new SimpleXMLTransfer();
      i2->setName("change");
      
      i3 = new SimpleXMLTransfer();
      i3->setName("date");      
      i3->setContent("Please write date.");
      i2->addChild(i3);
      
      i3 = new SimpleXMLTransfer();
      i3->setName("author");
      i3->setContent("Please write your name and email.");
      i2->addChild(i3);
      
      i3 = new SimpleXMLTransfer();
      i3->setName("en");
      i3->setContent("Please write down what you changed.");
      i2->addChild(i3);
      
      i1->addChild(i2);
      xml->addChild(i1);
            
      
      
      
      i1 = new SimpleXMLTransfer();
      i1->setName("aero");
      i1->setAttribute("version", "1");
      i1->setAttribute("units",   "0"); // units as used in air-file
      
      i2 = new SimpleXMLTransfer();
      i2->setName("ref");
      i2->setAttribute("chord", doubleToString(air->C_ref));
      i2->setAttribute("span",  doubleToString(air->B_ref));
      i2->setAttribute("area",  doubleToString(air->S_ref));
      i2->setAttribute("speed", doubleToString(air->U_ref));
      i1->addChild(i2);
      
      i2 = new SimpleXMLTransfer();
      i2->setName("misc");
      i2->setAttribute("Alpha_0",  doubleToString(air->Alpha_0));
      i2->setAttribute("eta_loc",  doubleToString(air->eta_loc));
      i2->setAttribute("CG_arm",   doubleToString(air->CG_arm));
      i2->setAttribute("span_eff", doubleToString(air->span_eff));
      i1->addChild(i2);
      
      i2 = new SimpleXMLTransfer();
      i2->setName("m");
      i2->setAttribute("Cm_0",  doubleToString(air->Cm_0));
      i2->setAttribute("Cm_a",  doubleToString(air->Cm_a));
      i2->setAttribute("Cm_q",  doubleToString(air->Cm_q));
      i2->setAttribute("Cm_de", doubleToString(air->Cm_de));
      i1->addChild(i2);
      
      i2 = new SimpleXMLTransfer();
      i2->setName("lift");
      i2->setAttribute("CL_0",    doubleToString(air->CL_0));
      i2->setAttribute("CL_max",  doubleToString(air->CL_max));
      i2->setAttribute("CL_min",  doubleToString(air->CL_min));
      i2->setAttribute("CL_a",    doubleToString(air->CL_a));
      i2->setAttribute("CL_q",    doubleToString(air->CL_q));
      i2->setAttribute("CL_de",   doubleToString(air->CL_de));
      i2->setAttribute("CL_drop", doubleToString(air->CL_drop));
      i2->setAttribute("CL_CD0",  doubleToString(air->CL_CD0));
      i1->addChild(i2);
      
      i2 = new SimpleXMLTransfer();
      i2->setName("drag");
      i2->setAttribute("CD_prof",  doubleToString(air->CD_prof));
      i2->setAttribute("Uexp_CD",  doubleToString(air->Uexp_CD));
      i2->setAttribute("CD_stall", doubleToString(air->CD_stall));
      i2->setAttribute("CD_CLsq",  doubleToString(air->CD_CLsq));
      i2->setAttribute("CD_AIsq",  doubleToString(air->CD_AIsq));
      i2->setAttribute("CD_ELsq",  doubleToString(air->CD_ELsq));
      i1->addChild(i2);
      
      i2 = new SimpleXMLTransfer();
      i2->setName("Y");
      i2->setAttribute("CY_b",  doubleToString(air->CY_b));
      i2->setAttribute("CY_p",  doubleToString(air->CY_p));
      i2->setAttribute("CY_r",  doubleToString(air->CY_r));
      i2->setAttribute("CY_dr", doubleToString(air->CY_dr));
      i2->setAttribute("CY_da", doubleToString(air->CY_da));
      i1->addChild(i2);
      
      i2 = new SimpleXMLTransfer();
      i2->setName("l");
      i2->setAttribute("Cl_b",  doubleToString(air->Cl_b));
      i2->setAttribute("Cl_p",  doubleToString(air->Cl_p));
      i2->setAttribute("Cl_r",  doubleToString(air->Cl_r));
      i2->setAttribute("Cl_dr", doubleToString(air->Cl_dr));
      i2->setAttribute("Cl_da", doubleToString(air->Cl_da));
      i1->addChild(i2);
      
      i2 = new SimpleXMLTransfer();
      i2->setName("n");
      i2->setAttribute("Cn_b",  doubleToString(air->Cn_b));
      i2->setAttribute("Cn_p",  doubleToString(air->Cn_p));
      i2->setAttribute("Cn_r",  doubleToString(air->Cn_r));
      i2->setAttribute("Cn_dr", doubleToString(air->Cn_dr));
      i2->setAttribute("Cn_da", doubleToString(air->Cn_da));
      i1->addChild(i2);

      
      xml->addChild(i1);

      
      
      
      
      
      conf = new SimpleXMLTransfer();
      conf->setName("config");
      conf->setAttribute("version", "1");
      
      
      i2 = new SimpleXMLTransfer();
      i2->setName("descr_long");
      i3 = new SimpleXMLTransfer();
      i3->setName("en");
      i3->setContent("Automatically converted from " + getBasename(filename) + ".air.");
      i2->addChild(i3);
      conf->addChild(i2);
      
      i2 = new SimpleXMLTransfer();
      i2->setName("descr_short");
      i3 = new SimpleXMLTransfer();
      i3->setName("en");
      i3->setContent("default");
      i2->addChild(i3);
      conf->addChild(i2);
      
      
      i1 = new SimpleXMLTransfer();
      i1->setName("mass_inertia");
      i1->setAttribute("version", "1");
      i1->setAttribute("units",   "0"); // units as used in air-file
      i1->setAttribute("Mass", doubleToString(air->Mass));
      i1->setAttribute("I_xx", doubleToString(air->I_xx));
      i1->setAttribute("I_yy", doubleToString(air->I_yy));
      i1->setAttribute("I_zz", doubleToString(air->I_zz));
      i1->setAttribute("I_xz", doubleToString(air->I_xz));            
      conf->addChild(i1);
   
      
      i1 = new SimpleXMLTransfer();
      i1->setName("sound");
      i1->setAttribute("version",      "1");
      i2 = new SimpleXMLTransfer();
      i2->setName("sample");
      i2->setAttribute("filename",    air->engine_sound);
      i2->setAttribute("type",        air->engine_type);
      i2->setAttribute("pitchfactor", doubleToString(air->engine_sound_pitchfactor));
      i2->setAttribute("maxvolume",   "1");
      i1->addChild(i2);
      conf->addChild(i1);
      
      if (air->power_descr != (SimpleXMLTransfer*)0)
      {
        air->power_descr->setAttributeOverwrite("units", "1");
        conf->addChild(air->power_descr);
      }
      
      
      xml->addChild(conf);
      
      
      
      
      i1 = new SimpleXMLTransfer();
      i1->setName("graphics");
      i1->setAttribute("version", "1");
      i1->setAttribute("model",   air->model);
      
      i2 = new SimpleXMLTransfer();
      i2->setName("descr_long");
      i3 = new SimpleXMLTransfer();
      i3->setName("en");
      i3->setContent("Automatically converted from " + getBasename(filename) + ".air.");
      i2->addChild(i3);
      i1->addChild(i2);
      
      i2 = new SimpleXMLTransfer();
      i2->setName("descr_short");
      i3 = new SimpleXMLTransfer();
      i3->setName("en");
      i3->setContent("default");
      i2->addChild(i3);
      i1->addChild(i2);
      
      xml->addChild(i1);

      
      
      i1 = new SimpleXMLTransfer();
      i1->setName("wheels");
      i1->setAttribute("version", "1");
      i1->setAttribute("units",   "0"); // units as used in air-file

      uSize = air->wheels.size();
      for (unsigned int uCnt=0; uCnt<uSize; uCnt++)
      {
        i2 = new SimpleXMLTransfer();
        i2->setName("wheel");
        
        i3 = new SimpleXMLTransfer();
        i3->setName("pos");
        i3->setAttribute("x", doubleToString(air->wheels[uCnt].v_P.r[0]));
        i3->setAttribute("y", doubleToString(air->wheels[uCnt].v_P.r[1]));
        i3->setAttribute("z", doubleToString(air->wheels[uCnt].v_P.r[2]));
        i2->addChild(i3);
        
        i3 = new SimpleXMLTransfer();
        i3->setName("spring");
        i3->setAttribute("constant", doubleToString(air->wheels[uCnt].spring_constant));
        i3->setAttribute("damping",  doubleToString(air->wheels[uCnt].spring_damping));
        i2->addChild(i3);

        if (air->wheels[uCnt].steering_mapping != 0 || air->wheels[uCnt].steering_max_angle != 0)
        {
          i3 = new SimpleXMLTransfer();
          i3->setName("steering");
          i3->setAttribute("mapping",    air->wheels[uCnt].steering_mapping);
          i3->setAttribute("max_angle",  air->wheels[uCnt].steering_max_angle);
          i2->addChild(i3);
        }
        
        i2->setAttribute("percent_brake",      doubleToString(air->wheels[uCnt].percent_brake));
        i2->setAttribute("caster_angle_rad",   doubleToString(air->wheels[uCnt].caster_angle_rad));
                                                          
        i1->addChild(i2);        
      }                       
      xml->addChild(i1);

                                 
      // todo: old-style: max_thrust
            
      // write to file
      std::ofstream out;
      out.open(outname.c_str());
      if (out)
      {
        xml->print(out);
        out.close();
        nSuccess = 1;
      }
      
      delete air;
      delete xml;
    }
  }
        
  return(nSuccess);
}

/**
 * returns number of files converted
 */
int air_to_xml_dir(std::string dirname)
{  
  return(0);
}

/** 
 * description: see header
 */
std::string getXMLFilename(std::string filename)
{
  if (strU(filename).find(".AIR") == filename.length()-4)
  {
    std::string outname = filename.substr(0, filename.length()-3) + "xml";
    return(outname);
  }
  else
    return(""); 
}

/** 
 * description: see header
 */
int air_to_xml()
{
  int nNrConv = 0;
    
  return(nNrConv);
}
