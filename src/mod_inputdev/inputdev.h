/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2004, 2005, 2007, 2008, 2010 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2005, 2007, 2008, 2010 - Jan Reucker
 *   Copyright (C) 2007 - Martin Herrmann
 *   Copyright (C) 2006 - Todd Templeton
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
#ifndef TX_INTERFACE_H
#define TX_INTERFACE_H

#include "../mod_misc/SimpleXMLTransfer.h"
#include "../mod_fdm/fdm_inputs.h"
#include "../mod_math/ratelim.h"


#include <stdio.h>
#include <string>

// 1: show constructor/init/destructor and a chunk of serial input data
//    for all T_TX_InterfaceSerial-based interfaces
// 2: plus getInputData, and continous serial input stream for
//    all T_TX_InterfaceSerial-based interfaces
#define DEBUG_TX_INTERFACE  (1)



// maximum number of axis supported by CRRCSim
#define TX_MAXAXIS    (8)

/**
 * Strings ordered like radio type enums in T_TX_Interface
 */
extern const char* RadioTypeStrings[];

// forward declarations
class T_TX_Interface;
class T_TX_Mixer;
class T_AxisMapper;
class T_Calibration;


/** \brief A simple axis-number-to-function-mapper.
 *
 *  This class maps a control function to each input device
 *  axis. The actual mapping is done through the public array
 *  <code>axis[]</code>. The integer value of <code>axis[n]</code>
 *  is the symbolic constant value of the control function assigned
 *  to axis <code>n</code>. Possible values are AILERON, ELEVATOR,
 *  THROTTLE, RUDDER and NOTHING.
 *
 *  Besides the array the class contains the usual functions for
 *  getting the initial values from the XML config file and putting
 *  them back if they were modified by the application.
 */
class T_AxisMapper
{
  public:
    /**
     * Initialize an empty T_AxisMapper
     */
    T_AxisMapper(T_TX_Interface *parent);

    /**
     * Create a T_AxisMapper object from the given config file.
     * It's the same as using the empty constructor and calling
     * <code>T_AxisMapper::init(cfg, child)</code>.
     */
    T_AxisMapper(T_TX_Interface *parent, SimpleXMLTransfer* cfg, std::string child);
  
   /**
    * Total number of functions for the axis (including NOTHING).
    */
   enum { NUM_AXISFUNCS=9 };

   /**
    *
    */
   enum { NOTHING=0, AILERON=1, ELEVATOR=2, RUDDER=3, THROTTLE=4,
          FLAP=5, SPOILER=6, RETRACT=7, PITCH=8 };
  
    int   func[T_AxisMapper::NUM_AXISFUNCS];    ///< joystick functions
    float inv[T_AxisMapper::NUM_AXISFUNCS];     ///< -1.0: invert, 1.0: don't invert
   
   /**
    * Available radio types. Ordered like RadioTypeStrings[]
    */
   enum { FUTABA = 0, AIRTRONICS = 1, HITEC = 2, 
          JR = 3,     COCKPIT = 4,    WALKERA=5, CUSTOM = 6 };
   
   /**
    * Number of available radio types
    */
   enum { NR_OF_RADIO_TYPES = 7 };

   /**
    * Loads configuration from <code>cfgfile</code>.
    * \param cfgfile Pointer to the XML config tree
    * \param childname Full name of the "bindings" child of the selected interface.
    */
    void init(SimpleXMLTransfer* cfgfile, std::string childname);
   
   /**
    * Writes current configuration back into <code>cfgfile</code>
    * \param cfgfile Pointer to the XML config tree
    */
    void putBackIntoCfg(SimpleXMLTransfer* cfgfile);
  
   /**
    * Set the mapper to a standard configuration.
    * \param radio_type Symbolic name of a predefined radio layout
    */
    void setRadioType(int rtype);
   
   /**
    * selected radio type
    */
   int radioType()  {return radio_type;};

   /**
    * Save the current mapping to the CUSTOM entries.
    * To make the mapping persistent you have to call
    * <code>T_AxisMapper::putBackIntoCfg()</code>
    * after that.
    */
   void saveToCustom();
   
  private:
    int   getValAxis  (std::string asString, int nDefault);
    std::string child_in_cfg;
    int   radio_type;             ///< selected radio type
    int   c_func[T_AxisMapper::NUM_AXISFUNCS];  ///< joystick axes (for CUSTOM mapping)
    float c_inv[T_AxisMapper::NUM_AXISFUNCS];   ///< polarity (for CUSTOM mapping)
    T_TX_Interface *iface;    ///< pointer to the 'parent' interface
  
};


/** \brief A simple software mixer.
 *
 *  This class yields the settings of a software mixer
 *  and offers methods to transfer it from and to a
 *  config file.
 *
 *  Note: Public data members are often considered as bad design,
 *  but as this class is more or less something like an extended
 *  struct I think it's o.k. in this case...
 */
class T_TX_Mixer
{
  public:
   /**
    * Number of mixers.
    */
    enum { NUM_MIXERS=4 };

    T_TX_Mixer(T_TX_Interface *parent);
    T_TX_Mixer(T_TX_Interface *parent, 
               SimpleXMLTransfer* cfg,
               std::string child);
    ~T_TX_Mixer();

    int  init(SimpleXMLTransfer* cfg, std::string child);
    int  reinit(SimpleXMLTransfer* cfg, bool fIsChild = true);
    void putBackIntoCfg(SimpleXMLTransfer* config);
    std::string getErrMsg();
  
    float mix_unsigned(float in, int function) const;
    float mix_signed(float in, int function) const;
    float mix_mixer(float *in, int function) const;

    float trim_val[T_AxisMapper::NUM_AXISFUNCS];
    float nrate_val[T_AxisMapper::NUM_AXISFUNCS]; // normal rate
    float srate_val[T_AxisMapper::NUM_AXISFUNCS]; // slow rate
    float exp_val[T_AxisMapper::NUM_AXISFUNCS];

    float mtravel_val[T_AxisMapper::NUM_AXISFUNCS]; // -travel 0..-0.5
    float ptravel_val[T_AxisMapper::NUM_AXISFUNCS]; // +travel 0..+0.5

    int enabled;
    int dr_enabled; // if dr_enabled (dual rate) use slow rate, else normal rate
    
    int   mixer_enabled[T_TX_Mixer::NUM_MIXERS];  // mixer enabled
    int   mixer_src[T_TX_Mixer::NUM_MIXERS];      // mixer source channel
    int   mixer_dst[T_TX_Mixer::NUM_MIXERS];      // mixer destination channel 
    float mixer_val[T_TX_Mixer::NUM_MIXERS];      // mixing rate

  private:
    void baseInit();
    float mix_exp(const float x, const float p) const;
    std::string   child_in_cfg;   ///< name of the child in the config file
    std::string   errMsg;         ///< error message
    T_TX_Interface *iface;        ///< pointer to 'parent' interface
};


/** \brief Calibration data class
 *
 *  This class holds calibration data for an interface,
 *  exchanges this data with a config file and offers
 *  a routine to apply the calibration values to a
 *  raw input value.
 */
class T_Calibration
{
  public:
    /**
     * Create an empty T_Calibration object
     */
    T_Calibration(T_TX_Interface *parent);

    /**
     * Create a T_Calibration object from the given config file.
     * It's the same as using the empty constructor and calling
     * <code>T_Calibration::init(cfg, child)</code>.
     */
    T_Calibration(T_TX_Interface *parent, 
                  SimpleXMLTransfer* cfg,
                  std::string child);

   /**
    * Loads configuration from <code>cfgfile</code>.
    * \param cfgfile Pointer to the XML config tree
    * \param childname Full name of the selected interface's child
    */
    void init(SimpleXMLTransfer* cfgfile, std::string childname);
   
   /**
    * Writes current configuration back into <code>cfgfile</code>
    * \param cfgfile Pointer to the XML config tree
    */
    void putBackIntoCfg(SimpleXMLTransfer* cfgfile);
  
   /**
    * Apply the calibration values to a raw value
    * \param axis The number of the axis to be calibrated.
    * \param raw  The raw, uncalibrated value.
    * \return The calibrated axis value.
    */
    float calibrate(int axis, float raw);
    
   /**
    * Set value for one axis.
    */
    void setValMinMax(int axis, float valmin, float valmax) {val_min[axis] = valmin; val_max[axis] = valmax; };
    
   /**
    * Set value for one axis.
    */
   void setValMid(int axis, float val) {val_mid[axis] = val;};
  
   /**
    * 
    */
   void PrintSettings(int axis);
    
  private:
    std::string child_in_cfg;
    float val_min[TX_MAXAXIS];
    float val_mid[TX_MAXAXIS];
    float val_max[TX_MAXAXIS];
    T_TX_Interface *iface;    ///< pointer to 'parent' interface
};


/**
 * There already are some TX-interfaces implemented into crrcsim and therefore
 * lots of 'if' and things in the code. I wanted to add my own interface, but
 * I don't like 'if' that much, so I implemented a new scheme.
 * All those 'if' could be removed if the interfaces were ported to this scheme.
 * See tx_interface.h and the things in interface_serial2 for an example.
 * 
 * Now I ported RCTRAN, the audio interface and PARALLEL1 to PARALLEL3 to use 
 * this scheme, too. 
 * 
 * There is a pointer to a class T_TX_Interface in crrc_main, which is
 * initialized on startup. Depending on the configuration, it points to one of
 * the interface-classes. That's it, no more 'if'.
 * 
 * Jens Wilhelm Wulf, 06.01.2005
 */
class T_TX_Interface
{
  public:
    T_TX_Interface();
    virtual ~T_TX_Interface();
   
    /**
     * Input methods. Ordered like InputMethodStrings[]
     */
    enum { NUM_INPUTMETHODS=12 };
    enum {  eIM_keyboard = 0, eIM_mouse   = 1,  eIM_joystick = 2, 
            eIM_rctran   = 3, eIM_audio   = 4,  eIM_parallel = 5,  
            eIM_serial2  = 6, eIM_rctran2 = 7,  eIM_serpic   = 8,  
            eIM_mnav     = 9, eIM_zhenhua = 10, eIM_CT6A     = 11 };
    /**
     * Get input method
     */
    virtual int inputMethod() { return(-1); };
   
    /**
     * Initialize interface. Read further config data from a file, if necessary.
     */
    virtual int init(SimpleXMLTransfer* config);

    /**
     * Write configuration back
     */
    virtual void putBackIntoCfg(SimpleXMLTransfer* config);
   
    /**
     * Set current input data. If some value is not available, the value 
     * is not overwritten.
     * From crrc_main.c it seems like the values should be in the range:
     * aileron, elevator, rudder: -0.5 .. 0.5
     * throttle: 0.0 .. 1.0
     * others?   
     */
    virtual void getInputData(TSimInputs* inputs);
   
    /**
     * Get raw interface data. Needed for calibration.
     * \param target Location where the data will be written to.
     *               Make sure there's enough memory reserved for
     *               getNumAxes() values!
     */
    virtual void getRawData(float* target) {};
   
     
    /**
     * Get the number of axis of the current interface.
     */
    virtual int  getNumAxes()  {return TX_MAXAXIS;};


    /**
     * Get text of error message (if any).
     */
    std::string getErrMsg();

    /**
     * Some generic interface hooks for the "software"
     * interfaces which receive their motion commands
     * from the event loop.
     */
    virtual void move_aileron(const float x);
    virtual void move_rudder(const float x);
    virtual void move_elevator(const float x);
    virtual void move_flap(const float x);
    virtual void move_spoiler(const float x);
    virtual void move_retract(const float x);
    virtual void move_pitch(const float x);
    virtual void increase_throttle();
    virtual void decrease_throttle();
    virtual void centerControls();

    virtual void setAxis(int axis, const float x) {};
   
    /**
     * Query if the interface uses the mixer.
     */
    virtual bool usesMixer()       {return (mixer != NULL);};

    /**
     * Query if the interface uses the mapper.
     */
    virtual bool usesMapper()      {return (map != NULL);};

    /**
     * Query if the interface uses the calibration class.
     */
    virtual bool usesCalibration() {return (calib != NULL);};

    /**
     * The parameters for scaling are public to make configuration easier.
     */
    T_TX_Mixer    *mixer;
    T_Calibration *calib;
    T_AxisMapper  *map;
   
    /**
     * Limit values to the "mechanical" range of a servo.
     * \param in Input value
     * \return The input value, but clamped to -0.5 ... 0.5
     */
    virtual float limit(float in);

    /**
     * Limit values to the "mechanical" range of a servo which
     * operates only in one direction (e.g. throttle, spoiler).
     * In addition, very small values are set to 0.
     *
     * \param in Input value
     * \return The input value, but clamped to 0.0 ... 1.0
     */
    virtual float limit_unsigned(float in);

    /**
     * Return the device name of the interface. The meaning
     * of this name is interface-dependend, if it is used
     * at all.
     */
    virtual std::string getDeviceName()  {return std::string("");};
   
    /**
     * Return the speed of the interface. The meaning
     * of this number is interface-dependend, if it is used
     * at all.
     */
    virtual int getDeviceSpeed()  {return 0;};

    /**
     * Set the device name of the interface. The meaning
     * of this name is interface-dependend, if it is used
     * at all.
     */
    virtual void setDeviceName(std::string devname)  {};
   
    /**
     * Set the speed of the interface. The meaning
     * of this number is interface-dependend, if it is used
     * at all.
     */
    virtual void setDeviceSpeed(int speed)  {};
  
    /**
     * Using this method might not be perfect, but code like this
     * happened to be used in nearly every T_TX_Interface, so I felt
     * like providing/using a single implementation...
     */
    void CalibMixMapValues(TSimInputs* inputs, float* myArrayOfValues);
  
    /**
     * Pre-initialize the axis values with the values from the
     * keyboard controller emulation.
     */
    inline void preInitFromKeyboard(TSimInputs* inputs)
    {
      inputs->elevator = keyb_elevator_input;
      inputs->aileron  = keyb_aileron_input;
      inputs->rudder   = keyb_rudder_input;
      inputs->throttle = keyb_throttle_input;
      inputs->flap     = keyb_flap_input;
      inputs->spoiler  = keyb_spoiler_limited.val;
      inputs->retract  = keyb_retract_limited.val;
      inputs->pitch    = keyb_pitch_input;
    }
      
    /**
     * Toggle the emulated retract axis.
     * Each call to this function will toggle the value of the emulated retract
     * axis. This value will be used if no "real" controller axis is
     * configured for the retract function.
     */
    virtual void toggleRetract();

    /**
     * Toggle the emulated spoiler axis.
     * Each call to this function will toggle the value of the emulated spoiler
     * axis. This value will be used if no "real" controller axis is
     * configured for the spoiler function.
     */
    virtual void toggleSpoiler();

    /**
     * Perform cyclic updates for the interface driver.
     * \param dt  elapsed time since last call
     */
    void update(double dt);
  
    /**
     * Reset the interface.
     */
    void reset();

  protected:
    /**
     * What kind of input device is this?
     */
    int input_device;

    /**
     * error message
     */
    std::string errMsg;

    float keyb_aileron_input;
    float keyb_rudder_input;
    float keyb_elevator_input;
    float keyb_throttle_input;
    float keyb_flap_input;
    float keyb_spoiler_input;
    float keyb_retract_input;
    float keyb_pitch_input;
  
    CRRCMath::RateLimiter<float> keyb_retract_limited;
    CRRCMath::RateLimiter<float> keyb_spoiler_limited;
};


#endif

