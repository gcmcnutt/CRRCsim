__          __             _     _ 
\ \        / /            | |   (_)
 \ \  /\  / /_ _ ___  __ _| |__  _ 
  \ \/  \/ / _` / __|/ _` | '_ \| |
   \  /\  / (_| \__ \ (_| | |_) | |
    \/  \/ \__,_|___/\__,_|_.__/|_|

                                   
To add the Wasabi slope aerobatic glider model to your CRRCsim you should:
1) copy wasabi.xml file into your CRRCsim installation "models" folder.
2) copy wasabi_wr.3ds and wasabi_yr.3ds files into your CRRCsim installation "objects" folder.
3) copy this file into your CRRCsim installation "documentation/models" folder.
The Wasabi model will now be available for selection, in two different colour schemes.

This model is a small (1.5m span) and "extreme" slope aerobatic glider.
It features a symmetric wing airfoil, so it will behave almost the same both upright and inverted.
For maximum performance the large aileron should be used as flap as well, lowering them to fly slowly 
in weak lift and mixing them with elevator (snap-flap) to enhance maximum positive/negative lift
and thus manuvering capability.

The 3d model has been prepared with Blender.
The aerodynamic model has been computed with XFLR (airfoil polars) and AVL (stability and control 
derivatives). Comments about it are embedded into the .xml file, including the suggestion for mixers
setting, copied here folowing as well:

      Assumed control travel are: 
        de=+/-8deg dr=+/-20deg da=+/-15deg flaps=+/-30deg.
      Note that actual flap range should be limited to +/-8deg as camber 
      changing flap (e.g. as snap flap), while 30deg down deflection is 
      to be used for landing.
      Best flown with snap-flap, using following mixer settings:
        elevator: rate=0.6 (i.e. +/-4.8deg)
        flap:     -travel=0.133, +travel=0.133
        mixer1:   elevator-to-flap mixing rate=0.533
                  (flap +/-8deg with ele +/-4deg)
        mixer2:   flap-to-elevator mixing rate=-0.467 
                  (elevator compensation 1deg down for 8deg flap down, to trim a DCL=0.8)
        mixer3:   spoiler-to-flap mixing rate=-0.5 
                  (flap down 30deg with full spoiler)
        mixer4:   spoiler-to-elevator mixing rate=0.234 
                  (elevator 6deg down for 30deg flap down)

It is also suggested to apply some expo to both aileron and elevator.
Note that if you leave full elevator throw (rate = 1) the wing will stall if pulling to hard, while 
with reduced throw (rate = 0.6) you will be able to pull full stick while manuvering without stalling.

To setup expo, reduced elevator rate and the mixers simply go to the Mixer dialog in 
Option->Control menu and load preset "Wasabi default" configuration.

Have fun.

Luca G., Feb 2012.