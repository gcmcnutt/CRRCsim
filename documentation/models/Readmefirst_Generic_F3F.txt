  ____                      _        _____ _____ _____ 
 / ___| ___ _ __   ___ _ __(_) ___  |  ___|___ /|  ___|
| |  _ / _ \ '_ \ / _ \ '__| |/ __| | |_    |_ \| |_   
| |_| |  __/ | | |  __/ |  | | (__  |  _|  ___) |  _|  
 \____|\___|_| |_|\___|_|  |_|\___| |_|   |____/|_|    
                                                       

The Generic F3F model has been derived from the original Skorpion model, and can be used with all the available F3F 3D model. 
Some wrong values have been corrected using reasonable guesses.
Just setting the mass in the range 3-4kg brings the inertias in the expected range suggesting that, 
contrary to the mass, they were estimated right.<br>

The new Generic_F3F model has been implemented to provide a reasonable FDM for F3F-like gliders, replacing Crossfire, Erwin and Skorpion models previously included in the standard distribution package, whose FDM's had some flawed values resulting in unrealistic flying performances.
These three 3D models are now just different graphical appearence of the same Generic_F3F FDM.
This FDM could also be applied to the many more 3D models of F3F gliders available as add-on (this requires editing of the Generic_F3F.xml file).

The FDM includes a suggested setting for mixers and dual rates.
To setup the mixers and dual rates go to the Mixer dialog in Option->Control menu and load the preset "F3F Default" configuration.

Have fun.

Luca G., Feb 2013.