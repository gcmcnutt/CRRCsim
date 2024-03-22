-------- Hardware interface, overview ---------------------

The PCs parallel port has an interrupt request input, which is pin 10 on the
DSub-25 connector.
The old 'IBM PC Parallel Port FAQ' says this IRQ to be positive edge
triggered. The pin is said to be a TTL input.

'Interfacing the Standard Parallel Port' by Craig Peacock says "However like
always, some cards may trigger the interrupt on the high to low transition.".


-------- Joystick driver ----------------------------------

The current module allows you to use your transmitter as a joystick, too! So
you can use it via 'rctran2' OR via 'joystick' in crrcsim.


-------- Hardware interface, 1 ----------------------------

Here is one possibility, found on the pages describing the PPJoy-Driver for
Windows. It is said to work with every PPM signal with at least TTL level.

This one should work with PPJoy for windows and with rctran(1/2) for linux.

PPM signal -------680 Ohm--------+------------- Ack, Pin 10 DSub-25
                                 |
                               ----
                                /\
                               /  \ ZD 4,7
                               ----
                                 |
                                 |
GND       -----------------------+-------------- GND, Pin 18 DSub-25





-------- Hardware interface, 2 ----------------------------

I don't like a limiting diode, so I opted to use this:

                                        +------------ D0, Pin 2 DSub-25
                                        |
                                        |
                                       4k7
                                        |
                                        |
                                        +------------ Ack, Pin 10 DSub-25
                                        |
                                       /
                                     |/ C
                                   B |
PPM signal ---------10k---+-----+----+    General Purpose NPN
                          |     |    |                                      
                          |     |    |\ E
                         ---   47k     \
                     1nF ---    |       |
                          |     |       |
                          |     |       |
GND        ---------------+-----+-------+------------ GND, Pin 18 DSub-25


The software has to set D0 to output high level, which is done by the
driver in this directory. I do not know if PPJoy does currently support this
interface. Maybe an email to its author helps.


-------- Kernel 2.4.x -------------------------------------

Login as root.
Call 'make -f Makefile.24' in this directory to compile the module 'rctran2.o'.
Call 'make -f Makefile.24 install' to install it.
Call 'modprobe rctran2' to load the module,
'rmmod rctran2' to unload it.


-------- Kernel 2.6.x -------------------------------------

Call
   cp Makefile.26 Makefile
   make
in this directory to compile the module 'rctran2.ko'.

After that (as root):

insmod ./rctran2.ko

I experienced building this module for 2.6.x to be more tricky than for
2.4.x. It worked after I compiled the whole kernel from source and used that.


Misc notes

http://www.captain.at/programming/kernel-2.6/ says:
  "If you get something like this
       # make
       make: Nothing to be done for `default'.
   you need to install the kernel source and compile the kernel first (run 
   "make" at least to the point until all "HOSTCC scripts/" stuff is done - 
   this will configure your kernel and allows external module compilation). 
   Make sure /lib/modules/$(shell uname -r)/build points to your build 
   directory (most likely /usr/src/linux...)."

Isn't there a way of compiling the module without getting and compiling the
whole kernel?

Yes. Debian: on my system, "uname -r" currently answers "2.6.21-2-686", so I installed 
  linux-headers-2.6.21-2-686
which also needed
  linux-headers-2.6.21-2
  linux-kbuild-2.6.21
to build the module without installing/compiling full kernel source.  



See "7.6.2. Modultreiber außerhalb der Kernelquellen" in 
  http://ezs.kr.hsnr.de/TreiberBuch/html/sec.kbs01.html
  
http://www.linux-magazin.de/heft_abo/ausgaben/2003/08/kern_technik
  "Um sich darin einzuklinken, muss das Makefile den Pfad zu den
   konfigurierten Kernelquellen kennen. Linus Torvalds hat festgelegt, dass die
   Quellen über das Module-Verzeichnis zu finden sein sollen:
   »/lib/modules/Version/build/«. Meist ist dies ein Symlink auf das bekannte
   Directory »/usr/src/linux-Version«."            

linux/Documentation/kbuild/modules.txt

http://www.cyberciti.biz/tips/build-linux-kernel-module-against-installed-kernel-source-tree.html


-------- Troubleshooting ----------------------------------

On a second terminal, use 
  tail -f /var/log/syslog
to see debug messages.

Do
  rmmod parport_pc
if the parallel port irq is already in use.
Maybe you also need 
  rmmod lp
before.

I experienced that rctran2 did not work if parport_pc had been loaded before
on kernel 2.6.8. Maybe this is a bug in the kernel or the module? I had to
remove parport_pc.ko from /lib/modules/... to use rctran. 
Same applies to 2.6.18 on debian.
To automatically load rctran2 on startup, use
  cp rctran2.ko /lib/modules/`uname -r`/kernel/drivers/parport/
  echo rctran2 >> /etc/modules
  depmod -a

-------- Changes in 2.6 code ------------------------------

2007-10-04:
  See http://lwn.net/Articles/107303/, "MODULE_PARM deprecated"; replaced by 
  module_param().
