## Notes on Fiducial System ##

<Notes by geronm>

### Notes from traffic tests

Pre-change:

Presently, worst offenders are

* Color-request messages:
  Frequency: 70-80 Hz
  Bandwidth: ~3KBps each.
  Channels: RECV_ORDER8, INCOMING_DATAVAL

* Fiducial Readings (probably safe, might turn up):
  Frequency: 20 Hz
  Bandwidth: ~.68 KBps
  Channels: FDCL

* Current Param States (probably safe also, though bursty):
  Frequency: 1 Hz
  Bandwidth: .34 KBps (bursty, mostly teach table)
  Channels: INCOMING_CONFIG, DESIRED_CONFIG, INCOMING_TEACH_TABLE, DESIRED_TEACH_TABLE

Post-change:

Debug removed, Dataval cranked down, FDCL cranked up slightly

* Color-request messages:
  Frequency: 0.5 Hz
  Bandwidth: ~.08 KBps.
  Channels: INCOMING_DATAVAL

* Fiducial Readings (probably safe, might turn up):
  Frequency: 40 Hz
  Bandwidth: ~1.41 KBps
  Channels: FDCL

* Current Param States (probably safe also, though bursty):
  Frequency: 1 Hz
  Bandwidth: .34 KBps (bursty, mostly teach table)
  Channels: INCOMING_CONFIG, DESIRED_CONFIG, INCOMING_TEACH_TABLE, DESIRED_TEACH_TABLE


### Notes from test with software + 2" strip on wheel

* Stably detected rotation speed up to .066 seconds between detections (900 rpm (???)) at a distance
of about 1.3 meters from the wheel. With 2" strip, 20" radius wheel, this was simulating the 2" strips
passing at 47 m/s which is similar to having the real 4" strips passing at 94 m/s. We did not rotate
the wheel any faster than this.

* Averaged time-between-samples converged smoothly to multiple exactly-right values. This is with HOLD=0,
no extra smoothing on our part.

* Didn't get a chance to observe raw time-between-samples times, to assess jitter.

### Temp: Checklist of hypotheses/tests/results ###

* There seem to, in fact, be two sets of configs and teach vectors, as advertised. Arg 0/1 config, 2/3* teach.

  * *note: haven't directly observed Teach Table 1 at arg=3 yet, only Teach Table 0 at arg=2

* It seems that you can set the entire Teach Table at a time even if maxcolno is set to 2.
Can set lower values for outmode=DIRECTHI (not sure about higher); with outmode=BINARY we're good to set any.

* Back long ago, when teach table buffer was to small and was presumably getting stomped, had a nonetheless
weird bug where all the upper values of the teach table (after row 5) were invalid numbers like 22000, and
sensor would accept partial new teach table but not full (checksum problems?). AND
incoming teach tables were failing local checksums *UPDATE: had buffer issues here, too*.
Power cycle re-cleared all the upper rows, solved this issue.

* Local checksums work now, and sensor is definitely accepting my checksummed teach tables.

* System seems to obey thresholds with EVALMODE=BESTHIT, TRIGGER=CONT, CALCMODE=X/Y INT.

* On sunny day in warehouse, 12noon, gain 4, we have:

  * No color: `X Y INT = [2340, 1755, 14]`

  * White paper towel: `X Y INT = [1540, 1400, 560]`

### Problem Overview ###

#### Problem Statement: ####

Need a parsing class that makes it easy to interact with fiducial device. Specific needs include:

* Use serial to flash config parameters to device, and read back their current values.

* Use serial to send any color table row to device, and to read any color table row to device.

* Obtain readings from the outputs of the device (OUT0-OUT4) and send back certain digital signals (IN0).

#### Standard use case workflow: ####

* Put the pod in the tube

* Slide the pod under the marker color, learn to recognize that color. Tune tolerances to readily discern marker vs no marker.

* System now reports sensor's reading of whether that color is in view

#### Software spec: ####

* `fiducial_module.cpp`

  * Maintains digital connection to device.

  * Listens for flips of the OUT0 port (flips correspond to color recognition) and reports them as LCM triggers.

* `FiducialController.cpp`

  * Maintains serial connection to device.

  * Flashes config parameters and color entries, and issues other commands (store/read to/from EEPROM)

* `fiducial_utils.hpp`

  * Contains various utilities, especially for converting between lcmtype representations and RS232 serial commands.

##### Misc: use `make -f testMakefile test` to build tests. Run them in the `test/build` directory. Remove them with `make -f testMakefile clean`.

#### LCM Types: ####

* mithl\_fiducial\_config\_t.lcm  -- A full set of configuration parameters for sensor.

* mithl\_fiducial\_color\_row\_t.lcm -- Teach table row for sensor (ROW-No.,X,Y,CTO,INT,ITO,GROUP).

* mithl\_fiducial\_color\_dataval_t.lcm -- Data value readings from sensor.


#### Class Spec: ####

    class FiducialController {
        Update()         // main loop
        HandleReceive()  // handles incoming communication from device
        ...
    }

### Notes on Logic and RS232 Protocol ###

Major Resource for this information is: [http://www.micro-epsilon.com/download/manuals/man--colorCONTROL-S--en.pdf]()

#### RS232 MSB Details ####

The RS232 protocol is unsigned, Most-Significant-Bit, no parity bit, 1 stop bit.

#### Teaching `i`th color: ####

* To teach `i`th color, pulse IN0 high-then-low `i+1` times, each with duration 250ms and with time between no more than 500ms. Eg. to teach the first color, aka the `0`th color, pulse the IN0 line 1 time.

* Window in which to do this is the same for all `i` BUT is proportional to MAXCOLNO

* Fortunately, we only have to teach one color right now. So one ~300ms-long low-hi-low pulse should do it.

#### BINARY vs DIRECT output: ####

* BINARY is like \[0b00000,0b00001,0b00010, ... ,0b11111\] (31 colors \[0-30\] possible, or NULL \[31\])

* DIRECT-HI is like \[0b00000,0b00001,0b00010,0b00100,0b01000,0b10000\] (5 colors possible, or NULL 0)

* DIRECT-LO is flipped version of DIRECT-HI

    * See docs section **6. Function of the LED display** for ordering specifics.

    * We probably want DIRECT-HI, reading OUT0 to determine whether visible.

#### Color measurement has mode X/Y INT or X/Y/INT (or s/i/ INT, which is weirdly different) ####

* Typically, will want to measure color as X/Y INT or X/Y/INT. This is "Red,Green and Intensity" or "Red,Green,Intensity"

* Difference with/without slash is whether X/Y and INT have separate tolerances `CTO` and `ITO`, or whether there is just one tolerance `TOL`

    * X/Y/INT Single-tolerance mode is implemented as a sphere constraint; check whether measured color is euclidean-distance `TOL` from color in table.

    * X/Y INT Two-tolerance mode is implemented as a _cylinder_ constraint; check whether X,Y is euclidean-distance `CTO` from X,Y in table AND whether INT is within +/-`ITO` of INT in table.

    * Will need to do testing, but we likely want X/Y INT mode with a tighter `CTO` and looser (but still discerning) `ITO`. In any case, will need a mechanism to quickly ascertain good values for these.

#### Useful commands: ####

See spec section **A 6.4 Examples** and its examples for more info/commands.

**ALL of this needs updating:**

* \#1 Flash _config_ parameters from PC to Sensor RAM, PC sends all config parameters to sensor, which stores them in RAM and replies with same message.

* \#2 Flash a _color table row_ from PC to Sensor RAM. Includes ROW-No.,X,Y,CTO,INT,ITO,GROUP. Sensor stores them in RAM and replies with same message.

* \#3 Read parameters from Sensor RAM to PC. Counterpart to \#1. PC sends dummy config parameters to sensor; sensor replies with config information as in \#1.

* \#4 Read a _color table row_ from PC to Sensor RAM. Counterpart to \#2. PC sends dummy values to sensor; sensor replies with color information as  in \#2.

* \#6 and \#8 are `save()` and `load()` operations, respectively. They induce the Sensor to save/load config parameters and teach table between RAM and EEPROM. This is an all-or-nothing operation, cannot save only part of the RAM.

* \#20 Send line OK. Just checks that line is clear:

    * PC sends: 0055, 0014, 00AA, 0000, 0000, .., 0000

    * Sn replies: 00AA, 0014, 00AA, 0000, 0000, .., 0000


* \#50 Enable/Disable autosend of a data frame (\#5) on sensor trigger. Can be used to interface serially with device, rather than through OUT0-OUT4. We probably will want the higher rate from just reading the pins.

#### Convenient list of config field names: ####

    ['power', 'powermode', 'average', 'evalmode', 'hold', 'intlim', 'maxcolno', 'outmode', 'trigger', 'exteach', 'calcmode', 'dyn_win_lo', 'dyn_win_hi', 'color_groups', 'ledmode', 'gain', 'integral']

    ['color_r', 'color_g', 'color_b', 'color_x', 'color_y', 'color_int', 'color_delta_c', 'color_no', 'group', 'trig', 'temp', 'color_r_raw', 'color_g_raw', 'color_b_raw']

    ['row_no','color_x','color_y','cto','color_int','ito','group']
