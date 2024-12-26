# miniPID

This is a small, fully self contained PID class designed to help provide simple, efficient tuning, and simple integration wherever any level of PID control might be needed. 

## Design Goals
- Provide all expected features of a quality PID loop. 
- Allow for stability without extensive tuning. 
- Be simple to integrate into projects without code restructuring
- Be easy to "cascade", sending the output of one PID to the input of another. 
- Be flexible enough that any provided functions can be used in isolation. 
- Be simple enough to be used in "transient" or one-off control sequences 

## Features
##### PID Functions
True to the name, the main purpose of this class is PID control.

##### Feed Forward configuaration 
Provides a predicted output, improving responsiveness, accuracy, and while decreasing the initial error the PID acts upon.

##### Setpoint Range
Force the PID system to cap the setpoint to a range near the current input values. This allows you to tune the PID in a smaller range, and have it perform sanely during large setpoint changes. 

##### Output ramping
Allows a maximum rate of change on the output, preventing jumps in output during setpoint changes

##### Output Range
Adjustable min and maximum range, so the output can directly drive a variety of systems. 

##### Output Filtering
Helps smooth the output, preventing high-frequency oscillations.

##### I term restriction
Allows you to specify a maximum output value the I term will generate. This allows active error correction, with minimal risk of I-term windup. 

##### Smart I term buildup
The I term and summed error will never increase if the system is already doing everything permitted to correct the system error. 

##### Simple API. 
No need for lots of convoluted calculation functions, or asyncronous calculation modes. After configuration, `getOutput()` is probably the only function you need. 

## Usage
A bare bones PID system could look like this. 

``` java
MiniPID pid=MiniPID(1,0,0);
//set any other PID configuration options here. 

while(true){
  //get some sort of sensor value
  //set some sort of target value
  double output=pid.getOutput(sensor,target);
  //do something with the output
  Timer.delay(50);  
}
```
That's it. No fuss, no muss. A few lines of code and some basic tuning, and your PID system is in place. 

### Getting and setting outputs
There's several ways. In simple systems, the fastest is  `output=pid.getOutput(sensor,target);`. This does all the calculations with the current values, applying any configuration, and returns the output. 

For more event-driven systems, you can split up the setting and getting, using `output=pid.getOutput(sensor);` and `pid.setSetpoint(target)`.

If your outputs are to be disabled or driven by a different system, then you may want to use the `pid.reset()` method. That will set the PID controller to re-initialize the next time  `getOutput()` is used. Because of this, `reset()` can be used when disabling PID control, when reenabling it, or at any point between.

### Configuring the PID. 
The most complex part of PID systems is the configuration. Tuning a PID process properly typically requires either significant calculation, significant trial and error, or both. 

This library is designed to produce "decent" PID results with minimal effort and time investment, by providing more extensive configuration options than most controllers.

Note, PID systems work best when the calculations are performed at constant time intervals. This PID implimentation does not handle this, and assumes the primary loop or framework handles the precise timing details.

#### `MiniPID(p,i,d)`

These create the basic PID object, allowing for further configuration. Generally, you initialize the PID values here, but you can re-configure on the fly using the `setP(double P)`,`setI(double I)`,`setD(double D)`, and `setF(...)` methods. 

Tuning PID systems is out of scope of this readme, but a good examples can be found all over the internet. 

#### `SetF(...)`
#### `SetFStatic(...)`
#### `SetFArm(...)`
#### `setFGravity(...)`
#### `SetFVelocity(...)`
Feed-Forward is a 0th system variable; While not part of the PID's feedback process, a feed forward allows you to provide your control loop with information about what you *expect* to happen for a given setpoint. In an ideal system, a feed-forward generates the majority of the output. 

A feed forward does not consider what the system is _actually_ doing, and a system driven solely by feed-forward is actually an open-loop system. Considering the difference between the expected result and the actual result is where the PID fits in. When a feedforward is set up right, the PID is left with very little to do, and when there is an error, it's easier to tune for.

A basic set of feed-forwards should be easy to set up, and provide significant benefits in tuning time and performance. This library provides these basics. Advanced feed-forwards can provide incredible precision, but often require more extensive calibration and inputs beyond the scope of this library.

Additional information on the feed-forward terms and the physics they're helping handle is can be found here: [FeedForward](./FeedForward.md)

#### `setOutputLimits(double minimum,double maximum)`
#### `setOutputLimits(double output)`
Optional, but highly recommended to set. The set the output limits, and ensure the controller behaves when reaching the maximum output capabilities of your physical system.

#### `setMaxIOutput(double maximum)`
Sets the maximum output generated by the I term inside the controller. This is independent of the `setMaxOutput` values. This can assist in reducing windup over large setpoint changes or stall conditions. 

#### `setSensorPhase(boolean isInPhase)`
Allows you to correct "sensor phase" errors, where a motor's positive direction and the sensor's positive direction are opposite eachother. 

When in phase, the PID response will attempt to reduce the the error, as expected. When out of phase, the error causes the system action to increase the error further (which is undesirable).

#### `setDirection(boolean reversed)`
Reverses the system output. This doesn't fix any particular issues, but can help when a system's action feels backwards. As an example, if a positive motor output casues an arm to go "down", then this can help make the numbers feel more correct.

#### `reset()`
Resets the PID controller. This primarily clears the I and D terms, clears the previous sensor state, and sets the target setpoint the the current position.

This is useful if the output system's state may have changed since the last time the PID system output was applied. This is generally the case when the output system is in a manual control mode (such as a joystick), or was disabled and may have been physically moved. 

#### `setOutputRampRate(double rate)`
Set the maximum rate of change in a single time cycle. This can be useful to prevent really sharp responses in error conditions, often caused by D terms, large setpoint changes, or incorrectly set gains. Generally should be permissive, such that output is unhindered during expected operation. When set too high, this will destabilize the system and cause significant lag. 

#### `setOutputFilter(double strength)`
The output filter prevents sharp changes in the output, adding inertia and minimizing the effect of high frequency oscillations. This adds significant stability to systems with poor tunings, but at the cost of slower setpoint changes, disturbance rejection, and increased overshoot.

#### `enableLogger(PIDLogger function)`
#### `enableLoggerCSV()`
Tuning PIDs can often be difficult without precise insight into the math. This logger splits out various components used in the calculations, so they can be put into a spreadsheet or other tool for analysis. 

Because there's a wide variety of desirable logging methods, this class defines an interface to permit attaching arbitrary functions to capture the data, rather than trying to define a bunch of methods and pulling in dependencies. 

A basic example is provided, which simply puts CSV data records to the standard output.

