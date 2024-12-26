
# FeedForwards Rundown

## kS
The Static FeedForward value represents a system's friction and electrical losses which can cause your system to not move at low output values. A Ks value offsets this, ensuring that even the smallest calculated PID values can effectively move the system to reduce the error.

As a result, a good kS improves setpoint tracking, while allowing for less aggressive P and I values.

kS affects the output as `output += ks*signof(error)`.

#### Finding kS
The easiest way to find your kS is experimentally; Because it represents the smallest value that moves your system, you can simply watch your encoder while slowly increasing the output manually. Once the system moves, you have a baseline to experiment with.

In practice the kS should be the largest value you can find that *does not* move your system; Once the kS moves the system on it's own, it's too high.

Once you've gotten your kS rougly calculated, test it with a simple P system using a low gain; If the system oscillates around the setpoint, reduce your kS slightly. If the system doesn't move at all, consider increasing it slighty. 

For systems that deal with gravity, you will want find the kG and kCos values before trying to find kS. By it's nature, kS *should* be identical in both forward and reverse outputs; If this is not the case, then one direction is fighting gravity, and the other assisted by it. Adjusting the kg or kCos by ~1/4 of the difference should fix this, or you can just pick the lower of the kS values.


## kG
The Gravity constant represents the weight of your system, and the output needed to resist against gravity. For elevator-type systems, this is usually just a fixed value added to the output. However, if your system's weight or leverage changes during travel (such as cascaded multi-rail systems), you may need a more complicated kG calculation.

#### Finding kG
Simply apply a motor output such that the system holds steady at some point in travel. Read the output, and that's kG. It affects the output as `output += kg`.

This can be difficult to figure out with manual driving though. The easiest method to find kG is remove your other FF values, set a relatively low P (just enough to get the system moving), and set a setpoint near the top or middle of your system. This will settle out somewhere in the middle of travel, and the output applied is your kG.


## kCos
This is a common variant of the Gravity Constant for rotational arms and pivoting systems. In this case, the output needed to hold against gravity changes in a predictable way: It's greatest when held horizontally, zero when directly above/ below the pivot, negative if it flips over the pivot. The forces match a `cos()` mathematical function, hence the name of the constant. It affects the output as `output += kCos * cos(current angle)` . 

Note, that because this is using a mathematical function, units and orientation matter! The `cos` function must operate on Radians, which may require a conversion if you prefer to use Degrees. It also requires the sensor mapping correlate to degrees/radians, increases in an upward direction, and that zero represents a horizontal. 

For systems where the reference angle does not match the angle between pivot and center of gravity. In these systems, you may need to apply an offset to generate correct output.

Note, that *technically* this reads the current angle, making it a *feedback* term, not a *feedforward* one. A true feed-forward would base the output on the *setpoint* angle. However, this form is much more robust in many cases, and better handles large setpoint changes.  

#### Finding kCos
Similar to kG, finding kCos involves holding the system stable against gravity, and recording the output. However, it's a bit trickier to find experimentally. To do so, it's easiest to remove any other FF values, set a kCos of zero, and set a reasonable P value.

Set a setpoint at zero, and see where the system falls. It *should* be somewhere near to horizontal. Go ahead and add the provided output value as your kCos value, and repeat this process. 

Within a couple iterations, the arm should be held horizontal, and the output stable. To properly test your kCos, set all other values to zero, and run the system using only kCos; The arm should hold it's position at all points in the rotation without falling or rising.


## Improving your systems further
Beyond these simple terms, feedforwards can be tricky to empirically sort out, and need a "system identification" tool and more advanced tooling to calculate. They also require additional setup and process knowledge that's not suitable for a streamlined PID library. 

#### Motion Profiling
a PID's biggest weakness is that when the setpoint jumps, the math involved results in big, instantaneous output changes and momentum. This is a big reason you get PIDs that have overshoot.  

While these jumps can be mitigated somewhat with internal tricks, motion profiling is the real solution. This operation cuts up one big setpoint jump into a set of small, predictable setpoints over time. 

Since the PID now is never making big jumps, the error is always smaller and more predictable; This usually removes overshoot entirely, allows you to have good results with much less tuning, and allows more aggressive PIDs to achieve stable tracking.

#### kVelocity + kAccelleration
Beyond being just for "rate" controlled systems, kVelocity actually applies to all systems, and can improve performance. After all, a change of position over a time period *is* a velocity.

However, making good use of this requires knowledge of the target velocity at each point in time, which is best served up by a motion profile.

kAccelleration is much the same. It allows even more accurate handling of the velocity changes for extremely precise responses.








