package io.github.tekdemo;

import java.util.function.DoubleSupplier;

/**
 * <p>
 * Simple to use PID implementation with extra features to make getting good closed loop control easy.
 * </p><p>
 * Source code documented to serve as a learning resource.
 * </p><p>
 * Minimal usage:
 * <pre>
MiniPID pid = new MiniPID(p,i,d);
//get sensor values
output= pid.getOutput(sensorvalue,target);
//use output values
</pre>
 * </p><p>
 * Recommended usage: 
 * <pre>
MiniPID pid = new MiniPID(p,i,d)
  // Define the systems reasonable output constraints
  .setOutputLimits(min,max)
  // Define the typical operating range for setpoint changes, which simplifies tuning
  .setSetpointRange(range)
  // Define a feedforward model for your system; Greatly simplifies tuning and improves performance
  .setF...(...)
  ; 
//get sensor values
output= pid.getOutput(sensorvalue,target);
//use output values
</pre>
 * </p><p>
 * @see https://github.com/tekdemo/MiniPID-Java
 * @see http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/improving-the-beginners-pid-introduction
 * </p>
 */
public class MiniPID{
	//**********************************
	// Class private variables
	//**********************************

	private double P=0;
	private double I=0;
	private double D=0;

	private double maxIOutput=0;
	private double errorSum=0;
	private double maxErrorSum=0;

	private double maxOutput=0; 
	private double minOutput=0;

	private double setpoint=0;

	private double lastActual=0;

	private boolean firstRun=true;
	private boolean reversed=false;

	private double outputRampRate=0;
	private double lastOutput=0;

	private double outputFilter=0;

	private double setpointRange=0;

	private double continuousMax=0;
	private double continuousMin=0;

	/** User-defined function for precise modeling of the controlled system
	 * Note, setpoint and error are constrained by any configuration options provided by the system.
	 */
	public interface FeedForwardLambda{
		public double compute(double setpoint, double actual, double error);
	}
	private FeedForwardLambda feedForwardLambda = (s,a,e)->{return 0.0;};

	/**
	 * Interface to define a logger function, so that you can validate certain internal 
	 * functions inside the PID. This is called after modifications the PID made to the 
	 * internal variables
	 */
	public interface PIDLogger{
		public void run(setpoint, actual, error, output,
		foutput,poutput,ioutput,doutput, errorSum);
	}
	/** Logger object; By default, it's unintialized to indicate logging is disabled */
	private PIDLogger logger = null;
	
	//**********************************
	// Constructor functions
	//**********************************
	
	/**
	 * Create a MiniPID class object. 
	 * See setP, setI, setD methods for more detailed parameters.
	 * @param p Proportional gain. Large if large difference between setpoint and target. 
	 * @param i Integral gain.  Becomes large if setpoint cannot reach target quickly. 
	 * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
	 */
	public MiniPID(double p, double i, double d){
		P=p; I=i; D=d;
		checkSigns();
		}

	//**********************************
	// Configuration functions
	//**********************************
	/**
	 * Configure the Proportional gain parameter. <br>
	 * This responds quickly to changes in setpoint, and provides most of the initial driving force
	 * to make corrections. <br>
	 * Some systems can be used with only a P gain, and many can be operated with only PI.<br>
	 * For position based controllers, this is the first parameter to tune, with I second. <br>
	 * For rate controlled systems, this is often the second after F.
	 *  
	 * @param p Proportional gain. Affects output according to <code>output+=P*(setpoint-current_value)</code>
	 */
	public MiniPID setP(double p){
		P=p;
		checkSigns();
		return this;
	}

	/**
	 * Changes the I parameter <br>
	 * This is used for overcoming disturbances, and ensuring that the controller always gets to the control mode. 
	 * Typically tuned second for "Position" based modes, and third for "Rate" or continuous based modes.
	 * </p><p>
	 * Affects output through <code>output+=previous_errors*Igain ;previous_errors+=current_error</code>
	 * </p><p>
	 * @see {@link #setMaxIOutput(double) setMaxIOutput} for how to restrict
	 *
	 * @param i New gain value for the Integral term
	 */
	public MiniPID setI(double i){
		if(i!=0){
			errorSum=errorSum*I/i;
			maxErrorSum=maxIOutput/i;
		}
		I=i;
		checkSigns();
		 // Implementation note: 
		 // This Scales the accumulated error to avoid output errors. 
		 // As an example doubling the I term cuts the accumulated error in half, which results in the 
		 // output change due to the I term constant during the transition.
		 return this;
	}

	/**
	 * Changes the D parameter
	 * <p>
	 * This term tends to oppose motion, with the following effects
	 * <list>
	 * <li>- When the system is moving toward the target, it provides a negative response, slowing the system and adding drag. 
	 * <li>- When the system is at/near the setpoint and disturbed away from it, it provides a sharp, strong response to oppose it.
	 * <li>- Adds a "startup kick" on setpoint changes.
	 * </list>
	 * </p><p>
	 * This tends to be a tricky term to tune; The correct D term provides stability and prevents overshoot. 
	 * However, if it's too big, you'll add significant instability to the system. 
	 * </p><p>
	 * Works most reliably when using input ramp rates and feed forwards, 
	 * so that setpoint changes between cycles are generally small, and the only large errors are disturbances.
	 * </p><p>
	 * Affects output through <code>output += -D*(current_input_value - last_input_value)</code>
	 * </p>
	 * @param d New gain value for the Derivative term
	 */
	public MiniPID setD(double d){
		D=d;
		checkSigns();
		return this;
	}

	/**
	 * Configure a simplified Velocity FeedForward
	 * <p>
	 * kS output according to <code>output+= (sign of error) * ks</code>.
	 * This value is determined by the smallest output that causes rotation. Can be set to 0 to ignore
	 * </p><p>
	 * kF output according to <code>output+=F*Setpoint</code> . 
	 * Because of this, a sensible default value is <code>maxOutput/maxAchievableVelocity</code>. 
	 * </p>
	 * @param ks Static feed forward output. 
	 * @param f Feed forward velocity gain. 
	 */
	public MiniPID setFVelocity(double ks, double kv){
		setF((s,a,e)-> Math.signum(e)*ks + s*v);
		return this;
	}

	/**
	 * Configure a simplified Gravity FeedForward
	 * <p>
	 * kS output according to <code>output+= (sign of error) * ks</code>.
	 * This value is determined by the smallest output that causes motion. Can be set to 0 to ignore
	 * </p><p>
	 * kg Affects output according to <code>output+=kg</code> . 
	 * This is simply the output applied to keep a system from falling down.
	 * </p>
	 * @param ks Static Feed Forward. 
	 * @param kg gravity feed forward
	 */
	public MiniPID setFGravity(double ks, double kg){
		setF((s,a,e)-> Math.signum(e)*ks + kg );
		return this;
	}

	/** Helper unit for defining feed forward angles */
	public enum AngularUnits{kRad,kDegrees}

	/**
	 * Configure a Gravity Feed Forward for pivot and arm systems 
	 * <p>
	 * kS output according to <code>output+= (sign of error) * ks</code>.
	 * This value is determined by the smallest output that causes motion. Can be set to 0 to ignore
	 * </p><p>
	 * kCos Affects output according to <code>output+=kcos*cos(current angle)</code> . 
	 * This negates the effect of gravity on the arm at all points in motion
	 * </p>
	 * @param ks Static Feed Forward. 
	 * @param kcos gravity feed forward
	 * @param units Units used for the calculation
	 * @param offset Offset between measured angle and the system's center of gravity
	 */
	public MiniPID setFArm(double ks, double kcos, AngularUnits units, double offset ){
		switch (units) {
			case kRad:
				setF((s,a,e)-> Math.signum(e)*ks + kcos*Math.cos(a + offset) );
				break;
			case kDegrees:
				setF((s,a,e)-> Math.signum(e)*ks + kcos*Math.cos(Math.toRadians(kcos + offset)) );
				break;
			}
		return this;
	}

	/**
	 * </p>
	 * Configure the FeedForward parameter using arbitrary function and system parameters <br>
	 * This allows for precise modelling of the expected beaviour of your system,
	 * allowing effective PID use on a wider variety of systems.
	 * </p><p>
	 * Before using this, consider if the existing setF functions cover your use case. 
	 * </p>
	 * @param ff Feed forward lambda, which would take (Setpoint, Sensor Actual, error) and return double
	 * @return
	 */
	public MiniPID setF(FeedForwardLambda ff){
		this.feedForwardLambda = ff;
		return this;
	}

	/** 
	 * Configure the PID object.
	 * See setP, setI, setD methods for more detailed parameters.
	 * @param p Proportional gain. Large if large difference between setpoint and target. 
	 * @param i Integral gain.  Becomes large if setpoint cannot reach target quickly. 
	 * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
	 */
	public MiniPID setPID(double p, double i, double d){
		P=p;D=d;
		//Note: the I term has additional calculations, so we need to use it's 
		//specific method for setting it.
		setI(i);
		checkSigns();
		return this;
	}

	/**
	 * Set the maximum output value contributed by the I component of the system
	 * This can be used to prevent large windup issues and make tuning simpler
	 * @param maximum. Units are the same as the expected output value
	 */
	public MiniPID setMaxIOutput(double maximum){
		// Internally maxErrorSum and Izone are similar, but scaled for different purposes. 
		// The maxErrorSum is generated for simplifying math, since calculations against 
		// the max error are far more common than changing the I term or Izone. 
		maxIOutput=maximum;
		if(I!=0){
			maxErrorSum=maxIOutput/I;
		}
		return this;
	}

	/**
	 * Specify a maximum output range. <br>
	 * When one input is specified, output range is configured to 
	 * <code>[-output, output]</code>
	 * @param output
	 */
	public MiniPID setOutputLimits(double output){
		setOutputLimits(-output,output);
		return this;
	}

	/**
	 * Specify a  maximum output.
	 * When two inputs specified, output range is configured to 
	 * <code>[minimum, maximum]</code>
	 * @param minimum possible output value
	 * @param maximum possible output value
	 */
	public MiniPID setOutputLimits(double minimum,double maximum){
		if(maximum<minimum) return this;
		maxOutput=maximum;
		minOutput=minimum;

		// Ensure the bounds of the I term are within the bounds of the allowable output swing
		if(maxIOutput==0 || maxIOutput>(maximum-minimum) ){
			setMaxIOutput(maximum-minimum);
		}
		return this;
	}

	/** 
	 * Set the operating direction of the PID controller
	 * @param reversed Set true to reverse PID output
	 */
	public MiniPID setDirection(boolean reversed){
		this.reversed=reversed;
		return this;
	}

	//**********************************
	// Primary operating functions
	//**********************************

	/**
	 * Configure setpoint for the PID calculations<br>
	 * This represents the target for the PID system's, such as a 
	 * position, velocity, or angle. <br>
	 * @see MiniPID#getOutput(actual) <br>
	 * @param setpoint
	 */
	public MiniPID setSetpoint(double setpoint){
		this.setpoint=setpoint;
		return this;
	}

	/**
	 * Calculate the output value for the current PID cycle.<br>
	 * @param actual The monitored value, typically as a sensor input.
	 * @param setpoint The target value for the system
	 * @return calculated output value for driving the system
	 */
	public double getOutput(double actual, double setpoint){
		double output;
		double Poutput;
		double Ioutput;
		double Doutput;
		double Foutput;

		this.setpoint=setpoint;

		// Do the simple parts of the calculations
		double error=setpoint-actual;

		// If we're in continous mode, wrap our error to better match the system,
		// and adjust the error sign to push us in the appropriate direction
		if(continuousMin != continuousMax){
			double continuousHalfRange;
			continuousHalfRange = (continuousMax-continuousMin)/2.0;
			error %= (continuousHalfRange*2);
			if(error>continuousHalfRange) error-=2*continuousHalfRange;
			if(error<-continuousHalfRange) error+=2*continuousHalfRange;
		}
		
		// Apply the setpoint range, which effectively constrains our error within a smaller range
		if (setpointRange != 0) {
			error = constrain(error,-setpointRange,setpointRange);
		}

		// Handle the feed forward, which provides an expected output for most systems
		Foutput = feedForwardLambda.compute(setpoint, actual, error);

		// Calculate P term
		Poutput=P*error;   

		// If this is our first time running this, we don't actually _have_ a previous input or output. 
		// For sensor, sanely assume it was exactly where it is now.
		// For last output, we can assume it's the current time-independent outputs. 
		if(firstRun){
			lastActual=actual;
			lastOutput=Poutput+Foutput;
			firstRun=false;
		}

		// Calculate D Term
		// This term tends to oppose motion, acting on the current rate of change.
		// When the other terms are pushing the system toward the setpoint, it acts as drag, preventing overshoot.
		// When the system is at setpoint and disturbed away from it, the D response will provide a 
		// sharp opposing force, providing stability to keep it at the setpoint.
		// Note that the standard form would look like this;
		//   Doutput = -D*(error-lasterror) ;
		// or 
		//   Doutput = D*((setpoint-sensor) - (lastsetpoint-lastsensor))
		// and can be rearranged to 
		//   Doutput = D*( (setpoint-lastsetpoint) + (lastsensor-sensor))
		// In most moments of operation, setpoint=lastsetpoint, reducing that term to zero. 
		// However, during setpoint changes, that term can be large, causing a large output spike
		// known as "derivitive kick", which is undesirable.
		// Instead, we use "derivitive on measurement" form, which simply sets the setpoint terms to 0,
		// removing this undesirable kick, while preserving the intended effects.
		Doutput= -D*(actual-lastActual);
		lastActual=actual;

		// The Iterm has a few conditions to ensure it's computed in sensible ways.
		if(minOutput!=maxOutput && !bounded(output, minOutput,maxOutput) ){
			// Our system is at maximum output, so additional error sum will not improve 
			// response, but will increase overshoot.
			// In this case, simply avoid adding the error sum.
		}
		else if(outputRampRate!=0 && !bounded(output, lastOutput-outputRampRate,lastOutput+outputRampRate) ){
			// As before; Our system's output ramp is constraining ouptut, so
			// as before, increasing error causes overshoot, but not improved immediate response.
		}
		else if(maxIOutput!=0){
			// This will be the typical case for most systems when I term is doing work. 
			// We keep the error sum within our defined limits to prevent I term from 
			// hitting unreasonable output.
			// These limits are set explicitly by setMaxIOutput() or implicity by setOutputLimits()
			errorSum=constrain(errorSum+error,-maxErrorSum,maxErrorSum);
		}
		else{
			// Suboptimal, and we don't want to be here. In this case, immovable objects
			// halting the system will cause massive I term windup, preventing corrective setpoint
			// changes.
			errorSum+=error;
		}

		//With errorSum figured out, we can now generate our I term response
		Ioutput=I*errorSum;
		if(maxIOutput!=0){
			Ioutput=constrain(Ioutput,-maxIOutput,maxIOutput); 
		}

		// And, finally, we can just add the terms up
		output=Foutput + Poutput + Ioutput + Doutput;

		// Restrict output to our specified output and ramp limits
		if(outputRampRate!=0){
			output=constrain(output, lastOutput-outputRampRate,lastOutput+outputRampRate);
		}
		if(minOutput!=maxOutput){ 
			output=constrain(output, minOutput,maxOutput);
			}
		if(outputFilter!=0){
			output=lastOutput*outputFilter+output*(1-outputFilter);
		}

		// Run a logger, if enabled.
		if(logger!=null){
			logger.run(setpoint, actual, error, output,
				Foutput,Poutput,Ioutput,Doutput,
				errorSum
			);
		}

		lastOutput=output;
		return output;
	}

	/**
	 * Calculate the output value for the current PID cycle.<br>
	 * In no-parameter mode, this uses the last sensor value, 
	 * and last setpoint value. <br>
	 * Not typically useful, and use of parameter modes is suggested. <br>
	 * @return calculated output value for driving the system
	 */
	public double getOutput(){
		return getOutput(lastActual,setpoint);
	}

	/**
	 * Calculate the output value for the current PID cycle.<br>
	 * In one parameter mode, the last configured setpoint will be used.<br>
	 * @see MiniPID#setSetpoint()
	 * @param actual The monitored value, typically as a sensor input.
	 * @return calculated output value for driving the system
	 */
	public double getOutput(double actual){
		return getOutput(actual,setpoint);
	}

	/**
	 * Resets the controller. This erases the I term buildup, and removes 
	 * D gain on the next loop.<br>
	 * This should be used any time the PID is disabled or inactive for extended
	 * duration, and the controlled portion of the system may have changed due to
	 * external forces.
	 */
	public MiniPID reset(){
		firstRun=true;
		errorSum=0;
		return this;
	}

	/**
     * Set the maximum rate the output can increase per cycle.<br>
     * This can prevent sharp jumps in output when changing setpoints or 
     * enabling a PID system, which might cause stress on physical or electrical
     * systems.  <br>
     * Can be very useful for fast-reacting control loops, such as ones 
     * with large P or D values and feed-forward systems.
     * 
	 * @param rate, with units being the same as the output
	 */
	public MiniPID setOutputRampRate(double rate){
		outputRampRate=rate;
		return this;
	}

	/** 
     * Set a limit on how far the setpoint can be from the current position
	 * <br>Can simplify tuning by helping tuning over a small range applies to a much larger range. 
	 * <br>This limits the reactivity of P term, and restricts impact of large D term
	 * during large setpoint adjustments. Increases lag and I term if range is too small.
	 * @param range, with units being the same as the expected sensor range. 
	 */
	public MiniPID setSetpointRange(double range){
		setpointRange=range;
		return this;
	}
	
	/**
	 * Tell the controller that min and max represent the same same physical value.
	 * <br>
	 * This is notably useful for position control of angular values. In many such systems, 
	 * 0 and 360 represent the same heading, but without special handling you'll often
	 * correct the error going in an inefficient direction.
	 * <br>
	 * Note, that restricting setpoints or sensor inputs to the specified range are not
	 * required. The controller wraps all values back within this range automatically.
	 * 
	 * @param minSensorValue Lower bound of the continuous range
	 * @param maxSensorValue Lower bound of the continuous range
	 */
	public MiniPID setContinuousMode(double minSensorValue, double maxSensorValue){
		this.continuousMin = minSensorValue;
		this.continuousMax = maxSensorValue;
		return this;
	}

	/**
	 * Disable the continous mode, and resume normal operation.
	 *  
	 */
	public MiniPID setContinousModeOff(){
		this.continuousMin=0;
		this.continuousMax=0;
		return this;
	}

	/**
     * Set a filter on the output to reduce sharp oscillations. <br>
	 * 0.1 is likely a sane starting value. Larger values use historical data
	 * more heavily, with low values weigh newer data. 0 will disable, filtering, and use 
	 * only the most recent value. <br>
	 * Increasing the filter strength will P and D oscillations, but force larger I 
	 * values and increase I term overshoot.<br>
	 * Uses an exponential wieghted rolling sum filter, according to a simple <br>
	 * <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre> algorithm.
	 * @param output valid between [0..1), meaning [current output only.. historical output only)
	 */
	public MiniPID setOutputFilter(double strength){
		if(strength==0 || bounded(strength,0,1)){
		outputFilter=strength;
		}
		return this;
	}


	//**************************************
	// Alternate closed loops for special cases
	//**************************************

	/**
	 * Configure a Bang-Bang output, returning the min or max output to correct the error. Replaces FeedForward functions.
	 * <p>
	 * This is useful for systems that only have on/off states, and respond slowly to changes.
	 * A common example is relay-controlled heaters and other thermostat systems.
	 * </p><p>
	 * Hysterisis allows you to define an error tolerance in which the output remains unchanged. 
	 * This helps prevent the system from switching too rapidly, which is often harmful to the systems involved.
	 * A large hysteris will cause overshoot, but too small will result in excess switching. 
	 * </p><p>
	 * When used, PID values are irrelivant, and recommended to be left at zero. 
	 * </p>
	 * @param hysterisis required change of error between state switches 
	*/
	public MiniPID setControlModeBangBang(double hysterisis){
		double output = minOutput;
		setF((s,a,e) -> {
			if(e > hysterisis) output = maxOutput;
			if(e < hysterisis) output = minOutput;
			return output;
		});
	}

	/**
	 * A simple tri state (on/off/reverse) control
	 * @param tolerance
	 * @return
	 */
	public MiniPID setControlModeTriState(double tolerance){
		setF((s,a,e) -> {
			if(e > tolerance) output = maxOutput;
			if(e < tolerance) output = minOutput;
			return maxOutput-minOutput;
		});
	}

	/**
	 * Configure an inverse-spring output, replacing the FeedForward functions. 
	 * <p>
	 * This responds strongly to small errors, but increasingly weakly as the error increases.
	 * This can generate fairly well behaved output when handling large, unconstrained setpoint
	 * changes or disturbances. However, it's more prone to oscillation around setpoints than a P term.
	 * </p><p>
	 * Most useful for tracking a continously moving setpoint where errors might be large.
	 * </p><p> 
	 * Recommended to use SetOutputRampRate() or an output filter to reduce oscillation at the setpoint.
	 * </p><p>
	 * When used, PID values will affect output normally, but I term is the most helpful to reduce potential standing errors.
	 * </p>
	 * @param gain spring constant k
	*/
	public MiniPID setControlModeInverseSpring(double gain){
		setF((s,a,e) -> {
			return gain * Math.sqrt(Math.abs(e)) * Math.signum(e);
		});
	}

	/** 
	 * <p>
	 * Configure a spring control system.
 	 * </p><p>
	 * Bouncy, uncontrolled motion. Recommended for systems under constant, rapid 
	 * setpoint changes or disturbances, where the goal is fun rather than actual control.
 	 * </p><p>
	 * Mostly here for demonstration purposes.
	 * </p>
	 */
	public MiniPID setControlModeSpring(double gain){
		setF((s,a,e) -> {
			return gain * e * Math.abs(e);
		});
	}

	//**************************************
	// Logging oriented functions
	//**************************************

	/** Enables logging with a simple CSV oriented output.
	 * This might be directly helpful for simple checking, 
	 * or put into a spreadsheet for detailed graphical analysis.
	 */
	public void enableLoggerCSV(){
		logger = (setpoint, actual, error, output, foutput,poutput,ioutput,doutput, errorSum) -> 
		System.out.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
			setpoint, actual, error, output,
			foutput,poutput,ioutput,doutput,errorSum
		);
	}

	/**
	 * Directly input a logger function for exporting to a different output stream or file.
	 * @param logger
	 */
	public void enableLogger(PIDLogger logger){
		this.logger = logger;
	}

	public void disableLogger(){
		this.logger = null; //unset the logger
	}

	//**************************************
	// Helper functions
	//**************************************

	/**
	 * Forces a value into a specific range
	 * @param value input value
	 * @param min maximum returned value
	 * @param max minimum value in range
	 * @return Value if it's within provided range, min or max otherwise 
	 */
	private double constrain(double value, double min, double max){
		if(value > max){ return max;}
		if(value < min){ return min;}
		return value;
	}  

	/**
	 * Test if the value is within the min and max, inclusive
	 * @param value to test
	 * @param min Minimum value of range
	 * @param max Maximum value of range
	 * @return true if value is within range, false otherwise
	 */
	private boolean bounded(double value, double min, double max){
		// Note, this is an inclusive range. This is so tests like
		// `bounded(constrain(0,0,1),0,1)` will return false.
		// This is more helpful for determining edge-case behaviour
		// than <= is.
		return (min<value) && (value<max);
	}

	/**
	 * To operate correctly, all PID parameters require the same sign
	 * This should align with the {@literal}reversed value
	 */
	private void checkSigns(){
		if(reversed){  // all values should be below zero
			if(P>0) P*=-1;
			if(I>0) I*=-1;
			if(D>0) D*=-1;
			if(F>0) F*=-1;
		}
		else{  // all values should be above zero
			if(P<0) P*=-1;
			if(I<0) I*=-1;
			if(D<0) D*=-1;
			if(F<0) F*=-1;
		}
	}
}