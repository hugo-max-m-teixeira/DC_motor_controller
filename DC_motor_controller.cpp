/*
	Autor: Hugo Max M. Teixeira
	Data: 05/2021

	Esta biblioteca tem por objetivo realizar o controle com maior precisão de motores com sensor encoder;

*/

#include <DC_motor_controller.h>


void DC_motor_controller::hBridge(uint8_t in1, uint8_t in2, uint8_t en){
	this-> in1 = in1;
	this-> in2 = in2;
	this-> en  =  en;
}

void DC_motor_controller::hBridge(uint8_t in1, uint8_t in2){
	hBridge(in1, in2, in2);
}

void DC_motor_controller::setPins(){
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(en, OUTPUT);
	pinMode(encoderPinA, INPUT);
	pinMode(encoderPinB, INPUT);
}

void DC_motor_controller::run(int pwm){
	if(pwm > 255) pwm = 255;
	if(pwm < (-255)) pwm = (-255);
	
	//Serial.println("Running at PWM: " + String(pwm));

	if(pwm == 0){
		digitalWrite(in1, LOW);
		digitalWrite(in2, LOW);
	} else {
		bool forward = (pwm > 0) ? true : false;
	
		pwm = abs(pwm);
		if((in1 == en) || (in2 == en)){	// If enable pin is also a direction control pin, PWM must be applied on a control pin that is on HIGH state
			if(forward){
				analogWrite(in1, pwm);
				digitalWrite(in2, LOW);
			} else {
				digitalWrite(in1, LOW);
				analogWrite(in2, pwm);
			}	
		} else {
			if(forward){
				digitalWrite(in1, HIGH);
				digitalWrite(in2, LOW);
			} else {
				digitalWrite(in1, LOW);
				digitalWrite(in2, HIGH);
			}
			analogWrite(en, pwm);
		}
	}
}

void DC_motor_controller::setEncoderPin(uint8_t pinA, uint8_t pinB){
	this->encoderPinA = pinA;
	this->encoderPinB = pinB;
}

void DC_motor_controller::isr(){
	if(digitalRead(encoderPinB)){ // Sentido horário
		pulses[0]+= direction;
		pulses[1]+= direction;
	} else {  // Sentido anti-horário
		pulses[0]-= direction;
		pulses[1]-= direction;
	}
}

void DC_motor_controller::setRefreshTime(unsigned long t){
	this->refreshTime = t;
}

void DC_motor_controller::setPPR(uint16_t ppr){
	this->ppr = ppr;
}

void DC_motor_controller::setRR(float rr){
	this->rr = rr;
}

void DC_motor_controller::setMaxI(int max){
    this->maxI = max;
}

void DC_motor_controller::setPIDconstants(float kp, float ki, float kd){
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

void DC_motor_controller::setAcceleration(int acceleration){
	this->accelerationInRPMPerSecond = acceleration;
}

float DC_motor_controller::getRPM(){
	return rpm;
}

int DC_motor_controller::getPWM(){
	return pwm;
}

unsigned int DC_motor_controller::getRefreshTime(){
	return refreshTime;
}

float DC_motor_controller::getAcceleration(){
	return accelerationInRPMPerSecond;
}

void DC_motor_controller::debugMaxVel(){
	direction = -direction;
}

void DC_motor_controller::invertDirection(){
	int old_in1 = in1, old_in2 = in2, old_encoderPinA = encoderPinA, old_encoderPinB = encoderPinB;
	
	debugMaxVel();
	
	in1 = old_in2;
	in2 = old_in1;
}

float DC_motor_controller::computeRPM(long deltaPulses, unsigned long deltaTime){
	float rpm; 
	
	rpm = pulsesToRPM(deltaPulses, deltaTime);
	
	if(isCountingRotations){
		elapsedRotations += pulsesToRotations(deltaPulses);
	}
	
	return rpm;
}

int DC_motor_controller::computePID(float input, float sp, unsigned long deltaTime, bool reset/* = false*/, bool inputInPulses/* = false*/){ // Compute and return the PID value.
	float error, P, D , pid;
	float deltaTimeInSeconds = deltaTime/1000.0;
	//static float I = 0, lastError = 0;
	
	if(reset){
		I=input;
		lastError = 0;
	}
	
	error = sp - input;
	
	//Serial.print("---------- Delta time received by PID: ");
	//Serial.println(deltaTime);
	
	//Serial.println("|PID input: " + String(input) + "\tPID set point: " + String(sp) + "\tPID error: " + String(error) + "\tPID pulsesToRPM: " + "\tPID deltaTime: " + String((int)deltaTime) + String(pulsesToRPM(error, deltaTime)) + "\tPID input in pulses: " + String(inputInPulses) );
	
	if(inputInPulses){
		//Serial.println("PID Error before conversion:" + String(error));
		error = pulsesToRPM((long)error, deltaTime)*pulsesToRPMPIDConversionConstant;
	}
	
	//Serial.print("|| PID input: " + String(input) + "\tPID set point: " + String(sp) + "\tPID error: " + String(error) + "\tPID deltaTime: " + String(deltaTime));
	
	P = error * kp;
	I += error * ki * deltaTimeInSeconds;
	
	if(deltaTime != 0){
		D = (error - lastError) * kd / deltaTimeInSeconds;
	} else {
		D = 0;
	}
	
	//Serial.println("||| PID input: " + String(input) + "\tPID set point: " + String(sp) + "\tPID error: " + String(error));
	
	applyIntegralLimit(I);
	
	pid = P + I + D;     
	
	//Serial.println("PID error: " + String(error) + "\tPID output: " + String(pid));
	//Serial.println("\tPID output: " + String(pid));
	                           
	lastError = error;                                 

	return pid;                                          
}

void DC_motor_controller::applyIntegralLimit(float &I){
	if(I > maxI) I=maxI;
    if(I < -maxI) I=-maxI;
}

void DC_motor_controller::walkAtConstantVelocity(float sp, bool reset/* = false*/){
	unsigned long deltaTime;
	long deltaPulses;
	int pwm;
		
	if(reset){
		walkAtConstantVelocityTiming.lastTimeInMs = millis();
		lastPulses = pulses[1];
		canAccelerate = true;
		previousCanAccelerate = true;
		accelerateProcess(1,1,1, true); // Resets timing variables
		computePID(0,0,0, true); // Resets the PID cumulative variables
		return;
	}
	
	if(canAccelerate){
		// Need to accelerate first
		canAccelerate = accelerateProcess(sp, accelerationInRPMPerSecond, lastTime);
	} else {
		// Now can just walk at constant velocity
		if(!canAccelerate && previousCanAccelerate){// Resets lastTime before executing repeatedly.
			walkAtConstantVelocityTiming.lastTimeInMs = millis();
			lastPulses = pulses[1];
			computePID(lastPWM,0,0, true); // Resets the PID cumulative variables
			previousCanAccelerate = false;
		}
		
		deltaTime = walkAtConstantVelocityTiming.deltaTimeInMs();
		
		if(deltaTime >= refreshTime){
			//Serial.println("Walk delta time: " + String(deltaTime));
			cli(); // Disables all external interruptions
			
			deltaPulses = pulses[1] - lastPulses;
			
			rpm = computeRPM(deltaPulses, deltaTime);
			lastPulses = pulses[1];
			
			pwm = computePID(rpm, sp, deltaTime);
			
			sei(); // Enables external interruptions
			walkAtConstantVelocityTiming.lastTimeInMs = millis();
			run(pwm);
		}		
	}
}

void DC_motor_controller::walk(float sp, float rot/* = 0*/){
	if(rot == 0){
		if(sp == 0){
			run(0);
		} else {
			walkAtConstantVelocity(sp);
		}
	} else {
		unsigned long accelerationTimeInMs = abs(sp)/accelerationInRPMPerSecond * 1000;
		long lastDesiredPulses = rotationsToPulses(accelerationInRPMPerSecond*pow((float)accelerationTimeInMs/1000.0,2.0))/120;
	
		reset();
		
		if(smoothMode){
			//bool accel_triangle = ((pow(abs(sp), 2)/(accelerationInRPMPerSecond*60.0)) > abs(rot)) ? true : false;
			bool accel_triangle = isAccelerationTriangle(sp, rot, accelerationInRPMPerSecond);
			//Serial.println("Acceleration and deceleration space: " + String((pow(sp, 2)/(accelerationInRPMPerSecond*60.0) > rot)));
			//Serial.println("rot: " + String(rot));
			//Serial.println("Acceleration triangle: " + String(accel_triangle));
			//if(!accel_triangle){
			
			if(!accel_triangle){
				accelerate(sp, accelerationInRPMPerSecond);
				//Serial.println("millis(): " + String(millis()));
				//elapsedTimeSinseStart = millis() - startTime;
			}
		}
				
		//lastTime=millis()-refreshTime;
		unsigned long startTime = millis();
		//unsigned long elapsedTimeSinseStart = refreshTime;

		rot -= pulsesToRotations(pulses[1]);
		
		if(sp>0){
			pulses[1] -= lastDesiredPulses; // pulses error from previous accelerate() is considered and charged in pulses[1]
		} else {
			pulses[1] += lastDesiredPulses;
		}
		
		//Serial.println("millis(): " + String(millis()));	
		//Serial.println("Remeaning rotations to be done: " + String(rot));	
			
		gyrate(0, 0, 0, true); // Resets time variable of gyrate
		while(gyrate(sp, rot, startTime));
		
		reset();
	}
}

void DC_motor_controller::reset(){
	//Pulses=0;
	pulses[1]=0; lastTime=millis(); rpm=0; deltaTime=0; 
	canAccelerate = true;
	lastTime_accel = millis();
	pwm = 0; 
	pulses[0] = 0; // Reset the pulses for the PWM counter
	elapsed_stop_time = 0;
	run(0);	// Turn off the motor
	
	computePID(0,0,0, true); // Resets the PID cumulative variables
	walkAtConstantVelocity(0, true); // Resets timing variable
	accelerateProcess(1,1,millis(), true); // Resets the accelerateProcess() time variable
	gyrate(1,1,millis(), true);	// Resets the gyrate() time variable
	
	//print("Motor reseted!");
}

void DC_motor_controller::resetTimingVariables(){
	//Pulses=0;
	//lastTime=millis();
		
	computePID(0,0,0, true); // Resets the PID cumulative variables
	walkAtConstantVelocity(0, true); // Resets timing variable
	accelerateProcess(1,1,millis(), true); // Resets the accelerateProcess() time variable
	gyrate(1,1,millis(), true);	// Resets the gyrate() time variable
}

bool DC_motor_controller::gyrate(float sp, float rot, unsigned long startTime, bool reset/* = false*/){	
	unsigned long elapsedTimeSinseStart = millis() - startTime;
	//static unsigned long lastTime = millis();
	unsigned long deltaTime;
	long currentDesiredPulses;
	
	static long startPulsesValue = pulses[1];
	static float startingValueOfElapsedRotations = elapsedRotations;

	ifNegativeAllNegative(sp, rot);

	if(reset){ // This means first call of gyrate (deltaT = 0)
		gyrateTiming.lastTimeInMs = millis() - refreshTime;
		startPulsesValue = pulses[1];
		startingValueOfElapsedRotations = elapsedRotations;
		//Serial.println("Gyrate reset!");
		print("Gyrate reset! Refresh time: " + String(refreshTime));
		return;
	}

	deltaTime = gyrateTiming.deltaTimeInMs();   // De acordo como tempo (para o PID)
	
	//Serial.println("Gyrate delta time: " + String(deltaTime));
	print("Gyrate delta time: " + String(deltaTime));
	if(deltaTime >= refreshTime){ 
		
		print("Gyrate delta time (processed): " + String(deltaTime));
		//Serial.println("Gyrate delta time (processed): " + String(deltaTime));
		//can_accelerate = false;
		// To do: fazer o controle de aceleração inicial tendo como base o tempo, não o valor atual do RPM.
		//if(!can_accelerate) {
			//Serial.println("Elapsed time since start: " + (String)(elapsedTimeSinseStart));
			//Serial.println("Rotations done since start: " + (String)(pulsesToRotations(pulses[1])));
			
			float velocityInPulsesPerMs = rotationsToPulses(sp)/60000.0;
			currentDesiredPulses = velocityInPulsesPerMs*elapsedTimeSinseStart; // Calcula a quantidade necessária da pulsos	
			//currentDesiredPulses = (rotationsToPulses(sp)/60000.0)*long(elapsedTimeSinseStart);
			//Pulses -= last_gived_pulses;
		//}
		//Serial.println("Set point value: " + String(sp));
		//Serial.println("Rotations to pulses value: " + String(rotationsToPulses(sp)));
		//Serial.println("Real pulses value: " + String(pulses[1]) + "\t Desired pulses value: " + String(currentDesiredPulses)+ '\n');
		//print("Real pulses value: " + String(pulses[1]) + "\t Desired pulses value: " + String(currentDesiredPulses)+ '\n');
		cli();	// Disables all external interruptions

		if(rot>0)   pwm = computePID(pulses[1],currentDesiredPulses, deltaTime, false, true);
		else        pwm = -computePID(pulses[1],currentDesiredPulses, deltaTime, false, true);

		//
		
		//print("Motor PWM output: " + String(pwm));
		if(isCountingRotations){
			elapsedRotations = startingValueOfElapsedRotations + pulsesToRotations(pulses[1] - startPulsesValue);
		}
		gyrateTiming.lastTimeInMs = millis();
		sei(); // Enables external interruptions
	}
	
	run((rot>0) ? pwm : -pwm);
	
	long totalPulses = rotationsToPulses(rot);
	if(rot>0){
		return (pulses[1] < totalPulses)? true : false;
	}else{
		return (pulses[1] > totalPulses)? true : false;
	}
	
}

void DC_motor_controller::stop(unsigned int t /*= 0*/){

	if(t == 0){
		run(0);
		return;
	}

	unsigned long startTime = millis();
	long int startingPulsesValue = pulses[1];
	
	// For PID threading:
	unsigned int lastTime = millis(), deltaTime = 0;
	
	computePID(0,0,0, true); // Resets the PID cumulative variables
	
	while((millis() - startTime) < t){     		// For the time "t"...
		deltaTime = millis() - lastTime;
		if(deltaTime >= refreshTime){         		// If it's time to compute...
			cli();                              	// Desativa todas as interrupções durante o cálculo;
			pwm = computePID(pulses[1],startingPulsesValue, deltaTime, false, true);
			lastTime = millis();                	// Update lastTime
			sei();                             		// Reativa todas as interrupções
		}
		run(pwm);
	}
	run(0); // Turn off the motor
}

void DC_motor_controller::stop_vel(unsigned int vel /*= 0*/){
  stop(anti_inertia_time(vel));
}

void DC_motor_controller::stop_both(int time /*= 0*/){
	deltaTime=millis() - lastTime;
	if(deltaTime >= refreshTime){         // If it's time to compute...
		cli();                              // Desativa todas as interrupções durante o cálculo;
		pwm = computePID(pulses[1],0, deltaTime, false, true);
		sei();                              // Reativa todas as interrupções
		lastTime = millis();                // Update lastTime
		elapsed_stop_time += deltaTime;
	}
	can_stop = (elapsed_stop_time < time)? true : false;
	run(pwm);
}

void DC_motor_controller::accelerate(float sp, float accel){
	unsigned long startTime = millis()/*, elapsedTimeSinseStart = 0*/;
	//print("Acceleration started!");
	accelerateProcess(1,1,1, true); // Resets time variable
	while(accelerateProcess(sp, accel, startTime));
	//print("Acceleration ended!");
}

bool DC_motor_controller::accelerateProcess(float maxVelocity, float acceleration, unsigned long startTime, bool reset/* = false*/){
	//static unsigned long lastTime = millis();
	//static long startPulsesValue = pulses[1];
	//static float startingValueOfElapsedRotations = elapsedRotations;
	
	if(reset){
		accelerateTiming.lastTimeInMs = millis();
		accelerateStartPulsesValue = pulses[1];
		startingValueOfElapsedRotations = elapsedRotations;
		return false;
	}
	
	unsigned int accelerationTimeInMs = abs(maxVelocity) / acceleration * 1000;
	unsigned long elapsedTimeSinseStart = millis() - startTime;
	unsigned long deltaTimeInMs;
	
	deltaTimeInMs = accelerateTiming.deltaTimeInMs();
	
	if(deltaTimeInMs > refreshTime){
		long desiredPulses = accelerateStartPulsesValue + rotationsToPulses(acceleration*pow((float)elapsedTimeSinseStart/1000.0,2.0))/120;
		int pwm;

		//Serial.println("\tDesired pulses: " + String(desiredPulses) + "\t Elapsed time: " + String(elapsedTimeSinseStart) + "\t pulses[1]: " + String(pulses[1]));
		
		if(maxVelocity>0){
			pwm = computePID((float)pulses[1],(float)desiredPulses, deltaTimeInMs, false, true);
		} else {
			pwm = computePID((float)pulses[1],-(float)desiredPulses, deltaTimeInMs, false, true);
		}	
		
		run(pwm);
		lastPWM = pwm;
		
		if(isCountingRotations){
			elapsedRotations = startingValueOfElapsedRotations + pulsesToRotations(pulses[1] - accelerateStartPulsesValue);
		}
			
		accelerateTiming.lastTimeInMs = millis();
	}
	return (elapsedTimeSinseStart <= accelerationTimeInMs);
}

float DC_motor_controller::anti_inertia_time(float vel/* = 50*/){
  unsigned int time;

  if(vel < 0) vel = -vel; // Absolute value of vel
  time = vel * inertia_time_coeficient;
  if(time > max_anti_inertia_time) time = max_anti_inertia_time;
  return time;
}

void DC_motor_controller::startCounting(){
	isCountingRotations = true;
	elapsedRotations = 0;
}

void DC_motor_controller::stopCounting(){
	isCountingRotations = false;
}

float DC_motor_controller::getRotations(){
	return elapsedRotations;
}

void DC_motor_controller::ifNegativeAllNegative(float &val_1, float &val_2){
	if ((val_1 < 0) || (val_2 < 0)){ // Garante que os dois valores sejam negativos no caso de um dos valores ser negativo.
  		val_1 = -abs(val_1);
  		val_2 = -abs(val_2);
  	}
}

void DC_motor_controller::print (String text, bool new_line /* = true*/){
	if(show_logs){ 
		if(new_line) {
			Serial.println(text);
		} else {
			Serial.print(text);
		}
	}
}

unsigned long DC_motor_controller::pulsePerRotation(){
	return ppr*rr;
}

long DC_motor_controller::rotationsToPulses(float rot){
	return (rot*pulsePerRotation());
}

float DC_motor_controller::pulsesToRotations(float pulses){
	return (pulses/(float)(pulsePerRotation()));
}

float DC_motor_controller::pulsesToRPM(long pulses, unsigned long deltaTime){
	//Serial.println("------------ Delta Time is: " + String(deltaTime));
	if(deltaTime == 0){
		return 0;
		//Serial.println("------------ Delta Time is zero!!!!!!! ---------------------");
	}
	//Serial.println("------------ RPM is:" + String((float)pulses*60000.0/(conversionConstant*deltaTime)));
	return (float)pulses*60000.0/(pulsePerRotation()*deltaTime);
}

bool DC_motor_controller::isAccelerationTriangle(float velocity, float rotations, float accelerationInRPMPerSecond){
	return (pow(abs(velocity), 2)/(accelerationInRPMPerSecond*60.0)) > abs(rotations);
}
