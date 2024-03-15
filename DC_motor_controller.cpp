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

float DC_motor_controller::getRPM(){
	return rpm;
}

int DC_motor_controller::getPWM(){
	return pwm;
}

unsigned int DC_motor_controller::getRefreshTime(){
	return refreshTime;
}

void DC_motor_controller::computeRPM(){
	deltaTime = millis() - lastTime;

	if(deltaTime >= refreshTime){
		rpm = (pulses[0] * (60000.0 / (ppr * rr))) / deltaTime;;
		pulses[0] = 0; 

		if(is_counting){
			total_rot += rpm*deltaTime/60000.0;
		}
	}
}

void DC_motor_controller::setPIDconstants(float kp, float ki, float kd){
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
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


int DC_motor_controller::computePID(float input, float sp, float error_coeficient){ // Compute and return the PID value.
	error = sp - input;                                   // Calcula o erro
	
	error *= error_coeficient;

	P = error * kp;                                       // Calcula a proporcional
	I += error * ki * (deltaTime / 1000.0);               // Calcula a integral
	D = (error - lastError) * kd / (deltaTime / 1000.0);

	applyIntegralLimit();
	
	//Serial.println("P: " + String(P) + '\t' + "I: " + String(I) + '\t' + "D: " + String(D) /*+ '\t' + "Delta time: " + String(I * 10.0)*/);
	
	pid = P + I + D;                                      // pid receba a soma de P, I e D
	//Serial.println("PID: " + String(pid) + '\n');

	lastError = error;                                    // Erro anterior = erro atual

	return pid;                                           // Retorna o valor do pid
}

void DC_motor_controller::applyIntegralLimit(){
	if(I > maxI) I=maxI;
    if(I < -maxI) I=-maxI;
}

int DC_motor_controller::computeAll(float sp){
	deltaTime = millis() - lastTime;      // Tempo decorrido

	if(deltaTime >= refreshTime){        	// Se o tempo deccorrido for maior ou igual ao tempo de refresh...
		cli();                              // Desativa todas as interrupções para o cálculo
		computeRPM();                       // Calcula a velocidade atual
		
		if(can_accelerate){ // Considerando a velocidade inicial = 0
			
			uint64_t elapsed_time = millis() -  lastTime_accel;
			int actual_vel = default_acceleration * elapsed_time/1000.0;
			
			if(actual_vel >= sp){
				can_accelerate = false;
			} else {
				sp = actual_vel;
			}
		} 
		
		pwm = computePID(rpm, sp, 1);   // Calcula o valor do PID tendo como entrada a velocidade(rpm)
											// e o set point(sp)
		//Serial.println(String(sp) + '\t' + String(rpm) + '\t' + String(error));
		
		lastTime = millis();                // Atualiza o tempo
		sei();                              // Reativa todas as interrupções após o cálculo		
	}
	return pwm;                           // Retorna o valor do pwm (o mesmo do pid)
}

byte DC_motor_controller::doPID(float input, float sp){ // Looks like compulte_all, but it don't use RPM as input value
	deltaTime = millis() - lastTime;      // Tempo decorrido

	if(deltaTime >= refreshTime){         // Se o tempo deccorrido for maior ou igual ao tempo de refresh...
		cli();                              // Desativa todas as interrupções para o cálculo
		pwm = computePID(input, sp, 1); // Calcula o valor do PID                       
		lastTime = millis();                // Atualiza o tempo
		sei();                              // Reativa todas as interrupções durante o cálculo
	}
	return pwm;                           // Retorna o valor do pwm (o mesmo do pid)
}

void DC_motor_controller::walk(float sp){	// Simply makes the wheel run by a constant velocity
	if(sp==0)	run(0);
	else		run(computeAll(sp));
}

void DC_motor_controller::walk(float sp, float rot){
	bool can_run_local = true;
	if(rot == 0){
		if(sp == 0)	run(0);
		else		run(computeAll(sp));
	} else {
		reset();
		lastTime=millis();
		if(smooth){
			bool accel_triangle = ((pow(sp, 2)/(default_acceleration*60.0)) > rot) ? true : false;
			//Serial.println("Acceleration and deceleration space: " + String((pow(sp, 2)/(default_acceleration*60.0) > rot)));
			//Serial.println("rot: " + String(rot));
			Serial.println("Acceleration triangle: " + String(accel_triangle));
			if(!accel_triangle){
				while(can_run)	gyrate(sp, rot);
			} else { // RPM doesnt't reaches the maximun vel, only accelerates and decelerates
			
			}
		}
		
		lastT=millis();
		reset();
		stop_vel(sp);
	}
}

void DC_motor_controller::resetForGyrate(){
	deltaT=0; lastT=millis(); Pulses=0; pulses[1]=0; I=0; D=0; lastError=0; lastTime=millis(); rpm=0; deltaTime=0; //lastError = error
	can_run=true; 
	can_accelerate = true;	lastTime_accel = millis();
	pwm = 0; pulses[0] = 0; // Reset the pulses for the PWM counter
	elapsed_stop_time = 0;
	run(0);
	print("Motor reseted!");
}

void DC_motor_controller::reset(){
	resetForGyrate();
}

bool DC_motor_controller::canRun(){
	return can_run;
}

void DC_motor_controller::gyrate(float sp, float rot /*= 0*/){
	if(rot == 0){
		walk(sp, 0);
		can_run = false;
	} else {
		ifNegativeAllNegative(sp, rot);
		long totalPulses=rot*ppr*rr;
		deltaTime = millis() - lastTime;   // De acordo como tempo
		
		cli(); // Desativa todas as interrupções durante o cálculo;
		
		deltaT=millis()-lastT; // Calcula o tempo decorrido
		
		if(deltaTime >= refreshTime){
		
			if(can_accelerate){ // Considerando a velocidade inicial = 0
				
				uint64_t elapsed_time = millis() -  lastTime_accel;
				int actual_vel = default_acceleration * elapsed_time/1000.0;
				
				
				
				Pulses=(pow(elapsed_time, 2) * default_acceleration       *ppr*rr)/120000.0; // Calcula a quantidade necessária da pulsos	
				
				Serial.println("Pulses: " + String(Pulses)) + "\n";
				
				if(actual_vel >= sp){
					can_accelerate = false;
				} else {
					sp = actual_vel;
				}
			} else {
				Pulses=(deltaT*sp*ppr*rr)/60000.0; // Calcula a quantidade necessária da pulsos			
			}
			
			//Serial.println("Set Point: " + String(sp) + '\t' + "RPM: " + String(rpm) + '\t' + "error: " + String(error) + "\n");
		
			
			
		
			
			
			
			if(rot>0)   pwm = computePID(pulses[1],Pulses, pulses_error_coeficient);
			else        pwm = computePID(-pulses[1],-Pulses, pulses_error_coeficient);
			
			lastTime = millis();
			
			sei(); // Reativa todas as interrupções
		}
		
		run((rot>0) ? pwm : -pwm);
		
		if(rot>0){
			can_run = (pulses[1] < totalPulses)? true : false;
		}else{
			can_run = (pulses[1] > totalPulses)? true : false;
		}
	}
}

void DC_motor_controller::stop(unsigned int t /*= 0*/){
	unsigned long lastT_local = millis();
	while((millis() - lastT_local) < t){     		// For the time "t"...
		deltaTime=millis() - lastTime;
		if(deltaTime >= refreshTime){         		// If it's time to compute...
			cli();                              		// Desativa todas as interrupções durante o cálculo;
			pwm = computePID(pulses[1],0, 1);
			lastTime = millis();                		// Update lastTime
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
		pwm = computePID(pulses[1],0, 1);
		sei();                              // Reativa todas as interrupções
		lastTime = millis();                // Update lastTime
		elapsed_stop_time += deltaTime;
	}
	can_stop = (elapsed_stop_time < time)? true : false;
	run(pwm);
}

void DC_motor_controller::accelerate(float sp, float accel){
	float time_sec = sp / accel;
	unsigned long last_time_local = millis(), delta_time_local = 0;
	
	ifNegativeAllNegative(sp, accel);
	
	while(delta_time_local < (time_sec*1000)){
		delta_time_local = millis() - last_time_local;
		
		long vel = (delta_time_local/1000.0) * accel;
		
		byte pwm = computeAll(vel);
		
		run(pwm);		
	}
}

float DC_motor_controller::anti_inertia_time(float vel/* = 50*/){
  unsigned int time;

  if(vel < 0) vel = -vel; // Absolute value of vel
  time = vel * inertia_time_coeficient;
  if(time > max_anti_inertia_time) time = max_anti_inertia_time;
  return time;
}

void DC_motor_controller::startCounting(){
	is_counting = true;
	total_rot = 0;
}

void DC_motor_controller::stopCounting(){
	is_counting = false;
}

float DC_motor_controller::getRotations(){
	return total_rot;
}

void DC_motor_controller::ifNegativeAllNegative(float &val_1, float &val_2){
	if ((val_1 < 0) || (val_2 < 0)){ // Garante que os dois valores sejam negativos no caso de um dos valores ser negativo.
  		val_1 = -abs(val_1);
  		val_2 = -abs(val_2);
  	}
}

void DC_motor_controller::print (String text, bool new_line = true){
	if(show_logs){ 
		if(new_line) {
			Serial.println(text);
		} else {
			Serial.print(text);
		}
	}
}
