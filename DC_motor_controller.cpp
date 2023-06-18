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
		if(pwm > 0){
			digitalWrite(in1, HIGH);
			digitalWrite(in2, LOW);
		}
		if(pwm < 0){
			digitalWrite(in1, LOW);
			digitalWrite(in2, HIGH);
		}
		pwm = abs(pwm);
			
			if((in1 == en) || (in2 == en)){	// If enable pin is also a direction control pin, PWM must be applied on a control pin that is on HIGH state
				if(digitalRead(in1)){			// If in1 is the control pin that is HIGH 
					analogWrite(in1, pwm);		// PWM is applied on in1
				} else if(digitalRead(in2)){	// If in2 is the control pin that is HIGH 
					analogWrite(in2, pwm);		// PWM is applied on in2
				}
			} else {												// Else, PWM is applied on enable pin 
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

void DC_motor_controller::computeRPM(){
	deltaTime = millis() - lastTime;

	if(deltaTime >= refreshTime){
		rpm = (pulses[0] * (60000.0 / (ppr * rr))) / deltaTime;
		pulses[0] = 0;

		if(is_counting){
			total_rot += rpm*deltaTime/60000.0;
		}
	}
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

void DC_motor_controller::setPIDconstants(float kp, float ki, float kd){
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

void DC_motor_controller::debugMaxVel(){
	direction = -1;
}

void DC_motor_controller::invertDirection(){
	int old_in1 = in1, old_in2 = in2, old_encoderPinA = encoderPinA, old_encoderPinB = encoderPinB;
	
	debugMaxVel();
	
	in1 = old_in2;
	in2 = old_in1;
}


int DC_motor_controller::computePID(float input, float sp, bool derivative){ // Compute and return the PID value.
	error = sp - input;                                   // Calcula o erro

	P = error * kp;                                       // Calcula a proporcional
	I += error * ki * (deltaTime / 1000.0);               // Calcula a integral
	derivative ? D = (error - lastError) * kd / (deltaTime / 1000.0) : D=0;

	applyIntegralLimit();
	pid = P + I + D;                                      // pid receba a soma de P, I e D

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
		computeRPM();                       // Calcula a velocidade
		pwm = computePID(rpm, sp, false);   // Calcula o valor do PID tendo como entrada a velocidade(rpm)
											// e o set point(sp)
		lastTime = millis();                // Atualiza o tempo
		sei();                              // Reativa todas as interrupções durante o cálculo
	}
	return pwm;                           // Retorna o valor do pwm (o mesmo do pid)
}

byte DC_motor_controller::doPID(float input, float sp){ // Looks like compulte_all, but it don't use RPM as input value
	deltaTime = millis() - lastTime;      // Tempo decorrido

	if(deltaTime >= refreshTime){         // Se o tempo deccorrido for maior ou igual ao tempo de refresh...
		cli();                              // Desativa todas as interrupções para o cálculo
		pwm = computePID(input, sp, false); // Calcula o valor do PID                       
		lastTime = millis();                // Atualiza o tempo
		sei();                              // Reativa todas as interrupções durante o cálculo
	}
	return pwm;                           // Retorna o valor do pwm (o mesmo do pid)
}

void DC_motor_controller::walk(float sp){	// Simply makes the wheel run by a constant velocity
	if(sp==0){
		run(0);
	}else{
		run(computeAll(sp));
	}
}

void DC_motor_controller::walk(float sp, float rot){
	bool can_run_local = true;
	if(rot == 0){
		if(sp==0){
			run(0);
		}else{
			run(computeAll(sp));
		}
	} else {
		ifNegativeAllNegative(sp, rot);
		long totalPulses=rot*ppr*rr;
		resetForGyrate();
		lastTime=millis();

		while(can_run_local){
			deltaTime = millis() - lastTime;        // Calcula o tempo decorrido desde a última execução
			if(deltaTime >= refreshTime){           // Se o tempo decorrido for maior ou igual ao tempo de refresh...
				cli();                                // Desativa todas as interrupções para o cálculo;
				deltaT=millis()-lastT;                // Calcula o tempo decorrido (para determinar o número de pulsos)
				Pulses=(deltaT*sp*ppr*rr)/60000.0;    // Calcula a quantidade necessária da pulsos, de acordo com o tempo
				if(rot > 0)  pwm = computePID(pulses[1],Pulses, true);   // Calcula o PID, de acordo com o número real de pulsos e a quantidade calculada
				else         pwm = computePID(-pulses[1],-Pulses, true); // Calcula o PID, de acordo com o número real de pulsos e a quantidade calculada
				lastTime = millis();                  // Atualiza o tempo, quando ocorreu essa execução
				sei();                                // Reativa todas as interrupções
			}
			
			run((rot > 0) ? pwm : -pwm);

			if(rot > 0){
				can_run_local = (pulses[1] < totalPulses)? true : false;
			} else {// rot < 0
				can_run_local = (pulses[1] > totalPulses)? true : false;
			}
		}
		lastT=millis();
		pulses[1]=0;
		while((millis() - lastT) <=100){        //Stop!
			deltaTime = millis() - lastTime;      // De acordo como tempo
			if(deltaTime >= refreshTime){
				cli();                              // Desativa todas as interrupções durante o cálculo;
				pwm = computePID(pulses[1]*2.0,0, true);
				lastTime = millis();
				sei(); // Reativa todas as interrupções
			}
			run(pwm);
		}
		resetForGyrate();
		lastError = 0;
	}
}

void DC_motor_controller::resetForGyrate(){
	deltaT=0; lastT=millis(); Pulses=0; pulses[1]=0; I=0; D=0; lastError=0; lastTime=millis(); rpm=0; deltaTime=0; //lastError = error
	can_run=true; pwm = 0; pulses[0] = 0; // Reset the pulses for the PWM counter
	run(0);
}

void DC_motor_controller::reset(){
	resetForGyrate();
}

bool DC_motor_controller::canRun(){
	return can_run;
}

void DC_motor_controller::gyrate(float sp, float rot=0){
	if(rot == 0){
		walk(sp, 0);
		can_run = false;
	} else {
		ifNegativeAllNegative(sp, rot);
		long totalPulses=rot*ppr*rr;
		deltaTime = millis() - lastTime;   // De acordo como tempo
		if(deltaTime >= refreshTime){
			cli(); // Desativa todas as interrupções durante o cálculo;
			deltaT=millis()-lastT; // Calcula o tempo decorrido
			Pulses=(deltaT*sp*ppr*rr)/60000.0; // Calcula a quantidade necessária da pulsos
			if(rot>0)   pwm = computePID(pulses[1],Pulses,true);
			else        pwm = computePID(-pulses[1],-Pulses,true);
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
void DC_motor_controller::stop(unsigned int t=0, int vel=0){
	t = (vel == 0) ? 100 : (vel*2); 				// If vel is 0, t is 100. If so, t change to vel*2
	unsigned long lastT_local = millis();
	while((millis() - lastT_local) < t){     		// For the time "t"...
		deltaTime=millis() - lastTime;
		if(deltaTime >= refreshTime){         		// If it's time to compute...
			cli();                              		// Desativa todas as interrupções durante o cálculo;
			pwm = computePID(pulses[1]*1.5,0, true);
			lastTime = millis();                		// Update lastTime
			sei();                             		// Reativa todas as interrupções
		}
		run(pwm);
	}
	run(0); // Turn off the motor
}

void DC_motor_controller::stop_both(int vel=0){
	int t = (vel == 0) ? 100 : (vel*2); 	// If vel is 0, t is 100. If so, t change to vel*2
	deltaTime=millis() - lastTime;
	if(deltaTime >= refreshTime){         // If it's time to compute...
		cli();                              // Desativa todas as interrupções durante o cálculo;
		pwm = computePID(pulses[1]*1.5,0, true);
		lastTime = millis();                // Update lastTime
		sei();                              // Reativa todas as interrupções
	}
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
