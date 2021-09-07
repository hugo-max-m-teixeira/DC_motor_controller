/*
	Autor: Hugo Max M. Teixeira
	Data: 05/2021

	Elsta biblioteca tem por objetivo realizar o controle com maior precisão de motores com sensor encoder;

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
    analogWrite(en, pwm);
  }
}

void DC_motor_controller::setEncoderPin(uint8_t pinA, uint8_t pinB){
  this->encoderPinA = pinA;
  this->encoderPinB = pinB;
}

void DC_motor_controller::isr(){
  if(digitalRead(encoderPinB)){ // Sentido horário
    pulses[0]++;
    pulses[1]++;
  } else {  // Sentido anti-horário
    pulses[0]--;
    pulses[1]--;
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
  }
}

float DC_motor_controller::getRPM(){
  return rpm;
}

int DC_motor_controller::getPWM(){
	return pwm;
}

void DC_motor_controller::setPIDconstants(float kp, float ki, float kd){
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
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

  if(deltaTime >= refreshTime){         // Se o tempo deccorrido for maior ou igual ao tempo de refresh...
    cli();                              // Desativa todas as interrupções para o cálculo
    computeRPM();                       // Calcula a velocidade
    pwm = computePID(rpm, sp, false);   // Calcula o valor do PID tendo como entrada a velocidade(rpm)
                                        // e o set point(sp)
    lastTime = millis();                // Atualiza o tempo
    sei();                              // Reativa todas as interrupções durante o cálculo
  }
  return pwm;                           // Retorna o valor do pwm (o mesmo do pid)
}

void DC_motor_controller::walk(float sp){
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
    while((millis() - lastT) <=200){        //Stop!
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
  }
}

void DC_motor_controller::resetForGyrate(){
  deltaT=0; lastT=millis(); Pulses=0; pulses[1]=0; I=0; D=0; lastError=error; lastTime=millis(); rpm=0; deltaTime=0;
  can_run=true; pwm = 0; pulses[0] = 0; // Reset the pulses for the PWM counter
  run(0);
}

bool DC_motor_controller::canRun(){
  return can_run;
}

void DC_motor_controller::gyrate(float sp, float rot=0){
    if(rot == 0){
        walk(sp, 0);
        can_run = false;
    } else {
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
