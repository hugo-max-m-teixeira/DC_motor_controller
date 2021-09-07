/*
	Autor: Hugo Max M. Teixeira
	Data: 05/2021

	Elsta biblioteca tem por objetivo realizar o controle com maior precisão de motores com sensor encoder;

*/

#ifndef DC_motor_controller_h
#define DC_motor_controller_h

#include <Arduino.h>

class DC_motor_controller{
  public:
  	// Set_pins:
    void hBridge(uint8_t in1, uint8_t in2, uint8_t en);	// Pinos da ponte H L298N responsáveis pelo motor
    void setEncoderPin(uint8_t pinA, uint8_t pinB);		// Pinos do encoder do motor

    // Set technical features:
    void setRefreshTime(unsigned long t);				// Refresh time for wakl and gyrate, default is 50, but you can change it with this function
    void setPPR(uint16_t ppr);							// Pulses per revolution, default is 11, but you can change it with this function.
    void setRR(float rr);								// Rotation ratio
    void setPIDconstants(float kp, float ki, float kd);	// PID constants (KP, Ki and KD), para mudar os valores das constantes do PID conforme a necessidade
    void setMaxI(int max);

    // Setting pins:
    void setPins();	// Configura os pinos usados pelo programa

    // Actions:
    void run(int pwm);					// Apply a simple pwm on the motor
    void walk(float sp);
    void walk(float sp, float rot);	// Motor simple walk - For only one motor and it uses While
    void gyrate(float sp, float rot=0);	// Motor gyrate - For one or two motors and needs be into a while

    // Others...
    void isr();
    void computeRPM();
    float getRPM();
    void resetPID();
    bool canRun();
    void resetForGyrate();
    int getPWM(); // Retorna o PWm aplicado aos motores

    volatile long int pulses[2] = {0, 0}; // pulses[0] - para o RPM, pulses[1]- rotação

//private:
    int maxI = 200;
    void applyIntegralLimit();
    unsigned long lastTime = 0, deltaTime, refreshTime=50;
    uint8_t encoderPinA, encoderPinB;
    float ppr=11, rr, rpm;
    float kp=1.8, ki=1, kd=0.15, P=0, I=0, D=0, pid;	// Valores padrão para as constantes do PID;
    int pwm=0;
    float error, lastError = 0;
    int computePID(float input, float sp, bool derivative);
    int computeAll(float sp);
    uint8_t in1, in2, en;
    bool can_run=false;
    uint16_t deltaT=0, lastT; // Controle de tempo e pulsos do métodp gyrate
    long Pulses=0;

};

#endif
