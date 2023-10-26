/*
   Código de exemplo da biblioteca para controle de motores DC com encoder
   Disponível em: https://github.com/hugo-max-m-teixeira/DC_motor_controller
   By: Ronaldy L. França (https://github.com/ronaldylf) and Hugo Max M. Teixeira (https://github.com/hugo-max-m-teixeira)
   Date: 04/22023
*/

#include <DC_motor_controller.h>

DC_motor_controller motor;

// pinos da ponte H
#define IN1 6
#define IN2 7
#define EN 8

// pinos do encoder
#define pin_encoder_interrupcao 2 // cada arduino tem pinos próprios para interrupção (checar pinout do arduino utilizado)
#define pin_encoder_secundario 4 // pode ser qualquer outro pino digital

// variáveis de controle interno
#define RR 21 // motor JGA25: 21 // motor maior (JGB37): 30
#define kp 1.4
#define ki 0.9
#define kd 0.05

void interrupt_motor () {   // Função interrupt_motor
  motor.isr();              // Chama o método isr(), que realiza a contagem do pulso
}

void setup() {
  // Pinos usados:
  motor.hBridge(IN1, IN2, EN); // pinos da ponte H
  motor.setEncoderPin(pin_encoder_interrupcao, pin_encoder_secundario); // pinos do encoder (sempre o pino de interrupção vem primeiro)
  attachInterrupt(digitalPinToInterrupt(pin_encoder_interrupcao), interrupt_motor, FALLING); // ativa a interrupção para contagem dos pulsos

  // Dados do motor:
  motor.setRR(RR); // razão da caixa de redução

  // Constantes para o sistema de controle PID:
  motor.setPIDconstants(kp, ki, kd); // constantes do PID

  // Inicialização do motor
  motor.setPins(); // configura os pinos e variáveis passados nas funções anteriores
  motor.stop(); // Para o motor, garante que ele fique desligado.
}

void loop() {
  int velocidade = 50; // velocidade em RPM
  int rotacoes = 1; // rotações
    
    // Método para fazer o motor andar em uma velocidade constante por algumas rotacoes.
    // Para dar rotacoes para trás, ambos os valores (velocidade e rotações) devem ser negativos
    
    motor.walk(velocidade, rotacoes);

    int tempo_parado = 2500;
    
    motor.stop(tempo_parado); // método para parar o motor e deixá-lo parado por algum tempo em millisegundos
}
