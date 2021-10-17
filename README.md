# DC_motor_controller_library

Esta biblioteca está sendo desenvolvida com o objetivo de proporcionar um controle mais preciso dos motores DC com encoder. 


# Requisitos:
  Arduino IDE instalada
  Arduino (servem os modelos Uno, Mega, Nano, Leonardo, Micro e muitos outros)
  Motor DC com encoder com 2 canais de sinal defasados em 90º (π/2 rad)
  Ponte H L298N

# Como usar essa biblioteca?

#1: instancie um objeto  (um motor)

DC_motor_controller motor; // "motor" é o nome que usaremos a partir de agora para nos referirmos ao motor físico.

# Já dei um nome para o meu motor, o que faço agora?
#2: Agora vamos para o void setup () do nosso programa 

void setup(){
  // primeiro devemos informar em quais pinos do Arduino estão conectados os pinos de controle da ponte H (IN, IN e EN)
  motor.hBridge(7,8,9); 
  
  // OK. Agora precisamos informar quais são os pinos de sinal que vêm do encoder do motor (primeiro o pino de interrupção)
  motor.setEncoderPin(2, 4); // Encoder conectado aos pinos 2 (canal A, interrupção) e 4 (canal B)
}

