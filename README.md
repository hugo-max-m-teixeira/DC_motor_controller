<title>
	teste
</title>


# DC_motor_controller_library

Esta biblioteca está sendo desenvolvida com o objetivo de proporcionar
um controle mais preciso dos motores DC com encoder. 


# Requisitos:
  - Arduino IDE instalada
  - Arduino (servem os modelos Uno, Mega, Nano, Leonardo, Micro e muitos outros)
  - Motor DC com encoder com 2 canais de sinal defasados em 90º (π/2 rad)
  - Ponte H L298N

# Como usar essa biblioteca?

Antes de tudo, precisamos incluir a biblioteca em nosso código

```cpp
// Incluindo a biblioteca
#include <DC_motor_controller.h>

// Agora dê um nome ao seu motor (instancie um objeto)
DC_motor_controller mot; 	// "mot" é o nome que usaremos a partir de agora
							//para nos referirmos ao motor físico.
```
# Já dei um nome para o meu motor, o que faço agora?
 Agora vamos para o void setup () do nosso programa 
```cpp
void setup(){
    // primeiro devemos informar em quais pinos do Arduino
    // estão conectados os pinos de controle da ponte H (IN, IN e EN)
    mot.hBridge(7,8,9); 
    // OK. Agora precisamos informar quais são os pinos de sinal
    // que vêm do encoder do motor (primeiro o pino de interrupção)
    mot.setEncoderPin(2, 4); 	// Encoder conectado aos pinos
                                // 2 (canal A, interrupção) e 4 (canal B)
    
    // Agora, considerando que o motor usado possue uma caixa de redução,
    // devemos informar qua a razão dessa caixa de redução, essa informação
    // pode ser obtida nas especificações técnicas do motor usado 
    // obs.: Caso o sensor encoder esteja acoplado diretamente ao eixo da roda,
    // considere essa razão como sendo 1
    mot.setRR(30);    // Razão da caixa de redução é 30
    
    // O sensor encoder envia pulsos pelos pinos de sinal conforme o eixo
    // do motor gira em rorno de si mesmo, no método setPPR() devemos
    // informar quantos pulsos o sensor encoder envia para cada rotação do
    // eixo conectado a ele. Essa informação pode ser obtida nas especificações
    // técnicas do encoder. OBS.: caso esse método não seja chamado, a biblioteca
    // considerará o valor do PPR (pulses per revolution) como sendo 11
    
    mot.setPPR(11); // 11 pulsos que o encoder envia para cada volta dada em
                    // torno de seu eixo
    
    // Realizamos as configurações necessárias para os pinos informados
    mot.setPins();
    
    // Agora precisamos ativar a interrupção externa para contagem dos pulsos
    attachInterrupt(digitalPinToInterrupt(2), interrupt_motor, FALLING);
    // Nessa função "digitalPinToInterrupt()" deve ser informado o pino do canal A do encoder
    
}

```

# Criando a interrupção para contagem dos pulsos
Quando ativamos a interrupção com o comando attachInterrupt() chamamos uma função de nome interrupt_motor. Agora precisamos criar essa função em nosso código

```cpp
void interrupt_motor (){    // Função interrupt_motor
    mot.isr();              // Chama o método isr(), que realiza a contagem do pulso
}
```

## Como está nosso programa até aqui:

```cpp
// Incluindo a biblioteca
#include <DC_motor_controller.h>

// Agora dê um nome ao seu motor (instancie um objeto)
DC_motor_controller mot; 	// "mot" é o nome que usaremos a partir de agora
							//para nos referirmos ao motor físico.

void interrupt_motor (){    // Função interrupt_motor
    mot.isr();              // Chama o método isr(), que realiza a contagem do pulso
}

void setup(){
    mot.hBridge(7,8,9);         // Pinos da ponte H
    mot.setEncoderPin(2, 4); 	// Pinos do encoder
    mot.setRR(30);    // Razão da caixa de redução é 30
    mot.setPPR(11); // 11 pulsos que o encoder envia para cada volta dada em
                    // torno de seu eixo
    mot.setPins();

    // Interrupção externa
    attachInterrupt(digitalPinToInterrupt(2), interrupt_motor, FALLING);
}

```
