# DC_motor_controller_library

Esta biblioteca está sendo desenvolvida com o objetivo de proporcionar
um controle mais preciso dos motores DC com encoder. 


# Requisitos:
  - Arduino IDE instalada
  - Arduino (servem os modelos Uno, Mega, Nano, Leonardo, Micro e muitos outros)
  - Motor DC com encoder com 2 canais de sinal defasados em 90º (π/2 rad)
  - Ponte H L298N
  - Fonte de alimentação extra para a ponte H (que alimentará os motores)
  
# Instalando a bilbioteca:
Existem duas maneiras principais pelas quais você pode fexer isso. Uma delas é pelo Git (a que eu recomendo) e a outra é baixando o arquivo .zip e depois instalando pelo Arduino IDE. 
Recomendo instalar pelo Git para que fique mais fácil para você atualizar a biblioteca, bastando executar um git pull no terminal na pasta da biblioteca. Lembrabdo que, para que você consiga fazer o processo pelo git, o mesmo já deve estar devidamente configurado em sua máquina. Para isso, vá até a pasta libraries do Arduino:

- Via terminal no Linux, abra o terminal e digite o seguinte comando (ele entrará na pasta de biblitecas do Arduino IDE)
```console
	cd Arduino/libraries/
```
E depois:

```console
	git clone https://github.com/hugo-max-m-teixeira/DC_motor_controller.git
```

- Pela Arduino IDE (tanto Linux quanto Windows):
	1. Baixe o arquivo .zip (clique no botão verde "code" ou "código", que fica mais acima nessa mesma página e clique em baixar .zip);
	2. Abra a Arduno IDE;
	3. Na barra superior de menu, vá para sketch -> Incluir biblioteca -> Incluir biblioteca .zip;
	4. Selecione o arquivo que você baixou e cliue em "OK" ou "abrir".

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

# Debug 
Então, até aqui vimos o como "configurar", pelo menos em parte, o nosso motor na programação e na ponte H L298N. Agora vamos fazer alguns testes para verificar se tudo está correto e se todos os pinos foram informados na ordem correta.
Para iniciarmos os testes, vamos colocar o seguinte comando no void loop ()

```cpp
void loop(){	// Como talvez você já saiba, esse é o noso "loop", o que o nosso programa ficará "fazendo"
	mot.walk(50);	//Essa linha de código pede para o motor "mot" girar na velocidade de 50 RPM
}
```
Agora que você já tem o void setup () com as devidas configurações e o void loop () com o comando que pede para o motor girar em uma determinada velocidade, agora é o momento em que você deve enviar o código para o Arduino e testar o motor, observando se ele girou como esperado (calmamente e em uma velocidade quase constante).

## "O motor fica girando para frente e para trás como um doido" ou "O motor está girando na velocidade máxima"
- Mau contato em algum dos pinos do encoder (verificar antes do passo seguinte);
- Inverter fisicamente os pinos do encoder do motor que estão conectados ao Arduino, invertendo a posição deles na placa (ex.: o cabo que estava no pino 1 agora vai para o pino 2 e o cabo que estava no pino 2 vai para o pino 1);
## "O motor está girando para o lado errado"
- Inverter fisicamente os pinos do encoder do motor que estão conectados ao Arduino, invertendo a posição deles na placa (ex.: o cabo que estava no pino 1 agora vai para o pino 2 e o cabo que estava no pino 2 vai para o pino 1);
- Inverter os pinos de IN (responsáveis pelo controle de direção de rotação dos motores) da ponte H no Arduino (ex.: se o pino de IN1 estava conectado no pino 12 do Arduino e o IN2 estava no pino 13, agora o IN1 vai para o 13 e o IN2 vai para o 12);

## "O motor não está girando"
- Pode ser mau contato em alguma conexão dos pinos da ponte H. Verifique as conexões entre os pinos da sua placa Arduino e a ponte H.

## "Mesmo depois de ter tentado tudo o que você disse acima, há algo de errado com o motor :( "
 - Então, se você chegou até aqui e continua com problemas, o seu caso é mais específico. Não convém muito deixar textos muito grandes aqui nesse README só explicando cada possibilidade de erro. Logo, sugiro que acesse o site de debug da biblioteca: (site em desenvolvimento...) Carinha triste :|

<br><br>

# Métodos principais

<br>

## Aplicar força (PWM)
```cpp
mot.run(int pwm_desejado);
```
 - Aplica uma força (controlada por meio de PWM, simplesmente) no motor.
 - Observação interessante: Lemsre-se que o PWM, na maioria das placas Arduino comuns, é um valor que varia entre 0 (desligado) e 255 (força máxima).

 <br><br>

 ## Girar em velocidade constante (walk simples)
 ```cpp
mot.walk(float velocidade_desejada);
 ```
 - O motor tenta girar em uma velocidade constante (velocidade angular constante).
 - Argumentos (respectivamente):
    - velocidade_desejada = velocidade com a qual você desea que o motor gire (em RPM)
 - Observação interessante: Lembre-se que a biblioteca usa um sistema de controle baseado em PID para tentar manter a velocidade do motor constante. Apesar disso, peuqenos desvios de velocidade podem ocorrer devido a vários fatores externos.

<br><br>

## Girar algumas rotações em velocidade constante (walk com número de rotações)

```cpp
mot.walk(float velocidade_desejada, float numero_de_rotacoes);
```
 - O motor tenta girar determinada quantidade de rotações mantendo uma velocidade constante.
- Argumentos (respectivamente):
    - velocidade_desejada = velocidade com a qual você desea que o motor gire (em RPM)
    - numero_de_rotacoes = número de rotações que você deseja que o motor desenvolva (número de voltas que o eixo do motor deve girar)
- Observação interessante: Esse método é um pouco mais preciso do que o "walk simples" (o método anterior) por ter o número de rotações determinado diretamente pelo usuário.

<br><br>

## Parar (stop)
```cpp
mot.stop(unsigned int tempo_desejado);
```
 - Faz com que o motor tente permanecer parado (sem girar), mesmo com forças extenas agindo em seu eixo, por determinado tempo.
- Argumentos (respectivamente):
    - tempo_desejado = tempo em milissegundos (1 segundo = 1000 milissegundos) que o motor deve permanecer parado. Lembre-se que esse argumento só pode assumir valores positivos (unsigned).

<br><br>

## Reset (reiniciar)
```cpp
mot.reset();
```
 - Reinicia o motor (as variáveis de tempo, os valores do PID e a contagem do RPM)
 - Usado sempre que:
    - O motor terminar de realizar alguma movimentação e passar a realizar outra.



## Acelerar (em desenvolvimento)

```cpp
mot.accelerate(float velocidade_final, float aceleracao);   // Velocidade final em RPM e aceleração em RPM/segundo 
```

 - Acelera o motor até uma velocidade final (<strong>"método em desenvolvimento!!!"</strong>)
 - Argumentos (respectivamente): 
    - velocidade_final = velocidade final do motor (em RPM)
    - aceleracao = aceleração (em RPM/segudo)
 - Observação interessante: o tempo que o motor leva para acelerar até a velocidade final pode ser calculado, aproximadamente, por: <br>
    ```Tempo = Velocidade_final / Aceleração```






