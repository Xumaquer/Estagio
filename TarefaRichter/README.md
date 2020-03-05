# Código com a tarefa solicitada


Esse código consiste de uma tarefa solicitada como seleção de estágio realizada na IDE ESP-IDF, completa inclusive com as tarefas opcionais, as tarefas são:
* Iniciar;
* Apresentar a string "Hello world!" Na porta UART (USB);
* Aguardar entrada de usuário no UART para continuar o programa;
* Fazer LED respirar (BreathingLED);
* Fazer LED ligar/desligar usando o botão SW de um rotatory encoder;
* Iniciar Wifi;
* Disponibilizar página web (http-server) com botão para iniciar e parar LED;
* Disponibilizar na página web caixa de texto e botão para enviar texto para porta UART.

E então as seguintes tarefas são as opcionais :
* Adicionar na página web botão de Hibernar e mudar ESP32 para modo hibernação(deep sleep);
* Usar GPIO para acordar MCU do modo hibernação (deep sleep);
* Adicionar histórico de comando recebidos da porta UART na página web;
* Implementar Rotatory Encoder para definir frequência do LED.

Os pinos utilizados estão definidos nos #Define, o rotatory encoder está sendo lido com GPIOs utilizando uma lógica simples sem interrupções , o histórico
de mensagens adiciona ao final do texto HTML as ultimas 8 mensagens recebidas na UART que são armazenadas em variaveis, todas as tarefas de comunicação do 
servidor HTTP, utilizam apenas HTTP GET, o servidor não está configurado para utilizar HTTP POST, foi utilizado as bibliotecas de lWip para o servidor,
existem muitas bibliotecas no código, todas elas foram adicionadas ao longo de tentativas com diferentes métodos, algumas delas talvez não mais necessárias mas nenhuma delas foi removida.


See the README.md file in the upper level 'examples' directory for more information about examples.
