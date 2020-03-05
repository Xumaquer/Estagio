#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "esp_attr.h"
#include "soc/mcpwm_periph.h"
#include "driver/mcpwm.h"
#include "soc/rtc.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include <esp_http_server.h>
#include "protocol_examples_common.h"
#include "esp_spiffs.h"

#include "esp_vfs.h"


#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"

#define BLINK_GPIO CONFIG_BLINK_GPIO
#define LEDPiscante 23
#define BreathingLED 27
#define SwitchLED 26
#define HttpLED 25
#define EncoderCLK 12
#define EncoderSW 13
#define EncoderData 14
#define RedeWifi "SCHUMACHER"
#define Senha "DOWNLOAD"
#define Retentativas 5

static EventGroupHandle_t s_wifi_event_group; //Grupo de eventos FreeRTOS para sinalizar quando a rede está conectada e pronta para fazer requisições
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define BUF_SIZE (1024)

static const char *TAG = "file_server";
static const char *TAG2 = "wifi station";

static int s_retry_num = 0;
int UART=0,Control=0; //A variavel UART armazena a quantidade de  comandos na UART a serem mandados ao site (max 8) e a variavel Control é usada como variavel auxiliar
//para um FOR
char Frase[8][30];  //Variavel onde são armazenados os utimos 8 comandos na UART
int state = 0;
const static char http_html_hdr[] = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\n\r\n";
const static char http_index_hml[] = "<html><head><title>ESP-32</title><script src=https://code.jquery.com/jquery-3.4.1.js></script></head><body><h1>Controle do LED</h1><a href=\"l\">Ligar LED</a><br><a href=\"d\">Desligar LED</a></body></html><h2><head><title>ESP-32</title></head><body><h1>Caixa de texto</h1><form action=/CaixaDeTexto target=_self method=get><textarea input type=text id=Texto name=Texto></textarea><br><input type=submit value=Enviar Mensagem><br></form></head><body><h1>Controle de atividades</h1><a href=\"s\">Hibernar</a><br></head><body><h1>Historico de Comandos</h1>";

static int Size (char Frase[]) //Função que retorna o tamanho da string 
{int A=0;
while(Frase[A++]!='\0'); //Nessa função procura-se o \0 que é um indicativo do final da string
return A-1; //Retorna-se a posição onde está o \0 que é o mesmo número do tamanho da string (não incluindo o \0)
}


//Webserver HTTP lWip-----------------------
static void
http_server_netconn_serve(struct netconn *conn) //Inicialização do servidor
{
  struct netbuf *inbuf;
  char *buf;
  u16_t buflen;
  err_t err;

   //Lê os dados da porta , bloqueando-os caso ainda não houver dados
   //Assume-se que todos os dados necessários estão em apenas um netbuf
  err = netconn_recv(conn, &inbuf); //Atribui-se a variavel err o retorno da função netconn_recv que recebe dados do servidor

  if (err == ERR_OK) {  
    netbuf_data(inbuf, (void**)&buf, &buflen);  //Realiza a leitura do webserver utilizando os buffers
    
  

    if (buflen>=5 &&  //Checa-se se o webserver está recebendo um HTTP GET
        buf[0]=='G' &&
        buf[1]=='E' &&
        buf[2]=='T' &&
        buf[3]==' ' &&
        buf[4]=='/' ) {
          
          
          
       
       
       gpio_pad_select_gpio(HttpLED); 
       gpio_set_direction(HttpLED, GPIO_MODE_OUTPUT);//Define como saída o LED a ser controlado pelo web server
       if(buf[5]=='l'){
         gpio_set_level(HttpLED,1);//Liga o led no caso de ter recebido /l , o que acontece ao usuário digitar na URL ou clicar no botão Ligar LED
       }
       if(buf[5]=='d'){
         gpio_set_level(HttpLED,0);//Desliga o led no caso de ter recebido /d , o que acontece quando o usuário digitar isso na URL ou clicar no botão Desligar LED
       }
       if(buf[5]=='s'){ //Comando retornado quando o usuário clica em Hibernar ou escreve /s na URL
       	esp_err_t esp_sleep_enable_ext0_wakeup(gpio_num_t gpio_num, int level); //Declara a função de definir um GPIO para acordar o ESP
       	void esp_deep_sleep_start(void);//Declara a função de entrar no modo Deep Sleep
  		esp_sleep_enable_ext0_wakeup(GPIO_NUM_2,1); //Chama a função de definir um GPIO para acordar o LED, o GPIO definido é o 2
       	esp_deep_sleep_start(); //Chama a função de entrar no modo Deep Sleep, ela não requer nenhum parâmetro
       
	   }
	   
	   if(buf[5]=='C') //Normalmente recebe-se "CaixaDeTexto?Texto=" , como nenhum outro comando envia um C maiusculo no GET, decidiu-se apenas verificar o primeiro C
	   {char Mensagem[50];
	   int BuffAtual=24,MensagemAtual=0;
	   while(1) //O while (1) foi escolhido para que a tarefa continue sendo executada a não ser que o programa encontre o fim da string (espaço em branco)
	   {
	   if(buf[BuffAtual]==' ') {Mensagem[MensagemAtual]='\0';break;} //Espaço em branco indica fim, visto que a URL codifica espaço como + , logo ao identificar
	   //um espaço em branco, é escrito um \0 indicando fim da string e encerra-se o while, permitindo a escrita da mensagem
	   		else if(buf[BuffAtual]!='%' && buf[BuffAtual]!='+') Mensagem[MensagemAtual++]=buf[BuffAtual++]; //Nesse caso é lido o valor da variavel e verificado se ela não
	   //é um espaço ou um caractere codificado (que geralmente são descritos como % e mais dois digitos) , assim ela pode ser armazenada direto sem nenhum processo
	   //de decodificaçção
			else if(buf[BuffAtual]=='+')  //Nesse IF, busca-se fazer a decodificação mais simples, que é a de + ser o equivalente a um espaço
			{
			Mensagem[MensagemAtual++]=' ';
			BuffAtual++;
			}
	   	
	   		else if(buf[BuffAtual]=='%') //Ao identificar o caractere "%" ele o reconhece como um código de três digitos, onde os dois próximos digitos são lidos
	   		//e de acordo com o que tiver neles, é decodificado e escrito a mensagem correta (visto que uma URL não pode ter certos caracteres que então são codificados)
	   		{
	   		if(buf[BuffAtual+1]=='2' && buf[BuffAtual+2]=='5') Mensagem[MensagemAtual++]='%'; //Em todos IFs , se avança a variavel MensagemAtual, para que 
			else if(buf[BuffAtual+1]=='3' && buf[BuffAtual+2]=='F') Mensagem[MensagemAtual++]='?';//o próximo caractere seja escrito num espaço correto
			else if(buf[BuffAtual+1]=='2' && buf[BuffAtual+2]=='1') Mensagem[MensagemAtual++]='!';
			else if(buf[BuffAtual+1]=='E' && buf[BuffAtual+2]=='1') Mensagem[MensagemAtual++]='á';
			else if(buf[BuffAtual+1]=='E' && buf[BuffAtual+2]=='A') Mensagem[MensagemAtual++]='ê';
			else if(buf[BuffAtual+1]=='4' && buf[BuffAtual+2]=='0') Mensagem[MensagemAtual++]='@';
			else if(buf[BuffAtual+1]=='2' && buf[BuffAtual+2]=='3') Mensagem[MensagemAtual++]='#';
			else if(buf[BuffAtual+1]=='2' && buf[BuffAtual+2]=='C') Mensagem[MensagemAtual++]=',';
			else if(buf[BuffAtual+1]=='2' && buf[BuffAtual+2]=='8') Mensagem[MensagemAtual++]='(';
			else if(buf[BuffAtual+1]=='2' && buf[BuffAtual+2]=='9') Mensagem[MensagemAtual++]=')';
			else if(buf[BuffAtual+1]=='3' && buf[BuffAtual+2]=='D') Mensagem[MensagemAtual++]='=';
			else if(buf[BuffAtual+1]=='2' && buf[BuffAtual+2]=='6') Mensagem[MensagemAtual++]='&';
			else if(buf[BuffAtual+1]=='E' && buf[BuffAtual+2]=='7') Mensagem[MensagemAtual++]='ç';
			else if(buf[BuffAtual+1]=='2' && buf[BuffAtual+2]=='4') Mensagem[MensagemAtual++]='$';
			else if(buf[BuffAtual+1]=='E' && buf[BuffAtual+2]=='0') Mensagem[MensagemAtual++]='à';
			else if(buf[BuffAtual+1]=='E' && buf[BuffAtual+2]=='D') Mensagem[MensagemAtual++]='í';
			else if(buf[BuffAtual+1]=='F' && buf[BuffAtual+2]=='A') Mensagem[MensagemAtual++]='ú';
			else if(buf[BuffAtual+1]=='C' && buf[BuffAtual+2]=='1') Mensagem[MensagemAtual++]='Á';
			else if(buf[BuffAtual+1]=='C' && buf[BuffAtual+2]=='0') Mensagem[MensagemAtual++]='À';
			else if(buf[BuffAtual+1]=='E' && buf[BuffAtual+2]=='3') Mensagem[MensagemAtual++]='ã';
			else if(buf[BuffAtual+1]=='C' && buf[BuffAtual+2]=='3') Mensagem[MensagemAtual++]='Ã';
			else if(buf[BuffAtual+1]=='C' && buf[BuffAtual+2]=='A') Mensagem[MensagemAtual++]='Ê';
			BuffAtual+=3;   	
			}
	   		
	   
	   }
	 	printf("%s\n",Mensagem); //Escreve-se a mensagem
	   }
	          
      netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);//Escreve no site o que está dentro da variavel http_html_hdr

      /* Send our HTML page */
      netconn_write(conn, http_index_hml, sizeof(http_index_hml)-1, NETCONN_NOCOPY); //Escreve no site o texto HTMl armazenado na http_index_html
      if(UART>0) //Função que escreve os ultimos 8 comandos recebidos na UART do ESP em ordem da mais recente para a mais antiga
			{	
				for(Control=0;Control<UART;Control++) 
				{
				netconn_write(conn,Frase[Control],Size(Frase[Control]), NETCONN_NOCOPY); //É escrita a frase armazenada (mais recentes primeiro)
				netconn_write(conn,"<br>",sizeof("<br>")-1, NETCONN_NOCOPY);//Escreve-se no site um <br> que tem como função realizar um espaço, para que cada comando
				//esteja numa nova linha
				}	
			}	
    }

  }
  
  netconn_close(conn); //Fecha a conexão

  
  netbuf_delete(inbuf); //deleta o conteúdo do buffer
}

static void http_server(void *pvParameters)
{
  struct netconn *conn, *newconn;
  err_t err;
  conn = netconn_new(NETCONN_TCP);//Define um novo server TCP 
  netconn_bind(conn, NULL, 80); //Define o uso da porta 80
  netconn_listen(conn);  //coloca o servidor TCP no modo listen
  do {
     err = netconn_accept(conn, &newconn); //Função para aceitar conexões de um client
     if (err == ERR_OK) {
       http_server_netconn_serve(newconn); //Inicializa o servidor
       netconn_delete(newconn); //Apaga os dados na variavel newconn
     }
   } while(err == ERR_OK);
   netconn_close(conn);//Fecha a conexão
   netconn_delete(conn);//Deleta os dados da conexão
}


//---------------------------------------------------------------------

//Wifi
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) { //Quando a função Event Handler recebe a variavel WIFI_EVENT ela tenta conectar o WIFI
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) { //Caso tenha desconectado, ela tenta até o número de retentativas
        if (s_retry_num < Retentativas) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG2, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);//Caso não consiga, seta o WIFI_FAIL_BIT 
        }
        ESP_LOGI(TAG2,"connect to the AP fail");//E informa que não foi possivel efetuar a conexão
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) { //Caso o event Handler tenha recebido o IP
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;//Cria uma variavel para armazenar o evento
        ESP_LOGI(TAG2, "got ip:" IPSTR, IP2STR(&event->ip_info.ip)); //Informa o IP recebido
        s_retry_num = 0;//Zera a variavel de retentativas
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); //Seta a variavel Wifi Connected
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate(); //Atribui ao Wifi_Event_group o retorno da função xEventGroupCreate que cria um grupo de eventos RTOS

    ESP_ERROR_CHECK(esp_netif_init());//Inicializa o stack do protocolo TCP/IP

    ESP_ERROR_CHECK(esp_event_loop_create_default()); //Cria um evento de loop com as configurações padrão
    esp_netif_create_default_wifi_sta(); //Executa a função, que pelo nome, deve incializar as configurações de uma rede STA

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();- //Define as configurações padrão de wifi
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));//Inicializa o Wifi com as configurações previamente estabelecidas

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL)); //Chama a função Event Handler com as informações do Wifi
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL)); //Chama a função Event Handler com as informações de IP

    wifi_config_t wifi_config = { //Aplica-se a estrutura wifi_config, o nome da rede wifi e a senha
        .sta = {
            .ssid = RedeWifi,
            .password = Senha
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );//Configura o modo do Wifi como modo STA
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );//Configura-se o Wifi com as configurações de nome da Rede e Senha
    ESP_ERROR_CHECK(esp_wifi_start() ); //Inicia-se o wifi

    ESP_LOGI(TAG2, "wifi_init_sta finished."); //As tentativas de conexão iniciam

   
     
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);  //a função xEventGroupWaitBits retorna as variaveis WIFI_CONNECTED_BIT ou WIFI_FAIL_BIT um valor dependnedo da conexão, se foi efetuada
			//com sucesso, WIFI_CONNECTED_BIT é setado para 1, caso contrário WIFI_FAIL_BIT é setado para 1 
     
     //Em seguida é testado para checar se a conexão aconteceu ou não
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG2, "connected to ap SSID:%s password:%s",
                 RedeWifi, Senha); //Mensagem de conexão efetuada com sucesso, mostrando o nome da rede e a senha
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG2, "Failed to connect to SSID:%s, password:%s",
                 RedeWifi, Senha); //Mensagem de conexão falha mostrando o nome da rede e a senha
    } else {
        ESP_LOGE(TAG2, "UNEXPECTED EVENT"); //Mensagem de erro inesperado
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

//------------------------------------------



void app_main(void)
{int Pull=0,SW=0,A=0,B=0,Tam_Contagem=50,EstadoAnterior,LEDPisca=0,Contagem=0;//As variaveis Pull e SW controlam o Switch do encoder, 
//as variaveis A e B definem a tensão média no BreathingLED e o sentido da variação (crescente/decrescente) respectivamente, as variaveis
//Tam_Contagem,Contagem e EstadoAnterior, controlam o tamanho da contagem que o ESP faz até alterar o estado do led (consequentemente 
//mudando a frequencia), a cnotagem em si e a mudança de estado do Encoder , o que permite ler a sua direção respectivamente, LedPisca
// é uma variavel que modifica-se sempre que a contagem chega ou ultrapasssa o tamanho da contagem (com proposito de piscar o led)
//A variavel Frase indica os comandos digitados na UART a serem armazenados e exibidos depois no historico de comandos, a variavel UART
// indica quantas frases o site precisa exibir(indo no máximo até 8), a variavel Control é utilizada para uso correto da função For

gpio_config_t io_conf; //define a variavel de configuração de gpio como io_config 
	 io_conf.intr_type = GPIO_PIN_INTR_DISABLE; //desativa interrupção
	io_conf.mode= GPIO_MODE_OUTPUT; //define GPIO como saída
	 io_conf.pin_bit_mask = (1<<(BreathingLED) | (1<<(SwitchLED)) | (1<<(LEDPiscante)));//Máscara de bits para definir quais portas GPIO serão modificadas
	 gpio_config(&io_conf); //aplica a configuração da estrutura ao GPIO
	 
	io_conf.mode= GPIO_MODE_INPUT; //define GPIO como entrada
	
	io_conf.pull_down_en = 0; //Desativa-se o Pull Down
	 io_conf.pin_bit_mask = (1<<(EncoderData) | 1<<(EncoderCLK)); //Dessa vez ativando apenas o pino de leitura das duas saidas do encoder
	  gpio_config(&io_conf); //Aplica as configurações
	  io_conf.pull_up_en = 1;//Ativa-se o Pull Up
	  io_conf.pin_bit_mask = 1<<(EncoderSW);//Define-se apenas para configurar o pino do Switch do Encoder
	  gpio_config(&io_conf);//Aplica as configurações


  
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, BreathingLED); //Configuração do PWM, inicia-se o PWM 0 no pino do BreathingLED
 mcpwm_config_t pwm_config;  //Declara-se a variavel de estrutura de configuraão
    pwm_config.frequency = 1;    //frequencia = 1Hz
    pwm_config.cmpr_a = 0.1;       //duty cycle do PWMxA = 0.1% 
    pwm_config.counter_mode = MCPWM_UP_COUNTER; //Contagem crescente
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0; //Modo de trabalho 0
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Inicialização do PWM
    

    printf("Hello world!\n");  //Escreve diretamente na serial a frase "Hello World"

 uart_config_t uart_config = { //Declara=se a variavel de estrutura que configura a UART
        .baud_rate = 115200, //BAUD RATE de 115200
        .data_bits = UART_DATA_8_BITS, //8 bits de dados
        .parity    = UART_PARITY_DISABLE, //Sem paridade
        .stop_bits = UART_STOP_BITS_1, //1 Stop Bit
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, //Desabilita-se o controle do fluxo de dados
        .source_clk = UART_SCLK_APB,//Definição do clock de fonte para a UART
    };
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);//Instala o driver da UART 0 - OBS: a UART 0 é a padrão que se comunica com a porta USB
    uart_param_config(UART_NUM_0, &uart_config);//Aplica-se a UART 0 a estrutura de configurações definida ates
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE); //Define=se os pinos para a porta, no caso estão todos em
    //default
     uint8_t *data = (uint8_t *) malloc(BUF_SIZE); //declara-se uma variavel UINT_8_t para que ela possa receber os dados da UART
	data[0]=0; //define-se ela como 0, para que na leitura seja possivel diferenciar o valor inicial da variavel para um caractere imprimivel (código ASCII acima de 31)
printf("Esperando pela entrada de qualquer caractere pela UART USB\n");	//Digita-se na UART a mensagem


while (data[0]<31) { //Foi definido que qualquer valor de entrada acima de valor ASCII 31, seria considerada caractere (valores imprimiveis da tabela ASCII)
//Assim o programa ficará preso nesse While até que o usuario digite um caractere imprimivel na porta UART USB
        int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS); //Nesse caso é realizada a leitura da porta UART USB (UART 0)
    }
printf("O programa detectou entrada, continuando....\n"); //Assim que o programa sair do while,ele deverá mostrar essa mensagem
vTaskDelay(1000 / portTICK_PERIOD_MS);//Essa função realiza um pequeno delay, a ideia é atrasar o programa para que o usuário veja a mensagem acima, caso contrário 
//o programa preencheria o terminal da UART rápido demais para o usuário perceber a mensagem anterior
	data[0]=0; //Zera-se a variavel Data, para que seja possível diferenciar novamente seu valor de um caractere imprimivel
    
    esp_err_t ret = nvs_flash_init();//Inicializa o armazenamento não volátil do ESP (NVS)
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }//Caso uma das flags de ERR_NVS_NO_FREE_PAGES e ESP_ERR_NVS_NEW_VERSION_FOUND estejam acionadas, o programa apaga o conteúdo do NVS e o reinicia
    ESP_ERROR_CHECK(ret);//Checa se não há erros na inicialização

    ESP_LOGI(TAG2, "ESP_WIFI_MODE_STA"); //Avisa ao usuário que está ligando o WIFI no modo STA 
    wifi_init_sta(); //Inicializa o Wifi
    xTaskCreate(&http_server, "http_server", 2048, NULL, 5, NULL); //Inicia o webserver e cria uma tarefa que abre as funções do webserver periodicamente
    
    EstadoAnterior=gpio_get_level(EncoderData) + gpio_get_level(EncoderCLK)*2; //Leitura dos pinos do Encoder, a ideia é separa-los em 4 estados diferentes 
    //0,1,2,3 por isso um deles é multiplicado por 2  , a ideia de colocar uma função dessas no main é para que haja um valor inicial antes de entrar no
    //switch
    

    
    
    
    while(1) {
    	
    		switch(gpio_get_level(EncoderData) + gpio_get_level(EncoderCLK)*2) //Na função Switch, é obsevada a diferença e de acordo com o valor anterior e o atual
    		//é possivel saber para que lado o encoder está girando, com isso , modifica-se o tamanho da contagem para alterar o estado do LED , assim quando o
    		//tamanho da contagem aumenta a frequência diminui e quando ela abaixa, a frequência aumenta (pois é necessário menos contagens para mudança de estado)
	{
		case 0:
		if(EstadoAnterior==1) Tam_Contagem--;
		if(EstadoAnterior==2) Tam_Contagem++;
		break;
		
		case 1:
		if(EstadoAnterior==3) Tam_Contagem--;
		if(EstadoAnterior==0) Tam_Contagem++;
		break;
		
		case 2:
		if(EstadoAnterior==0) Tam_Contagem--;
		if(EstadoAnterior==3) Tam_Contagem++; 
		break; 
		
		case 3:
		if(EstadoAnterior==2) Tam_Contagem--;
		if(EstadoAnterior==1) Tam_Contagem++;
		break;
			
	}
	EstadoAnterior=gpio_get_level(EncoderData) + gpio_get_level(EncoderCLK)*2; //Novamente é feita a leitura, só que dessa vez dentro de um while, a ideia é que
	//O encoder conte quando há uma diferença entre o sinal lido no switch e o sinal lido na variavel EstadoAnterior, para isso é bom que a leitura da variavel 
	//EstadoAnterior esteja o mais afastado possível da leitura realizada pela função Switch pois para haver uma diferença é preciso que a leitura seja feita
	//depois da atribuição do EstadoAnterior mas antes da leitura no Switch, o que no caso é a maior parte do tempo de execução do código, considerando que o while
	//tem várias outras funções junto e um delay no final então é improvável que ele erre o tempo para a medição, mas pode acontecer
    //considerou-se que o clock do ESP é suficiente para realizar essa tarefa sem uso de interrupções, porém caso o usuário gire o encoder rápido demais durante o 
    //intervalo de leitura (entre atribuição de EstadoAnterior e switch) e o clock não seja rápido o suficiente para percebe-las como duas leituras diferentes
    //o programa pode acabar lendo errado e achar que o Encoder está girando em outro sentido (quando na verdade não está)
	//por conta disso aconselha-se que o Encoder não seja girado muito rápido 
	
	if(Tam_Contagem<0) Tam_Contagem=0; //Define-se um limite inferior para o tamanho da contagem (para que não chegue abaixo de 0, o que não faz sentido)
    	if(Contagem>Tam_Contagem) //Assim que a contagem superar o tamanho da contagem
    	{
    		Contagem=0; //A contagem é zerada
    		gpio_set_level(LEDPiscante,LEDPisca^=1);//E o estado do LED alterado
    		
	    }
    	Contagem++; //Aumenta-se a contagem 
    	
         int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS); //Realiza-se a leitura da porta UART
    	if(data[0]>31) //Caso a leitura tenha sido de um caractere imprimivel
		{
			if(UART>0) //UART é uma variavel que define a quantidade de frases escritas na porta UART que foram armazenadas, nessa função o programa altera a ordem
			//Das frases, para que a mais recente seja a Frase[0]
			{	
				for(Control=UART;Control>0;Control--)
				{
				sprintf(Frase[Control],"%s",Frase[Control-1]); //A função SPRINTF armazena o conteúdo do printf na variavel Frase[Control]
				}	
			}	
		if(len-1>29)
		{
		sprintf(Frase[0],"Comando ultrapassou limite!!!"); //Caso o usuário digite uma mensagem muito grande, esta é substituida
	    
		
		}	
		else
		{	
	
		data[len-1]='\0';//Escreve-se manualmente um \0 após o final da frase recebida na UART
		sprintf(Frase[0],"%s",data); //Escreve-se o dado recebido na UART na variavel Frase
	    }
		printf("%s",Frase[0]); //Escreve-se a frase na porta UART para certificar-se de que o programa leu certo
    	data[0]=0;//Novamente se zera a primeira posição da variavel para poder identificar o próximo comando do usuário
    	if(UART<8) //Define-se um limite para quantas frases são aramazenadas, que no caso é 8
		{
		UART++;//Aumenta-se a variavel UART, pois após uma frase ser armazenada, significa que uma frase deve ser escrita
    	}
		}
	   
	   if(B==0) //A variavel B define o sentido atual do BreathingLED, de forma que quando B é zero, o valor de tensão média aumenta
       {
       	A++;//Variavel A que define o duty cycle do PWM, que consequentemente modifica a tensão média, alterando o brilho do Led
	   }
	  if(B==1) //Caso B for 1, a tensão média no LED vai abaixando
	  {
	  A--;	
	  }
	  if(A==100) B=1;//Quando o LED está no seu brilho máximo, ele modifica a variavel B para que agora ele diminua o brilho até zerar
	  if(A==0) B=0; //Quando o brilho está no minimo (LED desligado) essa função modifica a variavel B para que agora ele aumente seu brilho até o máximo
	   pwm_config.cmpr_a = A; //Define o Duty Cycle do PWM como o valor da variavel A
	   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Aplica as mesmas configurações de antes, só que agora com o Duty Cycle modificado
		if(gpio_get_level(EncoderSW)==1) //Leitura do Pino do Switch do Encoder (nesse caso, quando o botão não está pressionado, pois é um Pull-Up)
		{
			Pull=0; //Define-se que o botão deixou de ser pressionado com a variavel Pull
		}
			if(gpio_get_level(EncoderSW)==0 && Pull==0) //Aqui verifica-se se o botão foi pressionado e se é a primeira vez que isso acontece desde que o botão foi
			//solto, de forma que ele não fique alterando o estado do LED caso o usuário segure o botão
			{
				Pull=1; //Define que o usuário está segurando o botão
				gpio_set_level(SwitchLED,SW^=1); //Altera o estado do LED
			}
		
    	vTaskDelay(10 / portTICK_PERIOD_MS); //Realiza-se um pequeno delay
    }
}
