#include <math.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "pico/bootrom.h"
#include "pio_matrix.pio.h"
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "funcoes/mudar_LED.c"
#include <string.h>

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

#define UART_ID uart0 // Seleciona a UART0
#define BAUD_RATE 115200 // Define a taxa de transmissão
#define UART_TX_PIN 0 // Pino GPIO usado para TX
#define UART_RX_PIN 1 // Pino GPIO usado para RX

#define Led_RED 11
#define Led_GREEN 12
#define Led_BLUE 13

#define Botao_A 5
#define Botao_B 6

double numeros[10][25] = {
                        {0.8, 0.8, 0.8, 0.8, 0.8, // 0
                        0.8, 0.0, 0.0, 0.0, 0.8, 
                        0.8, 0.0, 0.0, 0.0, 0.8,
                        0.8, 0.0, 0.0, 0.0, 0.8,
                        0.8, 0.8, 0.8, 0.8, 0.8},
                        
                        {0.0, 0.0, 0.8, 0.0, 0.0, // 1
                        0.0, 0.0, 0.8, 0.8, 0.0, 
                        8.0, 0.0, 0.8, 0.0, 0.0,
                        0.0, 0.0, 0.8, 0.0, 0.0,
                        0.0, 0.0, 0.8, 0.0, 0.0},

                        {0.8, 0.8, 0.8, 0.8, 0.8, // 2
                        0.8, 0.0, 0.0, 0.0, 0.0, 
                        0.8, 0.8, 0.8, 0.8, 0.8,
                        0.0, 0.0, 0.0, 0.0, 0.8,
                        0.8, 0.8, 0.8, 0.8, 0.8},

                        {0.8, 0.8, 0.8, 0.8, 0.8, // 3
                        0.8, 0.0, 0.0, 0.0, 0.0, 
                        0.8, 0.8, 0.8, 0.8, 0.8,
                        0.8, 0.0, 0.0, 0.0, 0.0,
                        0.8, 0.8, 0.8, 0.8, 0.8},

                        {0.8, 0.0, 0.0, 0.0, 0.8, // 4
                        0.8, 0.0, 0.0, 0.0, 0.8, 
                        0.8, 0.8, 0.8, 0.8, 0.8,
                        0.8, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.8},

                        {0.8, 0.8, 0.8, 0.8, 0.8, // 5
                        0.0, 0.0, 0.0, 0.0, 0.8, 
                        0.8, 0.8, 0.8, 0.8, 0.8,
                        0.8, 0.0, 0.0, 0.0, 0.0,
                        0.8, 0.8, 0.8, 0.8, 0.8},

                        {0.8, 0.8, 0.8, 0.8, 0.8, // 6
                        0.0, 0.0, 0.0, 0.0, 0.8, 
                        0.8, 0.8, 0.8, 0.8, 0.8,
                        0.8, 0.0, 0.0, 0.0, 0.8,
                        0.8, 0.8, 0.8, 0.8, 0.8},

                        {0.8, 0.8, 0.8, 0.8, 0.8, // 7
                        0.0, 0.8, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.8, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.8, 0.0,
                        0.8, 0.0, 0.0, 0.0, 0.0},

                        {0.8, 0.8, 0.8, 0.8, 0.8, // 8
                        0.8, 0.0, 0.0, 0.0, 0.8, 
                        0.8, 0.8, 0.8, 0.8, 0.8,
                        0.8, 0.0, 0.0, 0.0, 0.8,
                        0.8, 0.8, 0.8, 0.8, 0.8},

                        {0.8, 0.8, 0.8, 0.8, 0.8, // 9
                        0.8, 0.0, 0.0, 0.0, 0.8, 
                        0.8, 0.8, 0.8, 0.8, 0.8,
                        0.8, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.8}

                        };

static volatile uint32_t last_time = 0; // Armazena o tempo do último evento (em microssegundos)
volatile int contador = 0; // Contador que vai mudar o valor nas interrupções, por isso voltatile

// String e char global para print na uart
char messageStr[50] = "Bem vindo";
char message = '\0';

// Função que vai pra interrupção
static void gpio_irq_handler(uint gpio, uint32_t events);

int main()
{
  // Inicialização de variáveis para a matriz de leds
  PIO pio = pio0; 
  uint16_t i;
  bool ok;
  uint32_t valor_led;
  double r,b,g;

  //coloca a frequência de clock para 128 MHz, facilitando a divisão pelo clock
  ok = set_sys_clock_khz(128000, false);

  // Inicializa todos os códigos stdio padrão que estão ligados ao binário.
  stdio_init_all();
  //Configurações da PIO
  uint offset = pio_add_program(pio, &pio_matrix_program);
  uint sm = pio_claim_unused_sm(pio, true);
  pio_matrix_program_init(pio, sm, offset, OUT_PIN);

  // INICIAR LEDS
  gpio_init(Led_RED);
  gpio_set_dir(Led_RED, GPIO_OUT);
  gpio_put(Led_RED, 0);

  gpio_init(Led_GREEN);
  gpio_set_dir(Led_GREEN, GPIO_OUT);
  gpio_put(Led_GREEN, 0);

  gpio_init(Led_BLUE);
  gpio_set_dir(Led_BLUE, GPIO_OUT);
  gpio_put(Led_BLUE, 0);

  // INICIAR BOTOES
  gpio_init(Botao_A);
  gpio_set_dir(Botao_A, GPIO_IN);
  gpio_pull_up(Botao_A);

  gpio_init(Botao_B);
  gpio_set_dir(Botao_B, GPIO_IN);
  gpio_pull_up(Botao_B);

  // INTERRUPÇÔES
  gpio_set_irq_enabled_with_callback(Botao_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(Botao_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

  // I2C Initialisation. Using it at 400Khz.
  i2c_init(I2C_PORT, 400 * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_pull_up(I2C_SDA); // Pull up the data line
  gpio_pull_up(I2C_SCL); // Pull up the clock line
  ssd1306_t ssd; // Inicializa a estrutura do display
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_send_data(&ssd); // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

  bool cor = true;

  // Inicializa a UART
  uart_init(UART_ID, BAUD_RATE);

  // Configura os pinos GPIO para a UART
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART); // Configura o pino 0 para TX
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART); // Configura o pino 1 para RX

  // Mensagem inicial
  const char *init_message = "UART Demo - RP2\r\n"
                              "Digite algo e veja o eco:\r\n";
  uart_puts(UART_ID, init_message);
  while (true)
  {
    desenho_pio(numeros[contador], valor_led, pio, sm, 0, 0, 1); // Desenha na matriz de led
    // Verifica se há dados disponíveis para leitura
      if (uart_is_readable(UART_ID)) {
          // Lê um caractere da UART
          char c = uart_getc(UART_ID);
          message = c;
  
          uart_putc(UART_ID, c);

          // Envia uma mensagem adicional para cada caractere recebido
          uart_puts(UART_ID, " <- Eco do RP2\r\n");
          
          if (c >= '0' && c <= '9') {
            switch (c)
            {
            case '0':
              contador = 0;
              break;
            case '1':
              contador = 1;
              break;
            case '2':
              contador = 2;
              break;
            case '3':
              contador = 3;
              break;
            case '4':
              contador = 4;
              break;
            case '5':
              contador = 5;
              break;
            case '6':
              contador = 6;
              break;
            case '7':
              contador = 7;
              break;
            case '8':
              contador = 8;
              break;
            case '9':
              contador = 9;
              break;
              
            }
          }
      } 

    if (message == '\0') {
      cor = !cor;
      // Atualiza o conteúdo do display com animações
      ssd1306_fill(&ssd, !cor); // Limpa o display
      ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); // Desenha um retângulo
      ssd1306_draw_string(&ssd, messageStr, 20, 30); // Desenha uma string
      ssd1306_send_data(&ssd); // Atualiza o display
    } else {
      cor = !cor;
      // Atualiza o conteúdo do display com animações
      ssd1306_fill(&ssd, !cor); // Limpa o display
      ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); // Desenha um retângulo   
      ssd1306_draw_char(&ssd, message, 60, 30); 
      ssd1306_send_data(&ssd); // Atualiza o display
    }

    

    sleep_ms(40);
  }
}

static void gpio_irq_handler(uint gpio, uint32_t events) {
  // Tempo atual em (ms)
  uint32_t current_time = to_us_since_boot(get_absolute_time()); 

  // Verificar o tempo que passou
  if ((current_time - last_time) > 200000) { // 200ms de deboucing
    uart_puts(UART_ID, "Interrupcao\r\n");
    last_time = current_time;
    if (gpio == 5) { // Se botão A
    uart_puts(UART_ID, "Click no A\r\n");
    if (!gpio_get(Led_GREEN)) {
      uart_puts(UART_ID, "Ligar led verde\r\n\n");
      strcpy(messageStr, "Led verde on");
      // Para indicar que eu quero print na string
      message = '\0';
      gpio_put(Led_GREEN, 1);
    } else {
      strcpy(messageStr, "Led verde off");
      message = '\0';
      gpio_put(Led_GREEN, 0);
      uart_puts(UART_ID, "Desligar led verde\r\n\n");
    }

    } else { // Se B
    uart_puts(UART_ID, "Click no B\r\n");
    if (!gpio_get(Led_BLUE)) {
      strcpy(messageStr, "Led azul on");
      message = '\0';
      uart_puts(UART_ID, "Ligar led azul\r\n\n");
      gpio_put(Led_BLUE, 1);
    } else {
      strcpy(messageStr, "Led azul off");
      message = '\0';
      uart_puts(UART_ID, "Desligar led azul\r\n\n");
      gpio_put(Led_BLUE, 0);
    }

    }
    
    printf("Numero: %d\n", contador);
  }
}