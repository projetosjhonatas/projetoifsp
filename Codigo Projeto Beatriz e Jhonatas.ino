/*
* Exibe o nível de som ponderado A medido pelo microfone I2S
*
* (c) 2019 Ivan Kostoski
*
* Este programa é um software livre: você pode redistribuí-lo e/ou modificá-lo
* sob os termos da GNU General Public License conforme publicada por
* a Free Software Foundation, seja a versão 3 da Licença, ou
* (a seu critério) qualquer versão posterior.
*    
* Este programa é distribuído na esperança de que seja útil,
* mas SEM QUALQUER GARANTIA; sem mesmo a garantia implícita de
* COMERCIALIZAÇÃO ou ADEQUAÇÃO A UM DETERMINADO FIM. Veja o
* GNU General Public License para mais detalhes.
*
* Você deve ter recebido uma cópia da GNU General Public License
* junto com este programa. Caso contrário, consulte <https://www.gnu.org/licenses/>.
 */

/*
* Esboce amostras de dados de áudio do microfone I2S, processa os dados
* com filtros IIR digitais e calcula o equivalente ponderado A ou C
* Nível de som contínuo (Leq)
*
* I2S é configurado para amostrar dados em Fs=48000KHz (valor fixo devido a
* projeto de filtros digitais IIR). Os dados são lidos da fila I2S
* em 'blocos de amostra' (bloco padrão de 125ms, igual a 6.000 amostras)
* por 'i2s_reader_task', filtrado através de dois filtros IIR (equalizador
* e ponderação), resumido e enviado para 'samples_queue' como
* soma dos quadrados das amostras filtradas. A tarefa principal então puxa os dados
* da fila e calcula o valor de decibéis em relação ao microfone
* amplitude de referência, derivada da sensibilidade da folha de dados dBFS
* valor, número de bits em dados I2S e o valor de referência para
* qual a sensibilidade é especificada (normalmente 94dB, senoidal puro
* onda a 1KHz).
*
* Exibe a linha na pequena tela OLED com LAeq 'curto' (125ms)
* resposta e valor numérico LAeq(1sec) dB do sinal RMS.
 */

#include <driver/i2s.h>
#include "sos-iir-filter.h"






////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>         // Comunicação I2C
#include <WiFi.h>         // Biblioteca para utilizar o Wi-Fi
#include <PubSubClient.h>           // Biblioteca do MQTT
///////////////////////////////////////////////////////////////////////////////////





//
// Configuração
//
#define TOPICO_PUBLISH_LIMITE   "topico_sensor_limite"
#define TOPICO_PUBLISH_RUIDO   "topico_sensor_ruido"
#define LEQ_PERIOD        1           // segundo(s)
#define WEIGHTING         C_weighting // Também disponível: 'C_weighting' ou 'None' (Z_weighting)
#define LEQ_UNITS         "LAeq"      // personaliza com base no peso acima usado
#define DB_UNITS          "dBA"       // personaliza com base no peso acima usado
#define USE_DISPLAY       0

// NOTA: Alguns microfones requerem pelo menos um filtro DC-Blocker
#define MIC_EQUALIZER     INMP441    // Veja abaixo os filtros IIR definidos ou defina como 'Nenhum' para desabilitar
#define MIC_OFFSET_DB     3.0103      // Offset padrão (RMS de onda senoidal vs. dBFS). Modifique este valor para calibração linear

// Personalize esses valores da folha de dados do microfone
# define  MIC_SENSITIVITY    - 26          // valor dBFS esperado em MIC_REF_DB (valor de sensibilidade da folha de dados)
# define  MIC_REF_DB         94.0         // Valor no qual a sensibilidade do ponto é especificada na folha de dados (dB)
# define  MIC_OVERLOAD_DB 116.0    //        dB - Ponto de sobrecarga acústica
# define  MIC_NOISE_DB       29           // dB - Noise floor
# define  MIC_BITS           24           // número válido de bits em dados I2S
# define  MIC_CONVERT(s) (s >> (SAMPLE_BITS - MIC_BITS))
# define  MIC_TIMING_SHIFT   0            // Defina como um para corrigir o tempo MSB para alguns microfones, ou seja, SPH0645LM4H-x

// Calcula o valor da amplitude de referência em tempo de compilação
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

//
// Pinos I2S - Podem ser roteados para quase qualquer pino ESP32 (não utilizado).
//             SD pode ser qualquer pino, incluindo pinos somente de entrada (36-39).
//             SCK (ou seja, BCLK) e WS (ou seja, L/R CLK) devem ser pinos com capacidade de saída
//
// Abaixo estão apenas exemplos para o layout da minha placa, coloque aqui os pinos que você vai usar
//
#define I2S_WS            15 
#define I2S_SCK           2 
#define I2S_SD            13 
#define led               4

// Periférico I2S a ser usado (0 ou 1)
#define I2S_PORT          I2S_NUM_0

//
// Configure sua biblioteca de exibição (e geometria) aqui
// 
#if (USE_DISPLAY > 0)
  // ThingPulse/esp8266-oled-ssd1306, você pode precisar da fonte mais recente e PR#198 para 64x48
  #include <SSD1306Wire.h>
  #define OLED_GEOMETRY     GEOMETRY_64_48
  // #define OLED_GEOMETRY GEOMETRY_128_32
  // #define OLED_GEOMETRY GEOMETRY_128_64
  #define OLED_FLIP_V       1
  SSD1306Wire display(0x3c, SDA, SCL, OLED_GEOMETRY);
#endif


//
// Filtros IIR
//

// Filtro DC-Blocker - remove o componente DC dos dados I2S
// Veja: https://www.dsprelated.com/freebooks/filters/DC_Blocker.html
// a1 = -0,9992 deve atenuar fortemente as frequências abaixo de 10Hz
SOS_IIR_Filter DC_BLOCKER = { 
  gain: 1.0,
  sos: {{-1.0, 0.0, +0.9992, 0}}
};

// 
// Filtros IIR do equalizador para achatar a resposta de frequência do microfone
// Veja o respectivo arquivo .m para o design do filtro. F = 48kHz.
//
// Os filtros são representados como seções de segunda ordem em cascata com suposição
// que b0 e a0 são iguais a 1,0 e 'ganho' é aplicado na última etapa
// Os coeficientes B e A foram transformados com GNU Octave:
// [sos, ganho] = tf2sos(B, A)
// Veja: https://www.dsprelated.com/freebooks/filters/Series_Second_Order_Sections.html
// NOTA: Os coeficientes 'a1' e 'a2' da matriz SOS são negativos da saída tf2sos
//

// TDK/InvenSense ICS-43434
// Folha de dados: https://www.invensense.com/wp-content/uploads/2016/02/DS-000069-ICS-43434-v1.1.pdf
// B = [0,477326418836803, -0,486486982406126, -0,336455844522277, 0,234624646917202, 0,111023257388606];
// A = [1,0, -1,93073383849136326, 0,86519456089576796, 0,06442838283825100, 0,00111249298800616];
SOS_IIR_Filter ICS43434 = { 
  gain: 0.477326418836803,
  sos: { // Seções de segunda ordem {b1, b2, -a1, -a2}
   {+0.96986791463971267, 0.23515976355743193, -0.06681948004769928, -0.00111521990688128},
   {-1.98905931743624453, 0.98908924206960169, +1.99755331853906037, -0.99755481510122113}
  }
};

// TDK/InvenSense ICS-43432
// Folha de dados: https://www.invensense.com/wp-content/uploads/2015/02/ICS-43432-data-sheet-v1.3.pdf
// B = [-0,45733702338341309 1,12228667105574775 -0,77818278904413563, 0,00968926337978037, 0,10345668405223755]
// A = [1,0, -3,3420781082912949, 4,4033694320978771, -3,0167072679918010, 1,2265536567647031, -0,2962229189311990, 0,0251085747458112]
SOS_IIR_Filter ICS43432 = {
  gain: -0.457337023383413,
  sos: { // Seções de segunda ordem {b1, b2, -a1, -a2}
    {-0.544047931916859, -0.248361759321800, +0.403298891662298, -0.207346186351843},
    {-1.909911869441421, +0.910830292683527, +1.790285722826743, -0.804085812369134},
    {+0.000000000000000, +0.000000000000000, +1.148493493802252, -0.150599527756651}
  }
};

// TDK/InvenSense INMP441
// Folha de dados: https://www.invensense.com/wp-content/uploads/2015/02/INMP441.pdf
// B ~= [1,00198, -1,99085, 0,98892]
// A ~= [1,0, -1,99518, 0,99518]
SOS_IIR_Filter INMP441 = {
  gain: 1.00197834654696, 
  sos: { // Seções de segunda ordem {b1, b2, -a1, -a2}
    {-1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091}
  }
};

// Infineon IM69D130 Shield2Go
// Folha de dados: https://www.infineon.com/dgdl/Infineon-IM69D130-DS-v01_00-EN.pdf?fileId=5546d462602a9dc801607a0e46511a2e
// B ~= [1,001240684967527, -1,996936108836337, 0,995703101823006]
// A ~= [1,0, -1,997675693595542, 0,997677044195563]
// Com componente de bloqueio DC adicional
SOS_IIR_Filter IM69D130 = {
  gain: 1.00124068496753,
  sos: {
    {-1.0, 0.0, +0.9992, 0}, // bloqueador DC, a1 = -0.9992
    {-1.994461610298131, 0.994469278738208, +1.997675693595542, -0.997677044195563}
  }
};

// Knowles SPH0645LM4H-B, rev. B
// https://cdn-shop.adafruit.com/product-files/3421/i2S+Datasheet.PDF
// B ~= [1,001234, -1,991352, 0,990149]
// A ~= [1,0, -1,993853, 0,993863]
// Com componente de bloqueio DC adicional
SOS_IIR_Filter SPH0645LM4H_B_RB = {
  gain: 1.00123377961525, 
  sos: { // Seções de segunda ordem {b1, b2, -a1, -a2}
    {-1.0, 0.0, +0.9992, 0}, // bloqueador DC, a1 = -0.9992
    {-1.988897663539382, +0.988928479008099, +1.993853376183491, -0.993862821429572}
  }
};

//
// Filtros de ponderação
//

//
// Filtro IIR de ponderação A, Fs = 48KHz
// (Pelo Dr. Matt L., Fonte: https://dsp.stackexchange.com/a/36122)
// B = [0,169994948147430, 0,280415310498794, -1,120574766348363, 0,131562559965936, 0,974153561246036, -0,282740857326553, -0,15281076]
// A = [1,0, -2,12979364760736134, 0,42996125885751674, 1,62132698199721426, -0,96669962900852902, 0,00121015844426781, 0,044003006967881, 0,044003006967898]
SOS_IIR_Filter A_weighting = {
  gain: 0.169994948147430, 
  sos: { // Seções de segunda ordem {b1, b2, -a1, -a2}
    {-2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926},
    {+4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332},
    {-0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989}
  }
};

//
// Filtro IIR de ponderação C, Fs = 48KHz
// Projetado por ajuste de curva invfreqz, consulte o respectivo arquivo .m
// B = [-0.49164716933714026, 0.14844753846498662, 0.74117815661529129, -0.03281878334039314, -0.29709276192593875, -0.06442545322197900, -0.00364152725482682]
// A = [1,0, -1,0325358998928318, -0,9524000181023488, 0,8936404694728326 0,2256286147169398 -0,1499917107550188, 0,0156718181681081]
SOS_IIR_Filter C_weighting = {
  gain: -0.491647169337140,
  sos: { 
    {+1.4604385758204708, +0.5275070373815286, +1.9946144559930252, -0.9946217070140883},
    {+0.2376222404939509, +0.0140411206016894, -1.3396585608422749, -0.4421457807694559},
    {-2.0000000000000000, +1.0000000000000000, +0.3775800047420818, -0.0356365756680430}
  }
};


//
// Amostragem
//
#define SAMPLE_RATE       48000 // Hz, corrigido para design de filtros IIR
#define SAMPLE_BITS       32    // bits
#define SAMPLE_T          int32_t 
#define SAMPLES_SHORT     (SAMPLE_RATE / 8) // ~125ms
#define SAMPLES_LEQ       (SAMPLE_RATE * LEQ_PERIOD)
#define DMA_BANK_SIZE     (SAMPLES_SHORT / 16)
#define DMA_BANKS         32

// Dados que enviamos para 'samples_queue'
struct sum_queue_t {
  // Soma dos quadrados das amostras de microfone, após o filtro Equalizador
  float sum_sqr_SPL;
  // Soma dos quadrados de amostras de microfone ponderadas
  float sum_sqr_weighted;
  // Somente depuração, tiques do FreeRTOS que passamos processando os dados I2S
  uint32_t proc_ticks;
};
QueueHandle_t samples_queue;

// Buffer estático para bloco de amostras
float samples[SAMPLES_SHORT] __attribute__((aligned(4)));

//
// Configuração de amostragem de microfone I2S
//
void mic_i2s_init() {
   // Configura I2S para amostrar canal mono para SAMPLE_RATE * SAMPLE_BITS
  // NOTA: Atualização recente para Arduino_esp32 (1.0.2 -> 1.0.3)
  //        parece ter trocado os canais ONLY_LEFT e ONLY_RIGHT
  const i2s_config_t i2s_config = {
    mode: i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    sample_rate: SAMPLE_RATE,
    bits_per_sample: i2s_bits_per_sample_t(SAMPLE_BITS),
    channel_format: I2S_CHANNEL_FMT_ONLY_LEFT,
    communication_format: i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,
    dma_buf_count: DMA_BANKS,
    dma_buf_len: DMA_BANK_SIZE,
    use_apll: true,
    tx_desc_auto_clear: false,
    fixed_mclk: 0
  };
   // Mapeamento de pinos I2S
  const i2s_pin_config_t pin_config = {
    bck_io_num:   I2S_SCK,  
    ws_io_num:    I2S_WS,    
    data_out_num: -1, // não usado
    data_in_num:  I2S_SD   
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  #if (MIC_TIMING_SHIFT > 0) 
    // Manipulação não documentada (?!) de registradores periféricos I2S
    // para corrigir problemas de tempo de MSB com alguns microfones I2S
    REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));   
    REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);  
  #endif
  
  i2s_set_pin(I2S_PORT, &pin_config);

  // FIXME: Há um problema conhecido com esp-idf e taxas de amostragem, consulte:
  //       https://github.com/espressif/esp-idf/issues/2634
  //        Enquanto isso, a linha abaixo parece definir a taxa de amostragem em ~47999,992Hz
  //        fifs_req = 24576000, sdm0 = 149, sdm1 = 212, sdm2 = 5, odir = 2 -> fifs_reached = 24575996  
  // NOTA: Isso parece ser corrigido no ESP32 Arduino 1.0.4, esp-idf 3.2
  //        Deve ser seguro remover...
  // #include <soc/rtc.h>
  // rtc_clk_apll_enable(1, 149, 212, 5, 2);
}

//
// Tarefa do Leitor I2S
//
// A justificativa para a leitura de tarefas separadas I2S é que o filtro IIR
// processamento cam ser agendado para núcleo diferente no ESP32
// enquanto a tarefa principal pode fazer outra coisa, como atualizar o
// exibe no exemplo
//
// Como isso deve ser executado como uma tarefa de alta prioridade separada,
// só fazemos o trabalho mínimo necessário com os dados I2S
// até que seja 'comprimido' em soma de quadrados
//
// Prioridade do FreeRTOS e tamanho da pilha (em palavras de 32 bits)
#define I2S_TASK_PRI   4
#define I2S_TASK_STACK 2048
//
void mic_i2s_reader_task(void* parameter) {
  mic_i2s_init();

  // Descarta o primeiro bloco, o microfone pode ter tempo de inicialização (ou seja, INMP441 até 83ms)
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  while (true) {
    // Bloqueia e espera pelos valores do microfone do I2S
    //
    // Os dados são movidos de buffers DMA para nosso buffer de 'amostras' pelo driver ISR
    // e quando há quantidade de dados solicitada, a tarefa é desbloqueada
    //
    // Nota: i2s_read não se importa se está escrevendo no buffer float[], ele irá escrever
    //        valores inteiros para o endereço fornecido, conforme recebido do periférico de hardware.
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);

    TickType_t start_tick = xTaskGetTickCount();
    
    // Converte (incluindo deslocamento) valores inteiros de microfone para floats,
    // usando o mesmo buffer (supondo que o tamanho da amostra seja igual ao tamanho do float),
    // para economizar um pouco de memória
    SAMPLE_T* int_samples = (SAMPLE_T*)&samples;
    for(int i=0; i<SAMPLES_SHORT; i++) samples[i] = MIC_CONVERT(int_samples[i]);

    sum_queue_t q;
     // Aplicar equalização e calcular a soma dos quadrados ponderada em Z,
    // grava amostras filtradas de volta no mesmo buffer.
    q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);

    // Aplicar ponderação e calcular a soma dos quadrados ponderada
    q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);

    // Somente depuração. Carrapatos que passamos filtrando e somando blocos de dados I2S
    q.proc_ticks = xTaskGetTickCount() - start_tick;

    // Envia as somas para a fila do FreeRTOS onde a tarefa principal irá buscá-las
    // e mais valores de decibéis calculados (divisão, logaritmos, etc...)
    xQueueSend(samples_queue, &q, portMAX_DELAY);
  }
}

//
// Configuração e loop principal
//
// Nota: Use doubles, não floats, aqui, a menos que você queira fixar
//        a tarefa para qualquer núcleo em que ela esteja sendo executada no momento
// 







/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Dados para conectar no Wi-Fi
const char* ssid = "TP-LINK_BF1DCE";  //Nome do Wi-Fi
const char* password = "zelia@252"; //Senha do Wi-Fi

//Dados do MQTT Broker
const char* mqtt_server = "projetosjhonatas.duckdns.org"; //Localização do MQTT Broker
const char* mqtt_username = "admin"; //Usuario do Broker
const char* mqtt_password = "wendell28"; //Usuario do Broker
const int mqtt_port = 1883; //Porta de acesso do Broker

WiFiClient espClient;
PubSubClient client(espClient);

void init_wifi() { //Função para inicializar o Wi-Fi
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  reconnect_wifi();
}

void init_mqtt(){ //Função para inicializar o MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void reconnect_wifi(){ //Função para conectar o Wi-Fi
  if (WiFi.status() == WL_CONNECTED)
        return;
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {//Loop que mostra a tentativa de se conectar no Wi-Fi
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
}

void reconnect_mqtt() { //Função para conectar o mqtt
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(1000);
    }
  }
}

void verificaConexao(){
   if (!client.connected()) {
    reconnect_mqtt();
  }
  reconnect_wifi();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////







void setup() {

   // Se necessário, agora você pode diminuir a frequência da CPU,
  // ou seja, se você quiser (levemente) reduzir o consumo de energia do ESP32
  setCpuFrequencyMhz(80); // Deve rodar tão baixo quanto 80MHz
  
  Serial.begin(9600);
  delay(1000); // Segurança
///////////////////////////////////////////////////////////////////////////////
  init_wifi(); //Inicializa o Wi_Fi
  init_mqtt(); //Inicializa o MQTT
/////////////////////////////////////////////////////////////////////////////////
  pinMode(led,OUTPUT);
  
  #if (USE_DISPLAY > 0)
    display.init();
    #if (OLED_FLIP_V > 0)
      display.flipScreenVertically();
    #endif
    display.setFont(ArialMT_Plain_16);
  #endif

  // Cria fila do FreeRTOS
  samples_queue = xQueueCreate(8, sizeof(sum_queue_t));
  
  // Cria a tarefa FreeRTOS do leitor I2S
  // NOTA: A versão atual do ESP-IDF irá fixar a tarefa
  //        automaticamente para o primeiro núcleo em que ele roda
  //        (devido ao uso das instruções de FPU de hardware).
  //        Para controle manual veja: xTaskCreatePinnedToCore
  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);

  sum_queue_t q;
  uint32_t Leq_samples = 0;
  double Leq_sum_sqr = 0;
  double Leq_dB = 0;
  double lim = 70;

  // Lê a soma das amostras, calculada por 'i2s_reader_task'
  while (xQueueReceive(samples_queue, &q, portMAX_DELAY)) {

    // Calcula os valores de dB relativos a MIC_REF_AMPL e ajusta para referência de microfone
    double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

    // Em caso de sobrecarga acústica ou medição abaixo do piso de ruído, informe o valor infinito de Leq
    if (short_SPL_dB > MIC_OVERLOAD_DB) {
      Leq_sum_sqr = INFINITY;
    } else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
      Leq_sum_sqr = -INFINITY;
    }

    // Acumula a soma Leq
    Leq_sum_sqr += q.sum_sqr_weighted;
    Leq_samples += SAMPLES_SHORT;

    // Quando coletamos amostras suficientes, calcula o novo valor Leq
    if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
      Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
      Leq_sum_sqr = 0;
      Leq_samples = 0;
      
      // Saída serial, personalize (ou remova) conforme necessário
      Serial.printf("%.1f\t", lim);
      Serial.printf("%.1f\n",  Leq_dB);

      if (Leq_dB > lim)
      {
       digitalWrite(led,HIGH); 
      }
      else
      {
        digitalWrite(led,LOW);
      }

       // Somente depuração
      //Serial.printf("%u processing ticks\n", q.proc_ticks);
    }

  verificaConexao();
  client.loop();

char ruido_str[10] = {0};
  dtostrf(Leq_dB, 2, 2, ruido_str); //temperatura - leitura para o dashboard
client.publish("TOPICO_PUBLISH_RUIDO", ruido_str); //publish para do mqtt para o dashboard

char lim_str[10] = {0};
dtostrf(lim, 2, 2, lim_str); //temperatura - leitura para o dashboard
client.publish("TOPICO_PUBLISH_LIMITE", lim_str); //publish para do mqtt para o dashboard

    #if (USE_DISPLAY > 0)

      //
      // Código de exemplo que exibe o valor medido.
      // Você deve personalizar o código abaixo para sua exibição
      // e biblioteca de exibição usada.
      //
      
      display.clear();

      // É importante notificar de alguma forma quando o dispositivo estiver fora de seu alcance
      // pois os valores calculados são muito prováveis ​​com grande erro
      if (Leq_dB > MIC_OVERLOAD_DB) {
        // Exibe 'Overload' se o valor dB estiver acima do AOP
        display.drawString(0, 24, "Overload");
      } else if (isnan(Leq_dB) || (Leq_dB < MIC_NOISE_DB)) {
        // Exibe 'Low' se o valor dB estiver abaixo do nível de ruído
        display.drawString(0, 24, "Low");
      }
      
      // A linha Leq 'short'
      double short_Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(sqrt(double(q.sum_sqr_weighted) / SAMPLES_SHORT) / MIC_REF_AMPL);
      uint16_t len = min(max(0, int(((short_Leq_dB - MIC_NOISE_DB) / MIC_OVERLOAD_DB) * (display.getWidth()-1))), display.getWidth()-1);
      display.drawHorizontalLine(0, 0, len);
      display.drawHorizontalLine(0, 1, len);
      display.drawHorizontalLine(0, 2, len);
      
      // Os decibéis numéricos Leq
      display.drawString(0, 4, String(Leq_dB, 1) + " " + DB_UNITS);
      
      display.display();
      
    #endif // USE_DISPLAY
  }
}

void loop() {
  // Nothing here..
}
