/*
========================================================================================================================
PROGRAMAÇÃO MOENDA MAQTRON

AUTOR: Robison Walter Wille 
DATA: 24/06/2022
========================================================================================================================
*/

/*
OBSERVAÇÕES PARA CPRRETO FUNCIONAMENTO 
1 - Descomentar a função de leitura de correte  "calculaCorrente()"
2 - Descomentar a função de gravação da matriz deslocada internamente a função "montaDB()"
3 - Descomentar a parte em que há a verificação de que a máquina foi desligada, e caso nenhum comando seja dado dentro de 4 segundos ela grava na flasha os dados de horas e minutos (Dentro do Loop())
4 - Decomentar a chamada da função calcula tensão da rede (Dentro do Loop())
5 - Descomentar a chamada da função analizaTensãoRede() (Dentro do Loop()), retirar o abre e fecha chaves do if necessário para a chamada da função; 
6 - Dentro da função atualizaHorimetro() descomentar a parte de gravação das horas na flash do microcontrolador
*/

#include <Arduino.h>
//#include <SPI.h>
#include "EmonLib.h"
#include <U8g2lib.h>
#include <string.h>
#include "nvs_flash.h"  
#include "LittleFS.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

/*======================================================================================================================
                                       MAPEAMENTO DE HARDWARE 
========================================================================================================================*/
#define CorrenteMotor            36   //Entrada analógica de leitura da corrente de trabalho do motor 
#define tensaoRede               39   //Entrada analógica de leiturad da tensão da rede 110/220VCA
#define fcPr1                    34   //Entrada digital Fim de curso 1 da proteção dos rolos 
#define fcPr2                    35   //Entrada digital Fim de curso 2 da proteção dos rolos 
#define ds                       32   //Saída digital ulizada no chip Select 
#define stcp                     33   //Saída digital utilizada no chip Select 
#define shcp                     25   //Saída digital utilizada no chip Select 
#define sinTrava                 27   //Entrada digital de status da trava magnética ("1" tampa fechada | "0" Tampa aberta)  (tensão da rede))
#define btLiga                   14   //Entrada digital do botão Liga
#define btDesliga                13   //Entrada digital do botão Desliga 
#define btReverte                02   //Entrada digtal do botão de Revesão
#define liberaTrava              04   //Saída digital para liberação da fechadura elétrica 
#define fcPt                     05   //Entrada digital do fim de curso de proteção das transmissões 
#define ena                      18   //Enable do display 
#define rw                       19   //Read/Write do display 
#define rs                       21   //RS do display 
#define rst                      22   //Reset do display
#define btEmegencia              23   //Entrada digital do botão de emergência 

#define TEMP_LUBRIFICA           800 //Tempo para librificação da máquina em horas 

/*======================================================================================================================
                            DEFINIÇÃO DAS CHAVES DE GRAVAÇÃO DE HORA E MINUTO 
========================================================================================================================*/
#define CHAVE_HORA   "horas"
#define CHAVE_MIN    "minu"
#define CHAVE_SEG    "seg"
#define CHAVE_PONT   "point"

#define Hr00 "hr00"    
#define Hr10 "hr10"     
#define Hr20 "hr20"     
#define Hr30 "hr30"     
#define Hr40 "hr40"     
#define Hr50 "hr50"     
#define Hr60 "hr60"     
#define Hr70 "hr70"     

#define Mi01 "mi01"
#define Mi11 "mi11"
#define Mi21 "mi21"
#define Mi31 "mi31"
#define Mi41 "mi41"
#define Mi51 "mi51"
#define Mi61 "mi61"
#define Mi71 "mi71"

#define V02 "v02"
#define V12 "v12"
#define V22 "v22"
#define V32 "v32"
#define V42 "v42"
#define V52 "v52" 
#define V62 "v62" 
#define V72 "v72"

#define I03 "i03"
#define I13 "i13"
#define I23 "i23"
#define I33 "i33"
#define I43 "i43"
#define I53 "i53"
#define I63 "i63"
#define I73 "i73"

#define MATRIZ "matriz"

#define LN "linha"  // Salva qual a ultima linah gravada na matriz

// DEFINIÇÃO DE PINAGEM DO DISPLAY 
U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, ena, rw, rs, rst);

/*======================================================================================================================
                                   DEFINIÇÕES DE VARIÁVEIS GLOBAIS
========================================================================================================================*/
 EnergyMonitor emon1;    //cria objeto para sensor de corrente
 double Irms;
 unsigned long timer1 = millis();
 byte saidaShSel = 0; // Saída do ShiftSelect [ ledLiga, ledDesl, ledRevert, triac,byPass, antiHor, Hor, LibBot]
 uint vetTensaoRede[10];
 int i = 0;                           // Variável de contagem das amostras de tensão 
 uint tensaoDaRede = 0;
 
 int    ledLiga     = 7;
 int    ledDesl     = 6;
 int    ledRevert   = 5;
 int    triac       = 4;
 int    byPass      = 3;
 int    antHor      = 2;
 int    hora        = 1;
 int    buzzer      = 0;



unsigned long    piscaalerta           = millis();
uint8_t           sensorR1              = 0;
int               timeBeep              = 300;         //Tempo em millisegundos de liga e desliga beep em caso de corrente alta 
uint8_t           sensorR2              = 0;
uint8_t           sensortransmissao     = 0;
uint8_t           botaoemerg            = 0;
uint8_t           pisca                 = 0;
uint8_t           sobrecorrente         = 0;
uint8_t           sobretensao           = 0;
uint8_t           subtensao             = 0;
uint8_t           tempTurnOff           = 15;           //Tempo em segundos, para desligar o Motor caso a corrente não baixe 
unsigned long     tempoinicializacao    = millis(); 
uint8_t           tempologo             = 0;
uint8_t           tempomuda             = 0;
uint8_t           botaoliga             = 0;
uint8_t           botaoreverte          = 0;
uint16_t          cont                  = 0;
unsigned long     horimetro             = 0;
uint8_t           contSubTensao         = 0;
uint8_t           contSubTensaoExtrema  = 0;
unsigned long     tempSubtensao         = 0;
uint8_t           contSobreCorre        = 0;
uint8_t           contExtSobreCorre     = 0;
uint8_t           auxCont               = 0;
unsigned long     tempSobCorrent        = 0;
unsigned long     delayGravaEprom       = 0;
unsigned long     timeCallDB            = 0;
unsigned long     timeHideDB            = 0;
unsigned long     timeBeepCorrente      = 0;
uint16_t          horas                 = 0;
uint16_t           seg                  = 0; 
uint16_t          minutos               = 0;
uint16_t          minAntigo             = 0;
uint16_t          segAntigo             = 0;
uint16_t          numPont               = 0;
uint16_t          minGravacaoFs         = 0;
uint16_t          numPontGravLfs        = 0;
byte              linha                 = 0;
byte              col                   = 0; 
byte              confLinha             = 0; // Variável de conferência das linhas da matriz para gravação correta na memória 
byte              auxLin                = 0;
byte              recLin                = 0; // Variável que auxilia na gravação das  linhas da matriz do banco de dados após as 7 lihas de dados ja gravados 
byte              recCol                = 0; // Variável que auxilia na gravação das colunas da matriz do banco de dados após as 7 lihas de dados ja gravados 
byte              teste                 = 0;
uint8_t           contLub               = 0;
unsigned long     timeMsgLubrif         = 0;
unsigned long     finalRamp             = 0;
byte              fRamp                 = 1;
int               controleDisp          = 0;


size_t ofSetInicial                     = 80;   // Bytes a serem mantidos na liberação de memória do arquivo.
size_t deletLengByte                    = 104;  // Bytes a serem exluidos na liberação de memória 
                                                // 24 Bytes tem a mensagem completa 500 pontos = 12kbytes
uint16_t          db[8][4]{
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0}
};

 boolean StatusLiga           =   false;
 boolean StatusRevert         =   false;
 boolean statuDesl            =   true;
 boolean StatusEmerg          =   false;
 boolean statusSR1            =   false;
 boolean statusSR2            =   false;
 boolean statusST             =   false;
 boolean statusSeguranca      =   false;
 boolean nivelTensao          =   false; // false --> tensão 220V  |  true --> tensão de 127V
 boolean V_baixaDetectada     =   false;
 boolean V_extBaixaDetectada  =   false;
 boolean I_altaDetectada      =   false; 
 boolean I_muitoAltaDetectada =   false; 
 boolean condGravaEprom       =   false;
 boolean habCallDB            =   false;
 boolean hb                   =   false;
 boolean habLunbrificacao     =   false;
 boolean StGravaFlah          =   false; //Se linha do banco > 7 "StGravaFlah" vai para true, indicando gravação do BD
 boolean sTBeeb               =   false;
 boolean habBeep              =   false;

/*==================================================================================
                                ESCOPO DAS FUNÇÕES 
====================================================================================*/
void calculaCorrente();                       //Função responsável pela leitura da corrente do motor 
void IRAM_ATTR InterrupLIGA();                //Função responsável por tratar as interrupções externas 
void IRAM_ATTR InterrupDESL();                //Função responsável por tratar as interrupções externas 
void IRAM_ATTR InterrupREVERT();              //Função responsável por tratar as interrupções externas 
void IRAM_ATTR InterrupEMERG();               //Função responsável por tratar as interrupções externas 
void statusSensores();                        //Funçaõ responsável por verificar o estatus dos sensores da moenda 
void atualizaSaidaSS(int posicao, int i);     //Função de atualização das saídas do ShiftSelect
void calculaTensaoRede();                     //Função que realiza o cáculo da tensão da rede 
void atualizaLedBotoes();                     //Função de acionamento do led dos botões da moenda 
void inicializadisplay();                     //Função que inicializa as configurações principais do displai ( sentido de escrita e orientação ) 
void telafixa(boolean SR1, boolean ST, boolean EM);  //Função que chama a telafixa do display(as divisões da tela)
void inicia_moenda();                         //Funcção de inicialização da moenda. Inicializa com a marca MAQTRON...
void aguardandooperacao();                    //Função da tela aguardando operação se os sinais de segurança estiverem OK
void tempoparainicializacao();                //Função de contagem do tempo de inicialização com a logo da maqtron 
void draw(int tela);                          //Função que separa as telas do sisplay a serem chamadas 
void tempomensagensdealerta();                //função que define o tempo de apresentação das mensagens de alerta do display 
void prot_rolos(boolean SR1);    //Função responsável por criar as telas quando ha a falha de um  ou ambos os sensores de proteção dos rolos 
void prot_transmissao(boolean ST);            //Função responsável por criar as telas quando ha a falha de sinal do sensor de proteção da transmissão 
void emergencia(boolean EM);                  //Função reponsável por apresentar a tela caso o botão de emergência seja pressioando 
void emOperacaoLigada();                      //Função de que apresenta a tela indicando que a moenda esta operando "LIGADA"
void emOperacaoReversa();                     //Função de que apresenta a tela indicando que a moenda esta operando "REVERSA"
void atualizaHorimetro();                     //Função responsável por fazer as atualizações da hora trabalhada;
void analisaTensaoRede();                     //Função responsável por realiza a análise a da tensão 
void alerta_Vbaixa();                         //Função de chamada da tela que indica a tensão baixa 
void alerta_Vmuitobaixa();                    //Função de chamada da tela que indica tensão muito baixa 
void analizeCorrenteMotor();                  //Função de verifica se a corrente do motor esta "Alta" ou "Muito Alta" baseandose na tensão onde onde o motro foi instalado
void alerta_Ialta();                         //Função de chama a tela do display, alertando da "corrente alta" 
void alertaImuitoalta();                      //Função de chama a tela do display, alertando da "corrente muito alta" 
void contagemHoras(uint16_t horaFuc);         //Função de configura as horas na tela dos display incrementando o espaço da esquerda do display
void grava_dado_nvs(uint16_t dado,  char leitura); //Função responsável pela gravação na memória Flash do ESP
uint16_t le_dado_nvs(char leitura);                            //Função responsável por realizar a leitura do dado da memória flash do ESP 
void montaDB(uint16_t Hr, uint16_t Mn, double Im, uint Vr);    //Realiza a montagem do banco de dados quando detecta tensão baixa ou corrente alta   
void banco_de_dados();                                         // Função responsável por chamara a tela e indicar os valores de tensão e corrente alterados  
void analisaChamaScDB();                                       // Fução que monitora a chamada do banco de dados 
void  lubrifique();                                            // Função de monitora chamada para avisar da necessidade de lubrificação da máquina.
void addDataToFile(int hora, int min, int segundos, int corrente, int tensao);
void analisPreArmaz();
void monBackupdeDados();
void printDataToSerial();
void analisFsArqv();
void libMemorisFS();
void reiniciaDisp(int nT);


void setup(){

  /*------------- INICIALIZAÇÃO DO SISTEMA DE ARQUIVOS FS ---------------*/
    if(LittleFS.begin()){ Serial.println(F("LittleFS Inicializado.."));}
    else{Serial.println(F("Fail to Open LittleFs Open.."));}

  /* ------------------ VERIFICA A EXISTÊNCIA DO AQUIVO ---------------- */
  if(!LittleFS.exists("/DadosBracana.txt")){
    File dataFile = LittleFS.open("/DadosBracana.txt", "w");

    if (!dataFile){Serial.println("Erro ao abrir o Arquivo");}
    else{
      Serial.println("Arquivo aberto com Sucesso");
      dataFile.println("-------------- DADOS BRACANA (HORA:MIN:SEG | TENSÃO | CORRENTE) ---------------");
    }
    dataFile.close();
  }
  else{Serial.println("Arquivo LittleFS ja existe...");}

  uint16_t dadoLidoHora;
  uint16_t dadoLidoMin;

  u8g2.begin();
  /*==================================================================================
                            DEFINIÇÃO DE ENTRADAS DIGITAIS   
  ====================================================================================*/
  pinMode(fcPr1,        INPUT);
  pinMode(fcPr2,        INPUT);
 //pinMode(sinTrava,     INPUT);
  pinMode(btLiga,       INPUT);
  pinMode(btDesliga,    INPUT);
  pinMode(btReverte,    INPUT);
  pinMode(btEmegencia,  INPUT);

  /*==================================================================================
                            DEFINIÇÃO DE SAÍDAS DIGITAIS   
  ====================================================================================*/
  pinMode(ds,           OUTPUT);
  pinMode(stcp,         OUTPUT);
  pinMode(shcp,         OUTPUT);
  pinMode(liberaTrava,  OUTPUT);

  /*==================================================================================
                      CONFIGURAÇÃO DAS INTERRUÇÕES EXTERNAS    
  ====================================================================================*/
  attachInterrupt(btLiga,      InterrupLIGA,   RISING);
  attachInterrupt(btDesliga,   InterrupDESL,   RISING);
  attachInterrupt(btReverte,   InterrupREVERT, RISING);
  attachInterrupt(btEmegencia, InterrupEMERG,  RISING);
  
  //Configuração do Sensor de corrente 
  emon1.current(CorrenteMotor,13);

 /*INICIALIZAÇÃO DAS PORTAS SERIAIS*/
    Serial.begin(9600); 

  //------------------------INICIALIZA AS VARIÁVEIS A MATRIZ DE DADOS COM AS INFORMAÇÕES DA FLASH --------------------------------
  linha = le_dado_nvs('L');
  confLinha = linha;
  Serial.print("Leu dados da linha - ");
  Serial.println(linha);

  //------------------------MONTA A METRIZ DO BANCO DE DADOS, CAPTURANDO OS DADOS DA MEMÓRIA FLASH --------------------------------
    for(int q = 0 ; q < linha ; q++){
      auxLin = q;
     for(int d = 0 ; d < 4; d++){
        if(d == 0)       { db[q][d] = le_dado_nvs('A'); Serial.println(db[q][d]);}
        else if (d == 1) { db[q][d] = le_dado_nvs('B'); Serial.println(db[q][d]);}
        else if (d == 2) { db[q][d] = le_dado_nvs('C'); Serial.println(db[q][d]);}
        else if (d == 3) { db[q][d] = le_dado_nvs('D'); Serial.println(db[q][d]);}
      }
      Serial.println();
    }
  
  //-----------------------------------  LÊ HORAS  E MINUTOS DA MEMÓRIA FLASH ------------------------------------------------------

  horas     = le_dado_nvs('H'); 
  minutos   = le_dado_nvs('M');
  seg       = le_dado_nvs('S');
  numPont   = le_dado_nvs('P');
  minAntigo = minutos;
  segAntigo = seg;
 // horas   = dadoLidoHora;
  Serial.print("Numero de pontos no Sistema de arquivos LittleFS: ");
  Serial.println(numPont);
  Serial.println();

  Serial.print(horas);
  Serial.print("H : ");
  Serial.print(minutos);
  Serial.print("M : ");
  Serial.print(seg);
  Serial.println("S ");

  // ----------------------------- VERIFICA CASO A MÁQUINA NECESSIDE DE LUBRIFICAÇÃO -----------------------------------------------

  if      (horas >= TEMP_LUBRIFICA && horas < TEMP_LUBRIFICA +2 && contLub == 0) {habLunbrificacao = true; timeMsgLubrif = millis();}
  else if (horas >= TEMP_LUBRIFICA * 2 && horas < (TEMP_LUBRIFICA * 2) + 2)      {habLunbrificacao = true; timeMsgLubrif = millis();}
  else if (horas >= TEMP_LUBRIFICA * 3 && horas < (TEMP_LUBRIFICA * 3) + 2)      {habLunbrificacao = true; timeMsgLubrif = millis();}
  else if (horas >= TEMP_LUBRIFICA * 4 && horas < (TEMP_LUBRIFICA * 4) + 2)      {habLunbrificacao = true; timeMsgLubrif = millis();}
  else if (horas >= TEMP_LUBRIFICA * 5 && horas < (TEMP_LUBRIFICA * 5) + 2)      {habLunbrificacao = true; timeMsgLubrif = millis();}
  else if (horas >= TEMP_LUBRIFICA * 6 && horas < (TEMP_LUBRIFICA * 6) + 2)      {habLunbrificacao = true; timeMsgLubrif = millis();}
  else if (horas >= TEMP_LUBRIFICA * 7 && horas < (TEMP_LUBRIFICA * 7) + 2)      {habLunbrificacao = true; timeMsgLubrif = millis();}
  else if (horas >= TEMP_LUBRIFICA * 8 && horas < (TEMP_LUBRIFICA * 8) + 2)      {habLunbrificacao = true; timeMsgLubrif = millis();}
  else if (horas >= TEMP_LUBRIFICA * 9 && horas < (TEMP_LUBRIFICA * 9) + 2)      {habLunbrificacao = true; contLub = 1; timeMsgLubrif = millis();}
  else if (horas >= 2 && contLub == 1 )                                          {habLunbrificacao = true; contLub = 0; timeMsgLubrif = millis();}

  if(digitalRead(btEmegencia)) StatusEmerg = true;
} 

void loop() {
  tempoparainicializacao();
  tempomensagensdealerta();
  if(tempoinicializacao < 8000)  draw(0);

  if (tempoinicializacao > 8000){
   statusSensores();
  }

  if (statuDesl || StatusEmerg){atualizaSaidaSS(buzzer,   0); }

  atualizaLedBotoes();
  analisaChamaScDB();
  if(Serial.available()) monBackupdeDados();

// ------------------------------ VERIFICA A CONDIÇÃO PARA APRESENTAR MENSAGEM DE LUBRIFICAÇÃO NO DISPLAY -------------------------

  if((millis() - timeMsgLubrif < 30000) && habLunbrificacao && tempoinicializacao > 8000 && !StatusEmerg && !habCallDB && !V_baixaDetectada && !V_extBaixaDetectada && !I_altaDetectada && !I_muitoAltaDetectada)   draw(12);
  else if(tempoinicializacao > 8000 &&  (millis() - timeMsgLubrif >= 30000))                  habLunbrificacao = false;

  if(StatusLiga || StatusRevert)atualizaHorimetro();
  

// APÓS DESLIGADA A MÁQUINA AGUARDA 2 SEGUNDOS E GRAVA A VARIÁVEL MINUTOS NA FLASH
 if((millis() - delayGravaEprom >= 2000) && condGravaEprom){
      if(minutos != minAntigo) { 
        grava_dado_nvs(minutos, 'M'); //GRAVA HORAS NA FLASH
        minAntigo = minutos;
        condGravaEprom = false;
        Serial.println();
        Serial.print("Sucesso na gravação dos Minutos");
      }
      if( seg != segAntigo) { 
        grava_dado_nvs(seg, 'S'); //GRAVA HORAS NA FLASH
        segAntigo = seg;
        condGravaEprom = false;
        Serial.println();
        Serial.print("Sucesso na gravação dos Segundos");
      }
  } //Grava conteúdo dos minutos e Segundos gravados  

//----------- VERIFICA SE O BOTÃO LIGA OU REVERTE FORAM PRESSIONADOS PARA FICAR CHAMANDO AS TELAS CORRETAMENTE----------------
  if(StatusLiga && !I_altaDetectada && !I_muitoAltaDetectada && !V_extBaixaDetectada && !V_baixaDetectada && statusSeguranca) draw(5);
  else if (StatusRevert && !I_altaDetectada && !I_muitoAltaDetectada && !V_extBaixaDetectada && !V_baixaDetectada && statusSeguranca) draw(6);

//----------- A CADA 2S MONITORA TENSÃO DA REDE, E CORRRNTE SE O MOTOR ESTIVER LIGADO---------------
  if(millis() - timer1 > 1000) {
    calculaTensaoRede();
    if(StatusLiga || StatusRevert){
      calculaCorrente();
      analizeCorrenteMotor();

      if(i > 9) {analisaTensaoRede();}
    }
    timer1 = millis();
  }
  if (habCallDB && !StatusEmerg) draw(11);
  
// ------------ DETECÇÃO DE CORRENTE ALTA E CORRENTE MUITO ALTA E CHAMADA CORRETA DAS TELAS, MENSAGEM PERMANECE POS 20S----------
  if(I_altaDetectada &&  !habCallDB && !StatusEmerg && !habCallDB && !V_baixaDetectada && !V_extBaixaDetectada && !statusSR1 && !statusST){
    draw(9);
    //LÓGICA DE ACIONAMENTO DO BEEM EM CASO DE CORRENTE ALTA 
    if (millis() - timeBeepCorrente >= timeBeep && !sTBeeb)
    {
     atualizaSaidaSS(buzzer,   1);
     sTBeeb = !sTBeeb;
     Serial.println("Buzzer_ON");
    }
    else if (millis() - timeBeepCorrente >= timeBeep*2 && sTBeeb)
    {
      atualizaSaidaSS(buzzer,   0);
      sTBeeb = !sTBeeb;
      auxCont++;
      Serial.println("Buzzer_OFF");
      Serial.println();
      Serial.println(auxCont*timeBeep*2);
      Serial.println();
      
      //CASO PERMANEÇA POR MAIS DE 50s COM CORRENTE ALTA O EQUIPAMENTO SE DESLIGA SOZINHO
      if (auxCont*timeBeep*2 >= tempTurnOff*1000)
      {
        V_baixaDetectada = false;
        V_extBaixaDetectada = false;
        I_altaDetectada = false;
        I_muitoAltaDetectada = false;
        condGravaEprom  =   true;
        delayGravaEprom = millis();
        Serial.println();
        Serial.print("Habilita Grava Flash");
        fRamp = 1;

        StatusLiga      =   false;
        StatusRevert    =   false;
        statuDesl       =   true; 

        atualizaSaidaSS(byPass,   0);
        atualizaSaidaSS(hora,     0);
        atualizaSaidaSS(antHor,   0);
        atualizaSaidaSS(triac,    0);
        atualizaSaidaSS(buzzer,   0);
        auxCont = 0;

          if(confLinha < linha && !StGravaFlah){
            grava_dado_nvs(linha , 'L');
          }  
      } 
      timeBeepCorrente = millis();
    }

    //APÓS 3 SEGUNDO SEM DETECTAR CORENTE ALTA VOLTA TELA E DESLIGA BEEP
    if(millis() - tempSobCorrent > 3000){
      I_altaDetectada = false;
      contSobreCorre = 0;

      if(StatusLiga){
        draw(5);
       // auxCont = 0;
        sTBeeb  = 0;
        atualizaSaidaSS(buzzer,   0);
      } 
      if(StatusRevert){
        draw(6);
       // auxCont = 0;
        sTBeeb  = 0;
        atualizaSaidaSS(buzzer,   0);
      }

    }
  }
  else if (I_muitoAltaDetectada && !habCallDB && !StatusEmerg && !habCallDB && !V_baixaDetectada && !V_extBaixaDetectada && !statusSR1 && !statusST) {
    draw(10);
    //LÓGICA DE ACIONAMENTO DO BEEM EM CASO DE CORRENTE ALTA 
    if (millis() - timeBeepCorrente >= 300 && !sTBeeb)
    {
     atualizaSaidaSS(buzzer,   1);
     sTBeeb = !sTBeeb;
    }
    else if (millis() - timeBeepCorrente >= 600 && sTBeeb)
    {
      atualizaSaidaSS(buzzer,   0);
      sTBeeb = !sTBeeb;
    }

    //APÓS 20 SEGUNDO SEM DETECTAR CORENTE ALTA VOLTA TELA E DESLIGA BEEP
    if(millis() - tempSobCorrent > 2000){
      I_muitoAltaDetectada = false;
      contExtSobreCorre = 0;
      if(StatusLiga) draw(5);
      if(StatusRevert) draw(6);
    }
  }
//------------------------------------------------------------------------------------------------------------------------------
// ------------ DETECÇÃO DE TENSÃO BAIXA E TENSAÃO MUITO BAIXA E CHAMADA CORRETA DAS TELAS, MENSAGEM PERMANECE POS 2S----------
//------------------------------------------------------------------------------------------------------------------------------
  if(V_extBaixaDetectada && !habCallDB && !StatusEmerg && !habCallDB && !statusSR1 && !statusST){
    draw(8);
    if(millis() - tempSubtensao > 2000){
      V_extBaixaDetectada = false;
      contSubTensaoExtrema = 0;
      if(StatusLiga) draw(5);
      if(StatusRevert) draw(6);
    }
  }
  
  else if(V_baixaDetectada && !habCallDB && !StatusEmerg && !habCallDB && !statusSR1 && !statusST){
    draw(7);
    if(millis() - tempSubtensao > 2000){
      V_baixaDetectada = false;
      contSubTensao = 0;
      if(StatusLiga) draw(5);
      if(StatusRevert) draw(6);
    }
  }

//------------------------------------------------------------------------------------------------------------------------------
//                                  ------------ SISTEMA DE PARTIDA DO MOTOR  ----------
//------------------------------------------------------------------------------------------------------------------------------
  if((StatusLiga || StatusRevert) && (millis() - finalRamp >= 600) && fRamp == 0){
    atualizaSaidaSS(triac,   1); 
    if (millis() - finalRamp >= 1000)
    {
      atualizaSaidaSS(byPass,   1);
      delay(200);
      atualizaSaidaSS(triac, 0);
      fRamp = 1;
    }
  }

}/*END LOOP*/

/*==================================================================================
                FUNÇÃO DE LEITURA DOS STATUS DAS ENTRADAS "SENSORES SEGURANÇA"  
====================================================================================*/
void statusSensores(){
 statusSR1  =  digitalRead(fcPr1);
 //statusSR2  =  digitalRead(fcPr2);
 statusST   =  digitalRead(fcPt);

 if(!digitalRead(btEmegencia)) StatusEmerg = false;
 telafixa(!statusSR1, !statusST, !StatusEmerg);

 if(!statusSR1 && !statusST && !StatusEmerg ){
    statusSeguranca = true;     // Moenda habilitada para operar 
    if(((!StatusLiga && statuDesl) || (!StatusRevert && statuDesl)) && !V_baixaDetectada && !V_extBaixaDetectada && !I_altaDetectada && !I_muitoAltaDetectada && !habCallDB && !habLunbrificacao)
    draw(1);
  }

  else if (StatusEmerg) {
    statusSeguranca  = false;
    StatusLiga = false;
    StatusRevert = false;
    statuDesl = true;
    atualizaSaidaSS(byPass,   false);
    atualizaSaidaSS(hora,     false);
    atualizaSaidaSS(antHor,   false);
    atualizaSaidaSS(triac,    false);
    draw(4);
  }

  else if (statusSR1 && !habCallDB) {
    statusSeguranca  = false;
    StatusLiga = false;
    StatusRevert = false;
    statuDesl = true;
    atualizaSaidaSS(byPass,   false);
    atualizaSaidaSS(hora,     false);
    atualizaSaidaSS(antHor,   false);
    atualizaSaidaSS(triac,    false);
    draw(2);
  }

  else if (statusST && !habCallDB) {
    statusSeguranca  = false;
    StatusLiga = false;
    StatusRevert = false;
    statuDesl = true;
    atualizaSaidaSS(byPass,   false);
    atualizaSaidaSS(hora,     false);
    atualizaSaidaSS(antHor,   false);
    atualizaSaidaSS(triac,    false);
    draw(3);
  }
}

/*==================================================================================
                FUNÇÃO DE ATUALIZAÇÃO DAS SAÍDAS DO SHIFT SELECT  
====================================================================================*/
 void atualizaSaidaSS(int posicao, int i){
   if(i) bitSet(saidaShSel, posicao);
   else bitClear(saidaShSel,posicao);
 
  //  delay(200);
    digitalWrite(stcp, LOW);                           // Avisa o Clock SHCP que vai ser escrito um BIT
    shiftOut(ds, shcp, LSBFIRST, saidaShSel);
    digitalWrite(stcp, HIGH);                 

  }
/*==================================================================================
                          FUNÇÃO PARA CÁLCULO DA CORRENTE DO MOTOR  
====================================================================================*/
void calculaCorrente(){
 //DESCOMENTAR AS DUAS PRÓXIMAS LINHAS PARA A LEITURA PARA A LEITURA CORRETA DA CORRENTE COM O SENSOR NÃO INVASIVO 
  /* --------   SIMULAÇÃO DE PARA TESTES CORRENTE ALTA -------------*/
  /*if(seg >5 && seg <15)Irms = 15;
  else if (seg >= 15 && seg < 25)Irms = 8;
  else if (seg >= 25 && seg < 35)Irms = 15;
  else Irms = 8;*/
  
  //Irms = 8;
  Irms = emon1.calcIrms(1480);        // Configura número de amostras para a corrente 
  Serial.println(Irms );
  if(Irms < 1) Irms = Irms*0;         // Caso a corrente for menor que 1A desconsidera a corrente drenada 
 //Irms = 11.1;
}

/*==================================================================================
                          FUNÇÃO PARA CÁLCULO DA TENSÃO DA REDE  
====================================================================================*/
void calculaTensaoRede(){
 if(i > 9){
    for(int b = 9 ; b > 0 ; b--){
      vetTensaoRede[b] = vetTensaoRede[b-1];
     }
    vetTensaoRede[0] = analogRead(tensaoRede);

    /*Serial.println("VETOR DE TENSÃO DA REDE");
    for(int c=0 ; c < 10 ; c++){
      Serial.println(vetTensaoRede[c]);
    }*/
    uint16_t somaTensao;
    for(int c = 0 ; c < 10 ; c++){
      somaTensao  = somaTensao + vetTensaoRede[c];
     }

    /*Serial.print("SOMA DAS TENSÕES DA REDE - ");
    Serial.println(somaTensao);
    Serial.println();*/

    uint16_t mediaTensao = somaTensao / 10;
    
   /* Serial.print("MEDIA DAS TENSOES - ");
    Serial.println(mediaTensao);
    Serial.println();*/

    tensaoDaRede = (226 * mediaTensao) / 4096;    // Primeiros 3 Protótipos
    //tensaoDaRede = (265 * mediaTensao) / 4096;    //Moenda Protótipo

    /*-------------  SIMULAÇÃO PARA TESTES DE BAIXA TENSÃO -------------*/
    /*if(seg >5 && seg <20)tensaoDaRede = 208;
     else if (seg >= 20 && seg < 35)tensaoDaRede = 221;
     else if (seg >= 35 && seg < 50)tensaoDaRede = 208;
     else tensaoDaRede = 221;
     Serial.print("Tensão Rede :");
     Serial.println(tensaoDaRede);*/


    /*Serial.print("TENSÃO CONVERTIDA - ");
    Serial.println(tensaoDaRede);
    Serial.println();*/
  }
 else{
   vetTensaoRede[i] = analogRead(tensaoRede);
   tensaoDaRede = (226 * vetTensaoRede[i])/ 4096;
   //tensaoDaRede = (265 * vetTensaoRede[i])/ 4096; Moenda Protótipo 
   if(tensaoDaRede > 180) nivelTensao = false;    //Tensão da rede de alimentação é 220Vca
   else nivelTensao = true;                       //Tensão da rede de alimentação é 127Vca
   //Serial.println("PREENCHENDO O VETOR");
   Serial.print("Amostra[");
   Serial.print(i);
   Serial.print("]");
   Serial.print("-");
   Serial.println(tensaoDaRede);
   i++;
 }  
}

/*=====================================================================================
                  FUNÇÃO QUE REALIZA PARA VERIFICAR SUBTENSÃO   
=======================================================================================*/
void analisaTensaoRede(){

  // VERIFICA SUBTENSÃO PARA TENSÃO DE TRABALHO DE 220V
  if(!nivelTensao && tensaoDaRede > 203 && tensaoDaRede < 210){
    contSubTensao++;
    Serial.println(contSubTensao);

    if(contSubTensao >= 5 && !V_baixaDetectada && StatusLiga){
      //Armazenar no banco de cados 
      V_baixaDetectada = true;
      analisPreArmaz();
//    montaDB(horas, minutos, Irms*10, tensaoDaRede);
      tempSubtensao = millis();
    }
  }
  if(!nivelTensao && tensaoDaRede >= 210) contSubTensao = 0;

  // VERIFICA SUBTENSÃO EXTREMA PARA TENSÃO DE TRABALHO DE 220V
  if(!nivelTensao && tensaoDaRede <= 203){
    contSubTensaoExtrema++;

    if(contSubTensaoExtrema >= 5 && !V_extBaixaDetectada  && StatusLiga){  // 
      //Armazenar no banco de cados 
      V_extBaixaDetectada = true;
      analisPreArmaz();
   //   montaDB(horas, minutos, Irms*10, tensaoDaRede);
      tempSubtensao = millis();
    }
  }
  if(!nivelTensao && tensaoDaRede > 203) contSubTensaoExtrema = 0; 

  //-----------------------------------------------------------------------------

  // VERIFICA SUBTENSÃO PARA TENSÃO DE TRABALHO DE 127V
  if(nivelTensao && tensaoDaRede+12 > 116 && tensaoDaRede+12 < 121){
    contSubTensao++;

    if(contSubTensao >= 5 && !V_baixaDetectada  && StatusLiga){
      //Armazenar no banco de cados 
      V_baixaDetectada = true;
      analisPreArmaz();
 //   montaDB(horas, minutos, Irms*10, tensaoDaRede+12);
      tempSubtensao = millis();
    }
  }
  if(nivelTensao && tensaoDaRede+12 >= 121) contSubTensao = 0;


// VERIFICA SUBTENSÃO EXTREMA PARA TENSÃO DE TRABALHO DE 127V
  if(nivelTensao && tensaoDaRede+12 <= 116 ){
    contSubTensaoExtrema++;

    if(contSubTensaoExtrema >= 5 && !V_extBaixaDetectada  && StatusLiga){  // 
      //Armazenar no banco de cados 
      V_extBaixaDetectada = true;
      analisPreArmaz();

     // montaDB(horas, minutos, Irms*10, tensaoDaRede+12);
      tempSubtensao = millis();
    }
  }
  if(nivelTensao && tensaoDaRede > 116) contSubTensaoExtrema = 0; 
}

/*==================================================================================
          FUNÇÃO REALIZA A ANÁLISE DO NÍVEL DE CORRENTE DRENADA PELO MOTOR  
====================================================================================*/
void analizeCorrenteMotor(){
  // VERIFICA CORRENTE ALTA PARA TENSÃO DE TRABALHO DE 220V
  if(!nivelTensao && Irms > 12.5 && Irms < 14){
    contSobreCorre++;
    Serial.print("Contagem de Sobrecorrente - ");
    Serial.println(contSobreCorre);
  

    if(contSobreCorre >= 5 && !I_altaDetectada){
      //Armazenar no banco de cados 
      I_altaDetectada = true;
      analisPreArmaz();
      //montaDB(horas, minutos, Irms*10, tensaoDaRede);
      tempSobCorrent = millis();
      if (!habBeep)
      {
        timeBeepCorrente = millis();
        habBeep = !habBeep;
      }
    }
  }
  if(!nivelTensao && Irms <= 12) {
    contSobreCorre = 0; 
    habBeep = false;
    auxCont = 0;
  }

  // VERIFICA CORRENTE MUITO ALTA PARA TENSÃO DE TRABALHO DE 220V
  if(!nivelTensao && Irms >= 14){
    contExtSobreCorre++;

    if(contExtSobreCorre >= 5 && !I_muitoAltaDetectada){
      //Armazenar no banco de cados 
      I_muitoAltaDetectada = true;
      analisPreArmaz();
     
 //     montaDB(horas, minutos, Irms*10, tensaoDaRede);
      tempSobCorrent = millis();
      timeBeepCorrente = millis();
    }
  }
  if(!nivelTensao && Irms < 14) {
   contExtSobreCorre = 0;
   habBeep = false;
   auxCont = 0;
  }

  //------------------------------------------------------------------------------------------
 
  // VERIFICA CORRENTE ALTA PARA TENSÃO DE TRABALHO DE 127V
  if(nivelTensao && Irms > 25.5 && Irms < 27){
    contSobreCorre++;

    if(contSobreCorre >= 5 && !I_altaDetectada){
      //Armazenar no banco de cados 
      I_altaDetectada = true;
      analisPreArmaz();
    
    // montaDB(horas, minutos, Irms*10, tensaoDaRede);
      tempSobCorrent = millis();
      timeBeepCorrente = millis();
    }
  }
  if(nivelTensao && Irms <= 24) {
    contSobreCorre = 0;
    habBeep = false;
    auxCont = 0;
  }
  // VERIFICA CORRENTE MUITO ALTA PARA TENSÃO DE TRABALHO DE 127V
  if(nivelTensao && Irms >= 27){
    contExtSobreCorre++;
   // Serial.println(contExtSobreCorre);

    if(contExtSobreCorre >= 5 && !I_muitoAltaDetectada){
      //Armazenar no banco de cados 
      I_muitoAltaDetectada = true;
      analisPreArmaz();

    //montaDB(horas, minutos, Irms*10, tensaoDaRede);
      tempSobCorrent = millis();
      timeBeepCorrente = millis();
    }
  }
  if(nivelTensao && Irms < 25.5) {
    contSobreCorre = 0;
    habBeep = false;
    auxCont = 0;
  }
}

/*=====================================================================================
                  FUNÇÃO PARA ATUALIZAÇÃO LED DOS BOTÕES   
=======================================================================================*/
void atualizaLedBotoes(){
  if(statuDesl) {
    atualizaSaidaSS(ledLiga,   false);  
    atualizaSaidaSS(ledRevert, false);
    atualizaSaidaSS(ledDesl,   true); 
  }

  else if (StatusLiga ){
    atualizaSaidaSS(ledLiga,    true);  
    atualizaSaidaSS(ledRevert,  false);
    atualizaSaidaSS(ledDesl,    false); 
  }

  else if (StatusRevert ){
    atualizaSaidaSS(ledLiga,    false);  
    atualizaSaidaSS(ledRevert,  true);
    atualizaSaidaSS(ledDesl,    false); 
  }
}

/*==================================================================================
                FUNÇÃO DE INICIALIZAÇÃO DO DISPLAY  
====================================================================================*/
  void inicializadisplay() {
    u8g2.setFont(u8g2_font_helvB10_tf);
    u8g2.setFontRefHeightExtendedText();
    u8g2.setDrawColor(1);
    u8g2.setFontPosTop();
    u8g2.setFontDirection(0);
}


/*==================================================================================
                FUNÇÃO DE CONSTRUÇÃO DA TELA FIXA  
====================================================================================*/
void  telafixa(boolean SR1, boolean ST, boolean EM) {

  u8g2.setFont(u8g2_font_timR08_tr);
  u8g2.drawLine(34, 0, 34, 100);
  u8g2.drawLine(34, 25, 200, 25);
  u8g2.drawStr(23, 1, "h"); //TÍTULO HORÍMETRO
  u8g2.setFont(u8g2_font_timR08_tr);
  u8g2.drawStr(1, 14, "SR"); //TÍTULO SENSOR ROLO 1
 // u8g2.setFont(u8g2_font_timR08_tr);
 // u8g2.drawStr(1, 25, "SR2"); //TÍTULO SENSOR ROLO 2
  u8g2.setFont(u8g2_font_timR08_tr);
  u8g2.drawStr(1, 32, "ST"); //TÍTULO SENSOR TRANSMISSÃO 2
  u8g2.setFont(u8g2_font_timR08_tr);
  u8g2.drawStr(1, 50, "EM"); //TÍTULO SENSOR TRANSMISSÃO 2

  //////////////INDICAÇÃO DOS SENSORES E EMERGÊNCIA////////////////
if (SR1 == 0)  // VERIFICA O ESTADO DO SENSOR ROLO 1
u8g2.drawCircle(27,18,4);
else
u8g2.drawDisc(27,18,4);

/*if (SR2 == 0)   // VERIFICA O ESTADO DO SENSOR ROLO 2
u8g2.drawCircle(27,29,4);
else
u8g2.drawDisc(27,29,4);*/

if (ST == 0)   // VERIFICA O ESTADO DO SENSOR DE TRANSMISSÃO
u8g2.drawCircle(27,36,4);
else
u8g2.drawDisc(27,36,4);

if (EM == 0)    // VERIFICA O ESTADO DO BOTÃO DE EMERGÊNCIA
u8g2.drawCircle(27,54,4);
else
u8g2.drawDisc(27,54,4);
  }


/*==================================================================================
                FUNÇÃO DE INICAIALICAÇAÕ DA MOENDA COM A MARCA DA MAQTRON
====================================================================================*/
 void inicia_moenda(){
 // Serial.println("Inicializou Moenda");
 if(tempoinicializacao < 3000){
  if (tempologo==1)
  {
    u8g2.setFontMode(1);  
    u8g2.setDrawColor(1); 
    u8g2.drawRBox(9,7,110,50,25);
    u8g2.setFont(u8g2_font_ncenB12_tf);
    u8g2.setDrawColor(0);
    u8g2.drawStr(16, 26, "MAQTRON");
  }
  else {
    u8g2.setFontMode(1);  
    u8g2.setDrawColor(1); 
    u8g2.drawRBox(9,7,110,50,25);
  }
 }

  ///////////////////////TEMPO DE INICIALIZAÇÃO TELA DE APRESENTAÇÃO//////////////////////
  if(tempoinicializacao>=3000){
    u8g2.setFontMode(1);  
    u8g2.setDrawColor(1);
    u8g2.drawRBox(9,7,110,50,25);
    u8g2.setFont(u8g2_font_ncenB12_tf);
    u8g2.setDrawColor(0);
    u8g2.drawStr(16, 26, "MAQTRON");
  }
} 

/*==================================================================================
                FUNÇÃO DE APRESENTAÇÃO DA TEMA "EM OPERAÇÃO"
====================================================================================*/
void aguardandooperacao(){
  if (pisca == 0) {
    u8g2.setFont(u8g2_font_timR08_tr);
    u8g2.drawStr(43, 4, "AGUARDANDO");
    u8g2.drawStr(54, 12, "COMANDO");
  }
  if (pisca == 1){
    u8g2.setFont(u8g2_font_timR08_tr);
    u8g2.drawStr(67, 0, "LIGA");
    u8g2.drawStr(72, 8, "OU");
    u8g2.drawStr(57, 16, "REVERTE");
  }
    u8g2.setFont(u8g2_font_timB10_tr);
    u8g2.drawStr(38, 30, "MOENDA");
    u8g2.drawStr(38, 45, "EM ESPERA");
}

/*==================================================================================
                   TEMPO  INICIALIZAÇÃO LOGOTIPO MAQTRON
====================================================================================*/

void tempoparainicializacao(){
  // TEMPO PARA PISCAR MENSAGENS DE ALERTA
  if((millis() - tempoinicializacao) < 1000)
  {
    tempologo = 1;
  }else
  {
    tempologo=0;
  }
  if ((millis() - tempoinicializacao) >= 2000)
  {
    tempoinicializacao =millis(); 
  }
}

/*==================================================================================
                  FUNÇÃO DE SELEÇÃO DE TELA DO DISPLAY
====================================================================================*/
void draw(int tela){
     u8g2.firstPage();  
  do {
      inicializadisplay();
      switch (tela) {
      case 0: //chama tela de inicialização com a marca da maqtron
        if (tempoinicializacao < 8000)
        inicia_moenda();
        break;

      case 1: //Apresenta a tela fixa
        telafixa(!statusSR1, !statusST, !StatusEmerg);
        contagemHoras(horas);
        aguardandooperacao();
        reiniciaDisp(tela);
        break;

      case 2: //Apresenta as informações em relação a falta de sinal em um dos sensores de proteção dos rolos 
        telafixa(!statusSR1, !statusST, !StatusEmerg);
        contagemHoras(horas);
        prot_rolos(!statusSR1);
        break;

      case 3: //Apresenta as informações em relação a falta de sinal do sensor de protação das transmissões 
        telafixa(!statusSR1, !statusST, !StatusEmerg);
        contagemHoras(horas);
        prot_transmissao(statusST);
        break;

      case 4: //Apresenta as informações caso o botão de emergência tenha sido acionado 
        telafixa(!statusSR1, !statusST, !StatusEmerg);
        contagemHoras(horas);
        emergencia(StatusEmerg);
        controleDisp = tela;
        break;

      case 5: //Apresenta tela da máquina em operação "LIGADA"
        telafixa(!statusSR1, !statusST, !StatusEmerg);
        contagemHoras(horas);
        emOperacaoLigada();
        controleDisp = tela;
        break;

      case 6: //Apresenta a tela da máquina em operação "REVERSÂO"
        telafixa(!statusSR1, !statusST, !StatusEmerg);
        contagemHoras(horas);
        emOperacaoReversa();
        controleDisp = tela;
        break;

      case 7: //Apresenta a tela de alerta de tensão "BAIXA" 
        telafixa(!statusSR1, !statusST, !StatusEmerg);
        contagemHoras(horas);
        alerta_Vbaixa();
        controleDisp = tela;
        break;

      case 8: //Apresenta a tela de alerta de tensão "MUITO BAIXA"
        telafixa(!statusSR1, !statusST, !StatusEmerg);
        contagemHoras(horas);
        alerta_Vmuitobaixa();
        controleDisp = tela;
        break;

      case 9: // Apresenta a tela de corrente "ALTA"
        telafixa(!statusSR1, !statusST, !StatusEmerg);
        contagemHoras(horas);
        alerta_Ialta();
        controleDisp = tela;
        break;

      case 10: // Apresenta a tela de "CORRENTE MUITO ALTA"
        telafixa(!statusSR1, !statusST, !StatusEmerg);
        contagemHoras(horas);
        alertaImuitoalta();
        controleDisp = tela;
        break;

      case 11: // Apresenta a tela puxando o banco de dados da Flash do microcontrolador
        banco_de_dados();
        break;

      case 12: // Apresenta a tela puxando o banco de dados da Flash do microcontrolador
        telafixa(!statusSR1, !statusST, !StatusEmerg);
        contagemHoras(horas);
        lubrifique();
        break;

      default:
        break;
      }

  } while( u8g2.nextPage() );
}

/*==================================================================================
                  FUNÇÃO DE TEMPO DE APRESENTAÇÃO DAS MENSAGENS 
====================================================================================*/
void tempomensagensdealerta()
{
  // TEMPO PARA PISCAR MENSAGENS DE ALERTA
  if((millis() - piscaalerta) < 2000){
    pisca = 1;

  }else{
    pisca=0;
  }

  if ((millis() - piscaalerta) >= 4000)  {
    piscaalerta =millis(); 
  }
}

/*==================================================================================
FUNÇÃO QUE APRESENTA AS TELAS CASO HAJA FALA EM UM DOS SENSORES DE PROTEÇÃO DOS ROLOS 
====================================================================================*/
void  prot_rolos(boolean SR1){

  if(SR1==0)//SINALIZA QUE O 1 ESTÁ SEM SINAL
  {
if (pisca == 1)
  {
    u8g2.setFont(u8g2_font_timR08_tr);
    u8g2.drawStr(50, 3, "PARA LIGAR ");
    u8g2.drawStr(62, 14, "INSIRA  ");
    
    /*u8g2.drawStr(55, 3, "FALHA NA ");
    u8g2.drawUTF8(91, 9, "\U0000007E");
    u8g2.drawStr(57, 14, "FIXACAO");
    u8g2.drawUTF8(85, 15, ",");*/
      }
  else{
    u8g2.setFont(u8g2_font_timR08_tr);
    u8g2.drawUTF8(100,-2, "\U0000007E");
    u8g2.drawStr(50, 3, "A PROTECAO ");
    //u8g2.drawUTF8(85,-2, "\U0000007E");
    //u8g2.drawStr(45, 3, "PROTECAO ");
    u8g2.drawUTF8(94, 4, ",");
    u8g2.drawStr(55, 14, "DOS ROLOS");
    //u8g2.drawStr(55, 14, "ROLOS");
    //u8g2.drawStr(85, 14, "SR1");
    }
    u8g2.setFont(u8g2_font_timR08_tr);
    u8g2.drawStr(107, 31, "E05");
    u8g2.drawFrame(105, 30,19,12);
    u8g2.setFont(u8g2_font_timB10_tr);
    u8g2.drawStr(38, 30, "MOENDA");
    u8g2.drawStr(38, 45, "INOPERANTE");
  }

}

/*==================================================================================
FUNÇÃO QUE APRESENTA AS TELAS CASO HAJA FALA NO SENSOR DE PROTEÇÃO DAS TRANSMISSÕES 
====================================================================================*/
void  prot_transmissao(boolean ST){
  if(ST)  // VERIFICA A CONDIÇÃO DO SENSOR DE TRANSMISSÃO
  {
if (pisca == 1)
  {
u8g2.setFont(u8g2_font_timR08_tr);
u8g2.drawUTF8(87, -2, "\U0000007E");  // CONFIGURAÇÃO PARA O "~" APARECER CORRETAMENTE
u8g2.drawStr(47, 4, "PROTECAO DA");
u8g2.drawUTF8(81, 5, ",");
u8g2.drawUTF8(106, 9, "\U0000007E");  // CONFIGURAÇÃO PARA O "~" APARECER CORRETAMENTE
u8g2.drawStr(47, 14, "TRANSMISSAO ");

  }
  else{
    u8g2.setFont(u8g2_font_timR08_tr);
    u8g2.drawStr(55, 9, "ABERTA ST");
    }
    u8g2.setFont(u8g2_font_timB10_tr);
    u8g2.drawStr(38, 30, "MOENDA");
    u8g2.drawStr(38, 45, "INOPERANTE");
    u8g2.setFont(u8g2_font_timR08_tr);
    u8g2.drawStr(107, 31, "E08");
    u8g2.drawFrame(105, 30,19,12);
  }
}
/*==================================================================================
    FUNÇÃO RESPONSÁVEL POR APRESENTAR A TELA EM CASO DE EMERGÊNCIA PRESSIONADO 
====================================================================================*/
void  emergencia(boolean EM){
if(EM){
  if (pisca == 0)  {
      u8g2.setFont(u8g2_font_timR08_tr);
      u8g2.drawStr(83, 4, "^");
      u8g2.drawStr(47, 8, "EMERGENCIA");
    }
    u8g2.setFont(u8g2_font_timB10_tr);
    u8g2.drawStr(38, 30, "MOENDA");
    u8g2.drawStr(38, 45, "INOPERANTE");
    }
}

/*==================================================================================
      FUNÇÃO  APRESENTAR A TELA DA MOENDA EM OPERAÇÃO "LIGADA"
====================================================================================*/
void  emOperacaoLigada(){
  u8g2.setFont(u8g2_font_timR08_tr);
  u8g2.drawStr(52, 7, "OPERANDO");
  u8g2.setFont(u8g2_font_timB10_tr);
  u8g2.drawStr(38, 30, "MOENDA");
  u8g2.drawStr(38, 45, "LIGADA");
}

/*==================================================================================
      FUNÇÃO  APRESENTAR A TELA DA MOENDA EM OPERAÇÃO "REVERSA"
====================================================================================*/
void emOperacaoReversa(){
  	u8g2.setFont(u8g2_font_timR08_tr);
	  u8g2.drawStr(52, 2, "OPERANDO");
	  u8g2.drawUTF8(103, 8, "\U0000007E");
	  u8g2.drawStr(45, 14, "EM REVERSAO");
	  u8g2.setFont(u8g2_font_timB10_tr);
	  u8g2.drawStr(38, 30, "MOENDA");
	  u8g2.drawStr(38, 45, "LIGADA");
}

/*==================================================================================
                             FUNÇÃO CALCULO DO HORÍMETRO  
====================================================================================*/
void atualizaHorimetro(){
  if(millis() - horimetro >= 1000) {
    seg++;
    Serial.print("Segundos = ");
    Serial.println(seg);
//-----CONTAGEM DOS MINUTOS ----------
    if(seg == 59){
      minutos++;
      Serial.print("Minutos =");
      Serial.println(minutos);
      seg = 0;
    }
//-----CONTAGEM DAS HORAS----------
    if(minutos == 59){
      horas++;
      Serial.print("Horas =");
      Serial.println(horas);
      grava_dado_nvs(horas, 'H'); //GRAVA HORAS NA FLASH
      minutos = 0;
    }
    horimetro = millis();
  }
}

/*==================================================================================
              FUNÇÃO QUE REALIZA A ATUALIZAÇÃO DAS HORAS NO DISPLAY  
====================================================================================*/
void contagemHoras(uint16_t horaFuc){
  char h[5] = "";
    sprintf(h, "%d", horaFuc); 
    if (horaFuc <= 9)
    u8g2.drawStr(15,1,h);
    if(horaFuc > 9 && horaFuc <= 99) 
    u8g2.drawStr(10,1,h);
    if(horaFuc > 99 && horaFuc <= 999) 
    u8g2.drawStr(5,1,h);
    if(horaFuc > 999) 
    u8g2.drawStr(0,1,h);
}

/*==================================================================================
                 FUNÇÃO DE CHAMADA DA TELA INDICANDO A TENSÃO BAIXA 
====================================================================================*/
void alerta_Vbaixa(){
if (pisca == 0)  {
  u8g2.setFont(u8g2_font_timR08_tr);
  u8g2.drawUTF8(88,-2, "\U0000007E");  // CONFIGURAÇÃO PARA O "~" APARECER CORRETAMENTE
  u8g2.drawStr(62, 4, "TENSAO");
  u8g2.drawStr(65, 13, "BAIXA");
}
else{
  u8g2.setFont(u8g2_font_timB10_tr);
  u8g2.drawUTF8(98, 0, "\U0000007E");
  u8g2.drawStr(48, 7, "ATENCAO");
  u8g2.drawUTF8(91, 8, ",");
  }
if(StatusLiga || StatusRevert){
  u8g2.setFont(u8g2_font_timB10_tr);
  u8g2.drawStr(38, 30, "MOENDA");
  u8g2.drawStr(38, 45, "LIGADA");
  u8g2.setFont(u8g2_font_timR08_tr);
  u8g2.drawStr(107, 31, "E01");
  u8g2.drawFrame(105, 30,19,12);
  }
else{ 
  u8g2.setFont(u8g2_font_timB10_tr);
  u8g2.drawStr(38, 30, "MOENDA");
  u8g2.drawStr(38, 45, "EM ESPERA");
  u8g2.setFont(u8g2_font_timR08_tr);
  u8g2.drawStr(107, 31, "E01");
  u8g2.drawFrame(105, 30,19,12);}
  }


/*==================================================================================
          FUNÇÃO DE CHAMADA DA TELA INDICANDO A TENSÃO MUITO BAIXA 
====================================================================================*/
void alerta_Vmuitobaixa(){
  if (pisca == 0)  {
    u8g2.setFont(u8g2_font_timR08_tr);
    u8g2.drawUTF8(88,-2, "\U0000007E");  // CONFIGURAÇÃO PARA O "~" APARECER CORRETAMENTE
    u8g2.drawStr(62, 4, "TENSAO");
    u8g2.drawStr(47, 13, "MUITO BAIXA");
  }
 if((StatusLiga || StatusRevert) && pisca!=0)  {
    u8g2.setFont(u8g2_font_timB10_tr);
    u8g2.drawStr(47, 1, "DESLIGUE");
    u8g2.drawStr(44, 12, "A MOENDA");
    }

  if(StatusLiga || StatusRevert){
    u8g2.setFont(u8g2_font_timB10_tr);
    u8g2.drawStr(38, 30, "MOENDA");
    u8g2.drawStr(38, 45, "LIGADA");
    u8g2.setFont(u8g2_font_timR08_tr);
    u8g2.drawStr(107, 31, "E02");
    u8g2.drawFrame(105, 30,19,12);
  }
  else{
    u8g2.setFont(u8g2_font_timB10_tr);
    u8g2.drawStr(38, 30, "MOENDA");
    u8g2.drawStr(38, 45, "EM ESPERA");
    u8g2.setFont(u8g2_font_timR08_tr);
    u8g2.drawStr(107, 31, "E02");
    u8g2.drawFrame(105, 30,19,12);
  }  
}


/*==================================================================================
          FUNÇÃO DE CHAMADA DA TELA INDICANDO ALERTA DE CORRENTE ALTA  
====================================================================================*/
void  alerta_Ialta(){
  if (pisca == 0) {
    u8g2.setFont(u8g2_font_timR08_tr);
    u8g2.drawStr(55, 5, "CORRENTE");
    u8g2.drawStr(67, 13, "ALTA");
  }
  else  {
    u8g2.setFont(u8g2_font_timB10_tr);
    u8g2.drawUTF8(97, 0, "\U0000007E");
    u8g2.drawStr(47, 7, "ATENCAO");
    u8g2.drawUTF8(90, 8, ",");
  }
    u8g2.setFont(u8g2_font_timR08_tr);
    u8g2.drawStr(107, 31, "E03");
    u8g2.drawFrame(105, 30,19,12);
    u8g2.setFont(u8g2_font_timB10_tr);
    u8g2.drawStr(38, 30, "MOENDA");
    u8g2.drawStr(38, 45, "LIGADA");
}


/*==================================================================================
       FUNÇÃO DE CHAMADA DA TELA INDICANDO ALERTA DE CORRENTE  MUITO ALTA  
====================================================================================*/
void alertaImuitoalta(){
  if (pisca == 0)    {
    u8g2.setFont(u8g2_font_timR08_tr);
    u8g2.drawStr(55, 5, "CORRENTE");
    u8g2.drawStr(50, 13, "MUITO ALTA");
  }
  else{
    u8g2.setFont(u8g2_font_timB10_tr);
    u8g2.drawStr(47, 1, "DESLIGUE");
    u8g2.drawStr(44, 12, "A MOENDA");
  }
  u8g2.setFont(u8g2_font_timR08_tr);
  u8g2.drawStr(107, 31, "E04");
  u8g2.drawFrame(105, 30,19,12);
  u8g2.setFont(u8g2_font_timB10_tr);
  u8g2.drawStr(38, 30, "MOENDA");
  u8g2.drawStr(38, 45, "LIGADA");
}


/*==================================================================================
            FUNÇÃO DE REALIZA A ESCRITA DE DADOS NA MEMÓRIA RETENTIVA 
====================================================================================*/
/* Função: grava na NVS um dado do tipo interio 32-bits
 *         sem sinal, na chave definida em CHAVE_NVS
 * Parâmetros: dado a ser gravado
 * Retorno: nenhum
 */
void grava_dado_nvs(uint16_t dado, char leitura){
    nvs_handle handler_particao_nvs;
    esp_err_t err;
    
    err = nvs_flash_init_partition("nvs");
     
    if (err != ESP_OK) {
        Serial.println("[ERRO] Falha ao iniciar partição NVS.");           
        return;
    }
    err = nvs_open_from_partition("nvs", "ns_nvs", NVS_READWRITE, &handler_particao_nvs);
    if (err != ESP_OK) {
        Serial.println("[ERRO] Falha ao abrir NVS como escrita/leitura"); 
        return;
    }
    /* Atualiza valor do horimetro total */
    switch (leitura)   {
    case 'H': //REALIZA A GRAVAÇÃO DA HORA 
      err = nvs_set_u16(handler_particao_nvs, CHAVE_HORA, dado);
      break;

    case 'M': //REALIZA A GRAVAÇÃO DOS MINUTOS 
      err = nvs_set_u16(handler_particao_nvs, CHAVE_MIN, dado);
      Serial.println("MINUTOS GRAVADOS");
      break;

    case 'S': //REALIZA A GRAVAÇÃO DOS MINUTOS 
      err = nvs_set_u16(handler_particao_nvs, CHAVE_SEG, dado);
      Serial.println("SEGUNDOS GRAVADOS");
      break;

    case 'P': //REALIZA A GRAVAÇÃO DOS MINUTOS 
      err = nvs_set_u16(handler_particao_nvs, CHAVE_PONT, dado);
      Serial.println("NUMERO DE PONTOS GRAVADOS");
      break;

    case 'A': //GRAVA NA FLASH A HORA MÁQUINA QUE OCORREU O ALARME 
      if(linha == 0)      err =  nvs_set_u16(handler_particao_nvs, Hr00, dado);
      else if(linha == 1) err =  nvs_set_u16(handler_particao_nvs, Hr10, dado);
      else if(linha == 2) err =  nvs_set_u16(handler_particao_nvs, Hr20, dado);
      else if(linha == 3) err =  nvs_set_u16(handler_particao_nvs, Hr30, dado);
      else if(linha == 4) err =  nvs_set_u16(handler_particao_nvs, Hr40, dado);
      else if(linha == 5) err =  nvs_set_u16(handler_particao_nvs, Hr50, dado);
      else if(linha == 6) err =  nvs_set_u16(handler_particao_nvs, Hr60, dado);
      else if(linha == 7) err =  nvs_set_u16(handler_particao_nvs, Hr70, dado);
      break;

    case 'B': //GRAVA NA FLASH O MINUTO MÁQUINA QUE OCORREU O PROBLEMA 
      if(linha == 0)      err =  nvs_set_u16(handler_particao_nvs, Mi01, dado);
      else if(linha == 1) err =  nvs_set_u16(handler_particao_nvs, Mi11, dado);
      else if(linha == 2) err =  nvs_set_u16(handler_particao_nvs, Mi21, dado);
      else if(linha == 3) err =  nvs_set_u16(handler_particao_nvs, Mi31, dado);
      else if(linha == 4) err =  nvs_set_u16(handler_particao_nvs, Mi41, dado);
      else if(linha == 5) err =  nvs_set_u16(handler_particao_nvs, Mi51, dado);
      else if(linha == 6) err =  nvs_set_u16(handler_particao_nvs, Mi61, dado);
      else if(linha == 7) err =  nvs_set_u16(handler_particao_nvs, Mi71, dado);
      break;

    case 'C': //GRAVA OS VALORES DE CORRENTE NA FLASH
      if(linha == 0)      err =  nvs_set_u16(handler_particao_nvs, I03, dado);
      else if(linha == 1) err =  nvs_set_u16(handler_particao_nvs, I13, dado);
      else if(linha == 2) err =  nvs_set_u16(handler_particao_nvs, I23, dado);
      else if(linha == 3) err =  nvs_set_u16(handler_particao_nvs, I33, dado);
      else if(linha == 4) err =  nvs_set_u16(handler_particao_nvs, I43, dado);
      else if(linha == 5) err =  nvs_set_u16(handler_particao_nvs, I53, dado);
      else if(linha == 6) err =  nvs_set_u16(handler_particao_nvs, I63, dado);
      else if(linha == 7) err =  nvs_set_u16(handler_particao_nvs, I73, dado);
      break;  

    case 'D': //GRAVA OS VALORES DE TENSÃO NA FLASH
      if(linha == 0)      err =  nvs_set_u16(handler_particao_nvs, V02, dado);
      else if(linha == 1) err =  nvs_set_u16(handler_particao_nvs, V12, dado);
      else if(linha == 2) err =  nvs_set_u16(handler_particao_nvs, V22, dado);
      else if(linha == 3) err =  nvs_set_u16(handler_particao_nvs, V32, dado);
      else if(linha == 4) err =  nvs_set_u16(handler_particao_nvs, V42, dado);
      else if(linha == 5) err =  nvs_set_u16(handler_particao_nvs, V52, dado);
      else if(linha == 6) err =  nvs_set_u16(handler_particao_nvs, V62, dado);
      else if(linha == 7) err =  nvs_set_u16(handler_particao_nvs, V72, dado);
      break; 

    case 'E': 
      if(recLin == 0){
        if      (recCol == 0) nvs_set_u16(handler_particao_nvs, Hr00, dado);
        else if (recCol == 1) nvs_set_u16(handler_particao_nvs, Mi01, dado);
        else if (recCol == 2) nvs_set_u16(handler_particao_nvs, I03, dado);
        else if (recCol == 3) nvs_set_u16(handler_particao_nvs, V02, dado); 
      }

      else if(recLin == 1){
        if      (recCol == 0) nvs_set_u16(handler_particao_nvs, Hr10, dado);
        else if (recCol == 1) nvs_set_u16(handler_particao_nvs, Mi11, dado);
        else if (recCol == 2) nvs_set_u16(handler_particao_nvs, I13, dado);
        else if (recCol == 3) nvs_set_u16(handler_particao_nvs, V12, dado); 
      }

      else if(recLin == 2){
        if      (recCol == 0) nvs_set_u16(handler_particao_nvs, Hr20, dado);
        else if (recCol == 1) nvs_set_u16(handler_particao_nvs, Mi21, dado);
        else if (recCol == 2) nvs_set_u16(handler_particao_nvs, I23, dado);
        else if (recCol == 3) nvs_set_u16(handler_particao_nvs, V22, dado); 
      }

      else if(recLin == 3){
        if      (recCol == 0) nvs_set_u16(handler_particao_nvs, Hr30, dado);
        else if (recCol == 1) nvs_set_u16(handler_particao_nvs, Mi31, dado);
        else if (recCol == 2) nvs_set_u16(handler_particao_nvs, I33, dado);
        else if (recCol == 3) nvs_set_u16(handler_particao_nvs, V32, dado); 
      }

      else if(recLin == 4){
        if      (recCol == 0) nvs_set_u16(handler_particao_nvs, Hr40, dado);
        else if (recCol == 1) nvs_set_u16(handler_particao_nvs, Mi41, dado);
        else if (recCol == 2) nvs_set_u16(handler_particao_nvs, I43, dado);
        else if (recCol == 3) nvs_set_u16(handler_particao_nvs, V42, dado); 
      }

      else if(recLin == 5){
        if      (recCol == 0) nvs_set_u16(handler_particao_nvs, Hr50, dado);
        else if (recCol == 1) nvs_set_u16(handler_particao_nvs, Mi51, dado);
        else if (recCol == 2) nvs_set_u16(handler_particao_nvs, I53, dado);
        else if (recCol == 3) nvs_set_u16(handler_particao_nvs, V52, dado); 
      }

      else if(recLin == 6){
        if      (recCol == 0) nvs_set_u16(handler_particao_nvs, Hr60, dado);
        else if (recCol == 1) nvs_set_u16(handler_particao_nvs, Mi61, dado);
        else if (recCol == 2) nvs_set_u16(handler_particao_nvs, I63, dado);
        else if (recCol == 3) nvs_set_u16(handler_particao_nvs, V62, dado); 
      }

      else if(recLin == 7){
        if      (recCol == 0) nvs_set_u16(handler_particao_nvs, Hr70, dado);
        else if (recCol == 1) nvs_set_u16(handler_particao_nvs, Mi71, dado);
        else if (recCol == 2) nvs_set_u16(handler_particao_nvs, I73, dado);
        else if (recCol == 3) nvs_set_u16(handler_particao_nvs, V72, dado); 
      }
      break;   

    case 'L': //SALVA A POSIÇÃO DA ULTIMA LINHA ESCRITA NA MATRIZ DO BANCO DE DADOS 
       nvs_set_u16(handler_particao_nvs, LN, dado);
      break;          
    
    default:
      break;
    }
  
    if (err != ESP_OK)    {
        Serial.println("[ERRO] Erro ao gravar horimetro");                   
        nvs_close(handler_particao_nvs);
        return;
    }
    else{
        Serial.println("Dado gravado com sucesso!");     
        nvs_commit(handler_particao_nvs);    
        nvs_close(handler_particao_nvs);      
    }
}

/*==================================================================================
            FUNÇÃO QUE REALIZA A LEITURA DE DADOS DA MEMÓRIA RETENTIVA
====================================================================================*/
uint16_t le_dado_nvs(char leitura)
{
    nvs_handle handler_particao_nvs;
    esp_err_t err;
    uint16_t dado_lido;
     
    err = nvs_flash_init_partition("nvs");
     
    if (err != ESP_OK)    {
        Serial.println("[ERRO] Falha ao iniciar partição NVS.");         
        return 0;
    }
 
    err = nvs_open_from_partition("nvs", "ns_nvs", NVS_READWRITE, &handler_particao_nvs);
    if (err != ESP_OK)  {
        Serial.println("[ERRO] Falha ao abrir NVS como escrita/leitura");         
        return 0;
    }
 
 switch (leitura) {
 case 'H': //FAZ A LEITURA DA HORA 
    err = nvs_get_u16(handler_particao_nvs, CHAVE_HORA, &dado_lido);
  break;

 case 'M': //FAZ A LEITURA DOS MINUTOS 
    err = nvs_get_u16(handler_particao_nvs, CHAVE_MIN, &dado_lido);
    Serial.println("Leu os dados de MINUTOS ");
  break;

case 'S': //FAZ A LEITURA DOS MINUTOS 
    err = nvs_get_u16(handler_particao_nvs, CHAVE_SEG, &dado_lido);
    Serial.println("Leu os dados de SEGUNDOS ");
  break;

case 'P': //FAZ A LEITURA DOS MINUTOS 
    err = nvs_get_u16(handler_particao_nvs, CHAVE_PONT, &dado_lido);
    Serial.println("Leu os dados de PONTOS gravados noa arquino LittleFS");
  break;

 case 'A': //RECUPERA AS HORAS PARA A MATRIZ DO BANCO DE DADOS 
    if(auxLin == 0 )       err = nvs_get_u16(handler_particao_nvs, Hr00, &dado_lido);
    else if (auxLin  == 1)  err = nvs_get_u16(handler_particao_nvs, Hr10, &dado_lido);
    else if (auxLin  == 2)  err = nvs_get_u16(handler_particao_nvs, Hr20, &dado_lido);
    else if (auxLin  == 3)  err = nvs_get_u16(handler_particao_nvs, Hr30, &dado_lido);
    else if (auxLin  == 4)  err = nvs_get_u16(handler_particao_nvs, Hr40, &dado_lido);
    else if (auxLin  == 5)  err = nvs_get_u16(handler_particao_nvs, Hr50, &dado_lido);
    else if (auxLin  == 6)  err = nvs_get_u16(handler_particao_nvs, Hr60, &dado_lido);
    else if (auxLin  == 7)  err = nvs_get_u16(handler_particao_nvs, Hr70, &dado_lido);
  break;

 case 'B': //FAZ A LEITURA DOS MINUTOS 
    if(auxLin  == 0 )       err = nvs_get_u16(handler_particao_nvs, Mi01, &dado_lido);
    else if (auxLin == 1)  err = nvs_get_u16(handler_particao_nvs, Mi11, &dado_lido);
    else if (auxLin  == 2)  err = nvs_get_u16(handler_particao_nvs, Mi21, &dado_lido);
    else if (auxLin  == 3)  err = nvs_get_u16(handler_particao_nvs, Mi31, &dado_lido);
    else if (auxLin  == 4)  err = nvs_get_u16(handler_particao_nvs, Mi41, &dado_lido);
    else if (auxLin  == 5)  err = nvs_get_u16(handler_particao_nvs, Mi51, &dado_lido);
    else if (auxLin  == 6)  err = nvs_get_u16(handler_particao_nvs, Mi61, &dado_lido);
    else if (auxLin  == 7)  err = nvs_get_u16(handler_particao_nvs, Mi71, &dado_lido);
  break;

 case 'C': //FAZ A LEITURA DOS MINUTOS 
    if(auxLin  == 0 )       err = nvs_get_u16(handler_particao_nvs, I03, &dado_lido);
    else if (auxLin  == 1)  err = nvs_get_u16(handler_particao_nvs, I13, &dado_lido);
    else if (auxLin  == 2)  err = nvs_get_u16(handler_particao_nvs, I23, &dado_lido);
    else if (auxLin  == 3)  err = nvs_get_u16(handler_particao_nvs, I33, &dado_lido);
    else if (auxLin  == 4)  err = nvs_get_u16(handler_particao_nvs, I43, &dado_lido);
    else if (auxLin  == 5)  err = nvs_get_u16(handler_particao_nvs, I53, &dado_lido);
    else if (auxLin  == 6)  err = nvs_get_u16(handler_particao_nvs, I63, &dado_lido);
    else if (auxLin  == 7)  err = nvs_get_u16(handler_particao_nvs, I73, &dado_lido);
  break;

 case 'D': //FAZ A LEITURA DOS MINUTOS 
    if(auxLin == 0 )         err = nvs_get_u16(handler_particao_nvs, V02, &dado_lido);
    else if (auxLin  == 1)  err = nvs_get_u16(handler_particao_nvs, V12, &dado_lido);
    else if (auxLin  == 2)  err = nvs_get_u16(handler_particao_nvs, V22, &dado_lido);
    else if (auxLin  == 3)  err = nvs_get_u16(handler_particao_nvs, V32, &dado_lido);
    else if (auxLin  == 4)  err = nvs_get_u16(handler_particao_nvs, V42, &dado_lido);
    else if (auxLin  == 5)  err = nvs_get_u16(handler_particao_nvs, V52, &dado_lido);
    else if (auxLin  == 6)  err = nvs_get_u16(handler_particao_nvs, V62, &dado_lido);
    else if (auxLin  == 7)  err = nvs_get_u16(handler_particao_nvs, V72, &dado_lido);    
  break;

  case 'L': //FAZ A LEITURA DOS MINUTOS 
    err = nvs_get_u16(handler_particao_nvs, LN, &dado_lido);
  break;


 default:
  break;
 }  
    if (err != ESP_OK)    {
        Serial.println("[ERRO] Falha ao fazer leitura do dado");         
        return 0;
    }
    else    {
  //      Serial.println("Dado lido com sucesso!");  
        nvs_close(handler_particao_nvs);   
        return dado_lido;
    }
}


/*===========================================================================================
   FUNÇÃO QUE FAZ A MONTAGEM DO BANCO DE DADOS COM OS DADOS DE TENSÃO E CORRENTE DA MÁQUINA 
==============================================================================================*/
void montaDB(uint16_t Hr, uint16_t Mn, double Im, uint Vr){
  if(linha<=7){
    for(col = 0; col < 4 ; col++){    
      if(col == 0){
      db[linha][col] = Hr;
      grava_dado_nvs(Hr, 'A');
      Serial.println();
      Serial.print("Gravou Hora - ");
      Serial.println(Hr);
      Serial.println();
      }
      else if(col == 1){
        db[linha][col] = Mn;
        grava_dado_nvs(Mn, 'B');
        Serial.print("Gravou Minuto - ");
        Serial.println(Mn);
        Serial.println();
      }
      else if(col == 2){
        db[linha][col] = Im;
        grava_dado_nvs(Im, 'C');
        Serial.print("Gravou Corrente - ");
        Serial.println(Im);
        Serial.println();
      }
      else if(col == 3) {
        db[linha][col] = Vr;
        grava_dado_nvs(Vr, 'D');
        Serial.print("Gravou Tensao - ");
        Serial.println(Vr);
        Serial.println();
      }
    }

    linha++;
    //Serial.print("LINHA --> ");
   // Serial.println(linha);
  }

  else if (linha > 7) {
    StGravaFlah = true;
    //Desloca uma linha acima os valores da matriz do DB
    Serial.println("Matriz deslocada");
    for(int s = 0 ; s < 7; s++){
      for(int cc = 0 ; cc < 4 ; cc++){
        db[s][cc] = db[s + 1][cc];
        Serial.println(db[s][cc]);     
      }
      Serial.println();
    }
    //Carrega o valir atual na ultima linha após deslocar uma liha acima os valores da matriz
    Serial.println("Ultimo Valor");
    for(int c = 0 ; c < 4 ; c++){
      if(c == 0)        db[7][c] = Hr;
      else if (c == 1)  db[7][c] = Mn;
      else if (c == 2)  db[7][c] = Im;
      else if (c == 3)  db[7][c] = Vr;
      Serial.println(db[7][c]);
    }
    Serial.println();
    Serial.print("GRAVAÇÃO NA MEMÓRIA");
    for(int i = 0 ; i < 8 ; i++){
      recLin = i;
      for(int j = 0 ; j < 4 ; j++){
        recCol = j;
        grava_dado_nvs(db[i][j],'E');
      }
    }
    StGravaFlah = false;
  }
}

/*=================================================================================================
   APRESENTA NA TELA DO DISPLAY OS VALORES DE TENSÃO E CORRENTE QUE EXCEDERAM OS KIMITES DO MOTOR  
==================================================================================================*/
void banco_de_dados(){

  u8g2.setFont(u8g2_font_timB10_tr);
  u8g2.drawLine(51,0,51,64);
  u8g2.setFont(u8g2_font_courR08_tr);

  int contLinha = 0;
  
  for(int i = 0 ; i <= 56 ; i=i+8){
    if(contLinha <= linha){
      u8g2.drawStr(1,  (i-1) , "    .  h");
      u8g2.drawStr(54, (i-1) , "     V");
      u8g2.drawStr(108, (i-1) , ".");
      u8g2.drawStr(120, (i-1) , "A");
      contLinha++;
    } 
  }

  u8g2.setFont(u8g2_font_timB10_tr);
  u8g2.drawLine(93,0,93,64);
int i1 = 0;

// --------- IMPRIMEINDO OS VALORES DA COLUNA "0" HORAS ------
    for(int a = 0 ; a <= linha ; a++){
      if(db[a][0] <= 9){
        u8g2.setFont(u8g2_font_courR08_tr);
        u8g2.setCursor(19,(i1 - 1));
        u8g2.print(db[a][0]);
        u8g2.drawStr(1, (i1-1) , "000");
        i1 = i1+8;
      }

      else if(db[a][0] > 9 && db[a][0] <= 99){
        u8g2.setFont(u8g2_font_courR08_tr);
        u8g2.setCursor(13,(i1 - 1));
        u8g2.print(db[a][0]);
        u8g2.drawStr(1, (i1-1) , "00");
        i1 = i1+8;
      }

      else if(db[a][0] > 99 && db[a][0] <= 999){
        u8g2.setFont(u8g2_font_courR08_tr);
        u8g2.setCursor(7,(i1 - 1));
        u8g2.print(db[a][0]);
        u8g2.drawStr(1, (i1-1) , "0");
        i1 = i1+8;
      }

      else if(db[a][0] > 999){
        u8g2.setFont(u8g2_font_courR08_tr);
        u8g2.setCursor(1,(i1 - 1));
        u8g2.print(db[a][0]);
        i1 = i1+8;
      }
    }

    
    // --------- IMPRIMEINDO OS VALORES DA COLUNA "1" HORAS ------
    i1 = 0;
    for(int a = 0 ; a <= linha ; a++){
      if(db[a][1] <= 9){
        u8g2.setFont(u8g2_font_courR08_tr);
        u8g2.setCursor(35,(i1 - 1));
        u8g2.print(db[a][1]);
        u8g2.drawStr(29, (i1-1) , "0");
        i1 = i1+8;
      }

      else{
        u8g2.setFont(u8g2_font_courR08_tr);
        u8g2.setCursor(28,(i1 - 1));
        u8g2.print(db[a][1]);
        i1 = i1+8;
      }
    }

    // --------- IMPRIMEINDO OS VALORES DA COLUNA "3" TENSÃO DA REDE ------
    i1 = 0;
    for(int a = 0 ; a <= linha ; a++){
      if(db[a][3] <= 9){
        u8g2.setFont(u8g2_font_courR08_tr);
        u8g2.setCursor(72,(i1 - 1));
        u8g2.print(db[a][3]);
        u8g2.drawStr(59, (i1-1) , "00");
        i1 = i1+8;
      }

      else if(db[a][3] > 9 && db[a][3] <= 99){
        u8g2.setFont(u8g2_font_courR08_tr);
        u8g2.setCursor(65,(i1 - 1));
        u8g2.print(db[a][3]);
        u8g2.drawStr(59, (i1-1) , "0");
        i1 = i1+8;
      }

      else if(db[a][3] > 99 ){
        u8g2.setFont(u8g2_font_courR08_tr);
        u8g2.setCursor(59,(i1 - 1));
        u8g2.print(db[a][3]);
        i1 = i1+8;
      }
    }

    // --------- IMPRIMEINDO OS VALORES DA COLUNA "2" CORRENTE DO MOTOR ------

    i1 = 0;
    for(int a = 0 ; a <= linha ; a++){
      if(db[a][2] <= 9){
        u8g2.setFont(u8g2_font_courR08_tr);
        u8g2.drawStr(98, (i1-1) , "0");
        u8g2.setCursor(104,(i1 - 1));
        u8g2.print(db[a][2]/10);
        u8g2.setCursor(112,(i1 - 1));
        u8g2.print(db[a][2] % 10);
        i1 = i1+8;
      }
      else{
        u8g2.setFont(u8g2_font_courR08_tr);
        u8g2.setCursor(98,(i1 - 1));
        u8g2.print(db[a][2]/10);
        u8g2.setCursor(112,(i1 - 1));
        u8g2.print(db[a][2] % 10);
        i1 = i1+8;
      }
    }
}

void analisaChamaScDB(){
  if((millis() - timeCallDB >= 4000 && millis() - timeCallDB <= 6000) && digitalRead(btDesliga)){
    habCallDB = true;
  } 
  else if ((millis() - timeHideDB >= 4000 ) && digitalRead(btDesliga))  {
    habCallDB = false;
  }
}


void  lubrifique(){
  u8g2.setFont(u8g2_font_timR08_tr);
  if (pisca == 1){
    u8g2.drawStr(47, 4, "LUBRIFIQUE A");
    u8g2.drawStr(68, 7, ".");
    u8g2.drawStr(69, 6, ".");
    u8g2.drawStr(55, 14, "MAQUINA");
  }

  if(StatusLiga || StatusRevert){
    u8g2.setFont(u8g2_font_timB10_tr);
    u8g2.drawStr(38, 30, "MOENDA");
    u8g2.drawStr(38, 45, "LIGADA");
  }
  else{
    u8g2.setFont(u8g2_font_timB10_tr);
    u8g2.drawStr(38, 30, "MOENDA");
    u8g2.drawStr(38, 45, "EM ESPERA");
  }
}

/*==================================================================================
====================================================================================
                    TRATAMENTO DAS iNTERRUPÇÕES EXTERNAS  
====================================================================================
====================================================================================*/

/*==================================================================================
                FUNÇÃO DE TRATAMENTO DA INTERRUPÇÃO DO BOTÃO LIGA  
====================================================================================*/
void IRAM_ATTR InterrupLIGA(){
  if(!StGravaFlah){
    if(!StatusRevert && !habCallDB && statusSeguranca)  {
      StatusLiga      =   true;
      statuDesl       =   false;
      StatusRevert    =   false;
      condGravaEprom  =   false;
      horimetro       =   millis();
      Serial.println("LIGA");
//      delay(10);
    }
    else   StatusLiga = false;
    

    if(StatusLiga && statusSeguranca && tempoinicializacao > 8000){
      atualizaSaidaSS(hora,     1);
    //  atualizaSaidaSS(triac,    1); 
      
      finalRamp = millis();
      fRamp = 0;
      draw(5);
    }
  }
}
/*==================================================================================
                FUNÇÃO DE TRATAMENTO DA INTERRUPÇÃO DO BOTÃO DESLIGA  
====================================================================================*/
void IRAM_ATTR InterrupDESL(){
  if(!StGravaFlah){
    if(habCallDB) timeHideDB = millis();
    Serial.println("DESL.");
//    delay(10);
    
    if(StatusLiga || StatusRevert) {
      V_baixaDetectada = false;
      V_extBaixaDetectada = false;
      I_altaDetectada = false;
      I_muitoAltaDetectada = false;
      condGravaEprom  =   true;
      delayGravaEprom = millis();
      Serial.println();
      Serial.print("Habilita Grava Flash");
      fRamp = 1;

      if(confLinha < linha && !StGravaFlah){
        grava_dado_nvs(linha , 'L');
      } 
      atualizaSaidaSS(buzzer,   0);    
    }
    //else condGravaEprom = false;
    timeCallDB = millis();
    StatusLiga      =   false;
    StatusRevert    =   false;
    statuDesl       =   true; 
    auxCont = 0;
    sTBeeb  = 0;
    minGravacaoFs = 0;

    atualizaSaidaSS(byPass,   0);
    atualizaSaidaSS(hora,     0);
    atualizaSaidaSS(antHor,   0);
    atualizaSaidaSS(triac,    0);
    atualizaSaidaSS(buzzer,   0);
  }
}
/*==================================================================================
                FUNÇÃO DE TRATAMENTO DA INTERRUPÇÃO DO BOTÃO REVERTE 
====================================================================================*/
void IRAM_ATTR InterrupREVERT(){
  if(!StGravaFlah){
    if(!StatusLiga && !habCallDB && statusSeguranca && !StGravaFlah)  {
      StatusRevert = true;
      statuDesl    = false;
      StatusLiga   = false;
      condGravaEprom  =   false;
      horimetro       =   millis();
      Serial.println("REVERT");
//      delay(10);
    }
    else StatusRevert = false;

    if(StatusRevert && statusSeguranca && tempoinicializacao > 8000 && !StGravaFlah){
      atualizaSaidaSS(antHor,   1);
      //atualizaSaidaSS(triac,    1); 
  
      finalRamp = millis();
      fRamp = 0;
      draw(6);
      }
  }
}

/*==================================================================================
           FUNÇÃO DE TRATAMENTO DA INTERRUPÇÃO DO BOTÃO DE EMERGÊNCIA   
====================================================================================*/
void IRAM_ATTR InterrupEMERG(){
  StatusEmerg       =  true;
  StatusLiga        =  false;
  StatusRevert      =  false;
  statusSeguranca   =  false;
  habCallDB         = false; 
  statuDesl         =  true;
  fRamp = 1;
  minGravacaoFs = 0;

  V_baixaDetectada = false;
  V_extBaixaDetectada = false;
  I_altaDetectada = false;
  I_muitoAltaDetectada = false;   
  auxCont = 0;
  sTBeeb  = 0;

  atualizaSaidaSS(byPass,   false);
  atualizaSaidaSS(hora,     false);
  atualizaSaidaSS(antHor,   false);
  atualizaSaidaSS(triac,    false);
}


/*==================================================================================
----------------- FUNÇÃO DE GRAVAÇÃO DE DADOS NO ARQUIVO LITTLEFS  -----------------  
====================================================================================*/
void addDataToFile(int hora, int min, int segundos, int corrente, int tensao){
    File dataFile = LittleFS.open("/DadosBracana.txt", "a");

        if (dataFile)
        {   
            Serial.println("Dado adicionado ao Arquivo");
            if (hora < 10) {
              dataFile.print("0");
              dataFile.print(hora);
            } else{dataFile.print(hora);}

            dataFile.print(":");

            if (min < 10) {
              dataFile.print("0");
              dataFile.print(min);
            }else{dataFile.print(min);}
            
            dataFile.print(":");

            if(segundos < 10){
              dataFile.print("0");
              dataFile.print(segundos);
            }else{dataFile.print(segundos);}

            dataFile.printf("- - -");
            dataFile.print(tensao);
            dataFile.printf(" - ");
            dataFile.println(corrente);
 //         delay(5);
            dataFile.close();
        }else{
            Serial.println("Erro ao Abrir o Arquivo");
        }
}

/*==================================================================================
----------------- PRÉ ANALISE PARA ARMAZENAMENTO DOS DADOS NO -----------------
                              ARQUIVO LITTLE FS    
====================================================================================*/
void analisPreArmaz(){
  int difMin = minutos - minGravacaoFs;
    if((sqrt((difMin*difMin))) >= 2)
     {
       addDataToFile(horas, minutos, seg, Irms*10, tensaoDaRede);
       numPont++;
       grava_dado_nvs(numPont,'P');
       montaDB(horas, minutos, Irms*10, tensaoDaRede);
       if (numPont >= 1000) {
        analisFsArqv();
        libMemorisFS();
        numPont = numPont/2;
        grava_dado_nvs(numPont, 'P');
       }
       minGravacaoFs = minutos;
     }
}


/*==================================================================================
------- FUNÇAO DE BACKUP DOS DADOS OU FORMATAÇÃO/ CRIAÇÃO DO ARQUIVO   ------------  
====================================================================================*/
void monBackupdeDados(){
  int comand = Serial.read();
  if (comand == 82 || comand == 114)  {printDataToSerial(); } //Se comando "R" ou "r" (Faz a leirura)

  else if (comand == 70 || comand == 102)                     //Se comando "F" ou "f" (Formata e cria novamente o arquivo caso não exista)
  {
    LittleFS.format();
    Serial.println("Arquivo Formatado..");

    //Verifica se o arquico especificado ja Existe, se não existe cria
    if(!LittleFS.exists("/DadosBracana.txt")){
      File dataFile = LittleFS.open("/DadosBracana.txt","w");
      if(!dataFile){
        Serial.print("Erro ao abrir o arquivo..");
      }else{
        Serial.println("Arquivo aberto com sucesso.");
        dataFile.println("-------------- DADOS BRACANA (HORA:MIN:SEG | TENSÃO | CORRENTE) ---------------");
      }
      dataFile.close();
    }else{
      Serial.println("Arquico exixtente..");
    }
    numPont = 0;
    grava_dado_nvs(numPont, 'P');
  }
  else if(comand == 0x64 || comand == 0x44){ //Se comando "D" ou "d" (Mostra análises de memória livre e usada no sistema de arquivo)
    analisFsArqv();
  }
  else if (comand == 0x4C || comand == 0x6C){ //Se comando "L" ou "l" (Libera memória do sistema de arquivos LittleFS)
    numPont = numPont - 1;
    grava_dado_nvs(numPont, 'P');
    libMemorisFS();
  }
  else{
    Serial.println("Comando Inválido");
  }
}

/*==================================================================================
------- FUNÇAO QUE IMPRIME TODOS OS DADOS NA SERIAL QUANDO SOLICITADO   ------------  
====================================================================================*/
void printDataToSerial(){
  File dataFile = LittleFS.open("/DadosBracana.txt","r");
   if (dataFile)
    {
        Serial.println("Arquivo aberto para leitura com Sucesso!!!");
        while (dataFile.available())
        {
            Serial.write(dataFile.read());
        }
            dataFile.close();
    } else{
      Serial.println("Erro ao ler Arquivo..");
    }
}

/*==================================================================================
------------- FUNÇAO  DE ANÁLISE DE MEMÓRIA LIVRE/USADA NO SISTEMA ----------------  
                            DE ARQUIVOS LITTLE FS  
====================================================================================*/
void analisFsArqv(){
  // Calcula o espaço ocupado pelos arquivos existentes
  size_t usedBytes = 0;

  File root = LittleFS.open("/");
  File file = root.openNextFile();
  while (file) {
    usedBytes += file.size();
    file = root.openNextFile();
  }
  root.close();
  deletLengByte = (usedBytes - 80)/2; //Bytes a serem deletados são os (bytes usados - bites do título)/2 (metade do arquivo de dados) 

  size_t totalBytes = LittleFS.totalBytes();
  size_t freeBytes = totalBytes - usedBytes;

  Serial.print("Tamanho total do sistema de arquivos: ");
  Serial.println(totalBytes);
  Serial.print("Bytes usados: ");
  Serial.println(usedBytes);
  Serial.print("Espaço livre: ");
  Serial.println(freeBytes);
  Serial.println();
  Serial.print("Numero de pontos no Arquivo: ");
  Serial.println(numPont);
}

/*==================================================================================
------------- FUNÇAO  DE LIBERAÇÃO DE MEMÓRIA DO ARQUIVO LITTLE FS  ---------------  
====================================================================================*/
void libMemorisFS(){
   // Abre o arquivo original em modo de leitura
    File originalFile = LittleFS.open("/DadosBracana.txt", "r");
    if (!originalFile) {
      Serial.println("Falha ao abrir o arquivo ORIGINAL");
      return;
    }

    // Abre o novo arquivo em modo de escrita
    File newFile = LittleFS.open("/meuArquivoNovo.txt", "w");
    if (!newFile) {
      Serial.println("Falha ao criar o novo arquivo NOVO");
      originalFile.close();
      return;
    }
    
    // Copia os dados iniciais que deseja manter para o novo arquivo
    for (size_t i = 0; i < ofSetInicial; i++) {
      newFile.write(originalFile.read());
    }

    // Pula os dados que você deseja excluir
    originalFile.seek(deletLengByte, SeekSet);

    // Copia o restante do arquivo original para o novo arquivo
    while (originalFile.available()) {
      newFile.write(originalFile.read());
    }

    originalFile.close();
    newFile.close();

    // Remove o arquivo original
    LittleFS.remove("/DadosBracana.txt");

    //Renomeia o novo arquivo com nome original 
    LittleFS.rename("/meuArquivoNovo.txt", "/DadosBracana.txt");

    Serial.println("Exclusão parcial de dados concluída com sucesso!");
}



/*==================================================================================
                        FUNÇÃO DE REINICIALIZAÇÃO DO DISPLAY    
====================================================================================*/
void reiniciaDisp(int nT){
 if(controleDisp != nT){
    u8g2.beginSimple();
    controleDisp = nT;
   // Serial.println("REINICIALIZAÇÃO DISPLAY");
  }
}