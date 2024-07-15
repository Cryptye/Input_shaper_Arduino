// Bibliotecas
#include <Arduino.h>
#include <U8g2lib.h>
#include <math.h>
#include <DueTimer.h>

// Funções
void limite(void);
void encoder_carro(void);
void encoder_pendulo(void);
void manual(void);
void zera(void);
void shaper(void);
void aplica_controle(void);
void calc_controle(void);
void draw(void);

// Definições
#define encoder0PinA 36
#define encoder0PinB 38
#define encoder1PinA 30
#define encoder1PinB 32
#define btn_sel 31
#define btn_dir 29
#define btn_esq 33
#define fim_curso 35
#define INA 9
#define INB 10
#define EN 8
#define PULSE 11

// Variáveis Gerais
boolean sel = 0;
boolean est_sel = 0;
boolean est_dir = 0;
boolean est_esq = 0;
boolean est_fc = 0;
uint8_t estado = 0;
uint8_t p_carro = 0;  // para o desenho
uint8_t last_dir = 1;
uint8_t last_esq = 1;
uint8_t last_sel = 1;
volatile int flaghome = 0;
volatile int encoder0Pos = 0;
volatile int encoder1Pos = 0;
const float pi = 3.14159267;
const float kp = 0.08;
float ang_pen = 0;    // para o desenho
int flagbtn = 0;
int x_dec = 0;
int y_dec = 0;
int pos_carro = 0;
int pos_pen = 0;
int controle = 0;
int ref_pos = 9950;
int erro_pos = 0;
int control_pos = 0;
unsigned long startMillis;
unsigned long currentMillis;

// Variáveis Input Shaper
int WN = 10; float Z = 0.1; float e = 2.71828;                // -> Conferir se usa Hz ou rad/s
float K = pow(e, -Z*pi/sqrt(1-sq(Z)));
float T = pi/(WN*sqrt(1-sq(Z)));
float denominador = 1 + 2*K + sq(K);
int final_ref_pos = 12500;  // Levar o carrinho até o final da trilha, por ex. 13300
int I1 = (1/denominador)*final_ref_pos;  // (Distância 1 em fração * 19000)
int I2 = I1+(2*K/denominador)*final_ref_pos;
// int I3 = (1-(sq(K)/denominador))*final_ref_pos;
int I3 = I2+(sq(K)/denominador)*final_ref_pos;
int impulsos[3] = {I1, I2, I3};
int T1 = 0*1000;  // (Tempo da aplicação do degrau 1) 
int T2 = T*1000;
int T3 = 2*T*1000;
int tempos[3] = {T1, T2, T3};
int stage_flg = 1;


// Classes e Objetos
// -------------------------------------------------------------------------------------------------------------------------------------

U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, 27, 25, 23, U8X8_PIN_NONE);

class MotorCC {
private:
  uint8_t IN_A;
  uint8_t IN_B;
  uint8_t P_ULSE;

public:
  MotorCC(uint8_t IN_A, uint8_t IN_B, uint8_t P_ULSE) {
    this->IN_A = IN_A;
    this->IN_B = IN_B;
    this->P_ULSE = P_ULSE;
    init();
  }

  void init() {
    pinMode(IN_A, OUTPUT);
    pinMode(IN_B, OUTPUT);
    pinMode(P_ULSE, OUTPUT);
    frenagem();
  }

  void forward(int controle) {
    digitalWrite(IN_A, LOW);
    digitalWrite(IN_B, HIGH);
    analogWrite(P_ULSE, controle);
  }

  void backward(int controle) {
    digitalWrite(IN_A, HIGH);
    digitalWrite(IN_B, LOW);
    analogWrite(P_ULSE, -controle);
  }

  void frenagem() {
    digitalWrite(IN_A, HIGH);
    digitalWrite(IN_B, HIGH);
    analogWrite(P_ULSE, 0);
  }
};

class Desenhador {
public:
  Desenhador(void) {
    u8g2_prepare();
  }

  void u8g2_prepare(void) {
    u8g2.setFont(u8g2_font_7x14B_tr);
    u8g2.setFontRefHeightExtendedText();
    u8g2.setDrawColor(1);
    u8g2.setFontPosTop();
    u8g2.setFontDirection(0);
  }

  void tela_inicial(void) {
    u8g2.drawStr(36, 10, "PENDULO");
    u8g2.drawStr(28, 25, "INVERTIDO");
    u8g2.drawStr(14, 50, "Prof.: Michael");
    u8g2.setFont(u8g2_font_5x7_tf);
  }

  void tela_main(void) {
    p_carro = map(pos_carro, 0, 850, 0, 83);
    x_dec = 23 * sin(ang_pen);
    y_dec = 23 * cos(ang_pen);
    // x_dec=0;  y_dec=23;
    u8g2.setDrawColor(1);
    u8g2.drawHLine(4, 13, 80);             // Desenha Guia Lineas
    u8g2.drawBox(14 + p_carro, 1, 18, 8);  // Desenha Carro
    u8g2.drawCircle(18 + p_carro, 11, 2);  // Desenha Rodas
    u8g2.drawCircle(28 + p_carro, 11, 2);
    //                  ponto inferior      ponto superior
    // u8g2.drawLine(23+p_carro, 25, 23+p_carro+x_dec, 25-y_dec); // Desenha Pendulo - Original
    //                  ponto superior        ponto inferior
    u8g2.drawLine(23 + p_carro, 5, 23 + p_carro - x_dec, 5 + y_dec);  // Desenha Pendulo 23+p_carro-x_dec, 25-y_dec
    u8g2.drawCircle(23 + p_carro - x_dec, 7 + y_dec, 2);
    // Escreve Valor de Posicao e Angulo
    u8g2.drawStr(96, 15, "A:");
    u8g2.setCursor(108, 15);
    u8g2.print(String(pos_pen));
    u8g2.drawStr(96, 25, "P:");
    u8g2.setCursor(108, 25);
    u8g2.print(String(pos_carro));
    
    if (encoder0Pos > 9999){
      u8g2.drawStr(90, 35, "E:");
      u8g2.setCursor(102, 35);
      u8g2.print(String(encoder0Pos));
    }
    else{
      u8g2.drawStr(96, 35, "E:");
      u8g2.setCursor(108, 35);
      u8g2.print(String(encoder0Pos));
    }
    
    // Desenha e Seleciona Botões
    u8g2.drawRFrame(1, 52, 26, 11, 3);   // Zera
    u8g2.drawRFrame(30, 52, 26, 11, 3);  // Ini.
    u8g2.drawRFrame(59, 52, 26, 11, 3);  // Manual
    u8g2.drawRFrame(88, 52, 39, 11, 3);  // Shaper
    u8g2.drawStr(4, 53, "Zera");
    u8g2.drawStr(33, 53, "Ini.");
    u8g2.drawStr(63, 53, "Man.");
    u8g2.drawStr(93, 53, "Shaper");
    switch (flagbtn) {
      case 0:
        if (sel == 0) {
          u8g2.drawRFrame(0, 51, 28, 13, 3);
        } else {
          u8g2.drawRBox(0, 51, 28, 13, 3);
          u8g2.setDrawColor(0);
          u8g2.drawStr(4, 53, "Zera");
        }
        break;
      case 1:
        if (sel == 0) {
          u8g2.drawRFrame(29, 51, 28, 13, 3);
        } else {
          u8g2.drawRBox(29, 51, 28, 13, 3);
          u8g2.setDrawColor(0);
          u8g2.drawStr(33, 53, "Ini.");
        }
        break;
      case 2:
        if (sel == 0) {
          u8g2.drawRFrame(58, 51, 28, 13, 3);
        } else {
          u8g2.drawRBox(58, 51, 28, 13, 3);
          u8g2.setDrawColor(0);
          u8g2.drawStr(63, 53, "Man.");
        }
        break;
      case 3:
        if (sel == 0) {
          u8g2.drawRFrame(87, 51, 41, 13, 3);
        } else {
          u8g2.drawRBox(87, 51, 41, 13, 3);
          u8g2.setDrawColor(0);
          u8g2.drawStr(93, 53, "Shaper");
        }
        break;
    }
  }
};

MotorCC motor(INA, INB, PULSE);
Desenhador desenho;  // Aparentemente tem q tirar os () quando a classe é 'void' nos parâmetros

// -------------------------------------------------------------------------------------------------------------------------------------

// Função Limite -> Chamada quando a chave de fim de curso é pressionada
void limite() {
  if (flaghome == 0) { // vai sempre entrar na primeira iteração do programa, pois flaghome é 0 inicialmente
    motor.frenagem();  // freia o motor, pois a chave de fim de curso foi pressionada
    flaghome = 1;      // prepara a variável para voltar com o carrinho para trás (variável acessada pela função ZERA)
  }
}

// Função Encoder 0 -> Chamada toda vez que o pino do encoder 0 muda de valor
void encoder_carro() {
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos--;
  } else {
    encoder0Pos++;
  }
}

// Função Encoder 1 -> Chamada toda vez que o pino do encoder 1 muda de valor
void encoder_pendulo() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }

  if ((encoder1Pos >= 2000) || (encoder1Pos <= -2000)) { encoder1Pos = 0; }
}

// Função Manual
void manual(void) {
  ref_pos = encoder0Pos;
  if (est_dir == 0) { ref_pos = encoder0Pos + 1000; }
  if (est_esq == 0) { ref_pos = encoder0Pos - 1000; }
  // ref_pos = constrain(ref_pos, 700, 19200);
  ref_pos = constrain(ref_pos, 100, 12500);
}

// Função Homing
void zera(void) {
  switch (flaghome) {
    case 0:
      controle = 150;
      aplica_controle();
      break;
    case 1:
      encoder0Pos = 13000;  // Seta encoder para posição máxima
      ref_pos = 100;
      Timer4.start();
      flaghome = 2;
      break;
    case 2:
      if (encoder0Pos <= 100) { // define qual o ponto de parada do motor na volta
        motor.frenagem();
        sel = 0;
        flaghome = 0;
        Timer4.stop();
      }  
      break;
  }
}

// Função Shaper -> NÃO UTILIZADA
void shaper(void) {

  ref_pos = impulsos[0];
  // Criar aqui lógica para calcular os steps de distância modelados a partir de ref_pos
  // ----------------------------------------------------------------------------------
  // Exemplo de ZVD:
  //
  
  //
  
  
  // float A1 = (1/denominador);  // (Distância 1 em fração * 19000)
  // float A2 = (2*K/denominador);
  // float A3 = (sq(K)/denominador);

  // Serial.println(A1);
  // Serial.println(A2);
  // Serial.println(A3);
  // Serial.println(K);
  // Serial.println('-');

  //(Garantir que a lógica permita inserir os 3 pulsos em sequência)
  //
  // (Usar alguma função de timer (SEM TRAVAR EXECUÇÃO) para contar os segundos entre a aplicação dos pulsos)

  // Timer.Start();
  // while (not in ref_pos) {
  //    while(less than A1) {wait}
  //    delay(T2)
  //    while(less than A2) {wait}
  //    delay(T3)    
  //                        }

  // ref_pos = A1;
  // Timer4.start();
  // // if(encoder0Pos+580 < ref_pos) {Serial.println(ref_pos-encoder0Pos);}
  // if(encoder0Pos+580 < ref_pos) {
  //   TImer4.stop()
  //   motor.frenagem()
  //   delay(T2);
  //   ref_pos = A2
  //   Timer4.start()
  //   }
  
  
}

// Função Aplica Controle
void aplica_controle(void) {
  if (controle < 0) { motor.backward(controle); }
  if (controle > 0) { motor.forward(controle); }
  // if (controle < 0) { motor.backward(-150); }
  // if (controle > 0) { motor.forward(150); }
}

// Função Calcula Controle
void calc_controle(void) {
  // ------------------------------------------------------------------------------------- Segunda parte
  // Fazer o cálculo para o A1, A2 e A3, nos tempos devidos
  erro_pos = ref_pos - encoder0Pos;  // posição de ref = posição inicial (máxima)
  control_pos = kp * erro_pos;       // calculo * 0.08

  if (control_pos >= 10) {  // control_pos>=255
    control_pos = 150;
  }  // control_pos=255;

  if (control_pos <= -10) {
    control_pos = -150;
  }

  controle = control_pos;  // aplica o controle positivo se cálculo anterior for > que 0 e negativo caso contrário
  aplica_controle();       // Aplicar o controle em loop até que as 3 posições sejam alcançadas
}

// Função que chama as telas do programa
void draw(void) {
  switch (estado) {
    case 0: desenho.tela_inicial(); break;
    case 1: desenho.tela_main(); break;
  }
}

void setup(void) {

  Serial.begin(9600);
  Serial.println("start");
  // Serial.println(T1); Serial.println(T2); Serial.println(T3);
  for (int i=0; i<3; i++){
    Serial.println(tempos[i]); 
    Serial.println(impulsos[i]); 
  }
  
  // Configura Pinos Botoeiras e Fim de Curso
  pinMode(btn_sel, INPUT_PULLUP);
  pinMode(btn_dir, INPUT_PULLUP);
  pinMode(btn_esq, INPUT_PULLUP);
  pinMode(fim_curso, INPUT_PULLUP);
  // Configura Pinos Motor e Seta Condição Inicial
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(PULSE, OUTPUT);
  analogWrite(PULSE, 0);
  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);
  digitalWrite(EN, HIGH);
  // Seta Pinos do Encoder e Zera Valor
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);
  pinMode(encoder1PinA, INPUT);
  digitalWrite(encoder1PinA, HIGH);
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinB, HIGH);
  // Seta Interrupcao de Pinos
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), encoder_carro, CHANGE);    // encoder pin on interrupt
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder_pendulo, CHANGE);  // encoder pin on interrupt
  attachInterrupt(digitalPinToInterrupt(fim_curso), limite, FALLING);             // encoder pin on interrupt

  // Inicializa Display e Seta configurações iniciais
  u8g2.begin();  //u8g2_prepare();
  // Desenha Tela Inicial
  u8g2.clearBuffer();
  desenho.tela_inicial();
  u8g2.sendBuffer();
  estado++;
  delay(1000);
  //Zera Encoders
  encoder0Pos = 0;
  encoder1Pos = 0;

  last_dir = digitalRead(est_dir);

  last_esq = digitalRead(est_esq);
  last_sel = digitalRead(est_sel);

  // Configura Interrupção de Controle
  // Timer4.attachInterrupt(controler.calc_controle(void));
  Timer4.attachInterrupt(calc_controle);
  Timer4.setFrequency(20000);  // Interrupção a cada 20ms
}

void loop(void) {

  est_sel = digitalRead(btn_sel);  // botão da direita -> estado não pressionado = 1
  est_dir = digitalRead(btn_dir);  // botão do meio -> estado não pressionado = 1
  est_esq = digitalRead(btn_esq);  // botão da esquerda -> estado não pressionado = 1

  // Verifica e Seleciona (Zera / Start / Manual)
  if (((est_sel == 0) & (last_sel == 1)) || (Serial.read() == 'd')) { sel = !sel; }  // muda de 0 para 1 na primeira vez que est_sel é apertado
  last_sel = digitalRead(btn_sel);

  switch (sel) {
    case 0:  // Nao Selecionado
      flaghome = 0;
      motor.frenagem();
      Timer4.stop();
      // if (est_dir == 0 & last_dir == 1){
      if (((est_dir == 0) & (last_dir == 1)) || (Serial.read() == 's')) {
        flagbtn++;
        if (flagbtn > 3) { flagbtn = 0; }
      }
      last_dir = digitalRead(btn_dir);

      if (((est_esq == 0) & (last_esq == 1)) || (Serial.read() == 'a')) {
        flagbtn--;
        last_esq = digitalRead(btn_esq);
        if (flagbtn < 0) { flagbtn = 3; }
      }
      last_esq = digitalRead(btn_esq);

      break;
    case 1:  // Selecionado
      switch (flagbtn) {
        case 0:  // Executa Homing  ->  Aciona movimentação sem controle algum, até bater na chave de fim de curso
          // Serial.print("flaghome: "), Serial.print(flaghome), Serial.print('\n');
          zera();
          break;
        case 1:             // Inicia Controle
          ref_pos = 12500;  // Levar o carrinho até o final da trilha, por ex. 19000
          Timer4.start();
          // Serial.println(control_pos, DEC);  // DEC é parâmetro para usar Decimal
          break;
        case 2:  // Modo Manual
          manual();
          Timer4.start();
          //  Serial.println(ref_pos, DEC);
          break;
        case 3:  // Modo Shaper
          ref_pos = impulsos[2];
          Timer4.start();

          if (encoder0Pos > I1 && stage_flg == 1){
            Timer4.stop();
            motor.frenagem();
            if (stage_flg == 1){startMillis = millis(); stage_flg += 1;}
            else {currentMillis = millis(); Serial.println(currentMillis - startMillis); if ((currentMillis - startMillis) >= T2+2000){Timer4.start();}}
            }
          
          if (encoder0Pos > I2 && stage_flg == 2){
            Timer4.stop();
            motor.frenagem();
            if (stage_flg == 2){startMillis = millis(); stage_flg += 1;}
            else {currentMillis = millis(); if ((currentMillis - startMillis) >= T3+2000){Timer4.start();}}
          }


          // Serial.println(encoder0Pos, DEC);
          // Serial.println(ref_pos, DEC);
          // sel = 0;
      }
      break;
  }
  // sel = 0;

  // Faz Mapeamento da leitura dos Encoders
  // pos_carro= map(encoder0Pos, 0, 19900, 0, 850);
  // pos_pen= map(encoder1Pos, 0, -2000, 0, 360);
  // ang_pen= (pos_pen*pi)/180;
  pos_carro = map(encoder0Pos, 0, 13000, 0, 535);
  pos_pen = map(encoder1Pos, 0, -2000, 0, 360);
  ang_pen = (pos_pen * pi) / 180;

  // Atualiza Tela
  u8g2.clearBuffer();
  draw();
  u8g2.sendBuffer();
  delay(20);
}