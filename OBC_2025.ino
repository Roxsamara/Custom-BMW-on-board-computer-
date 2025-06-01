/*почему то не получилось заюзать D-пины на прерывания, пока хз.
почему то не работает адекватно на А1, но работает на А2 и А3 06,02,2023*/

#include "GyverButton.h"
GButton myButt(8, HIGH_PULL, NORM_OPEN);
#include "GyverFilters.h"
GFilterRA analog7;  
#include <iarduino_VCC.h>
volatile boolean running = false;
volatile int check_3 = 0;
volatile int y = 0;
volatile byte pin_in = 1;
int hs = LOW;
int hs_prev = HIGH;
int a0;
int a00;
int a1;
int a2;
char msg;
int i;
int res;
int counter = 0;
int64_t v;
float outputValue = 0;
int n1, n2, n3, n4;
unsigned long n;
volatile int64_t speed_last_pulse = 0;
volatile int64_t speed_prev_pulse = 0;
volatile int64_t a = 0;
double current_speed = 0;
int Vold = 0;
#pragma message "XUY"
#define A1_READ bitRead(PINC, 1)
int temper = A0;
long v_value = 0;
int v_count = 0;
unsigned long timing;  // Переменная для хранения точки отсчета


int calc_message_checksum(char* msg)  // считаем контрольную сумму
{
  res = 1;
  for (i = 0; i < 16; i++)         //если i ровно нулю, если меньше чем 16 то i+1
    res = (res + (255 & *msg++));  //msg=msg+1
  return (res & 255);
}

void vivod(char msg[]) {
  digitalWrite(7, LOW);
  Serial.write(msg, 16);
  Serial.write(calc_message_checksum(msg));
  Serial.flush();
  digitalWrite(7, HIGH);  // отпускаем  LAC
}

void sped() {
  if (hs == HIGH && hs != hs_prev) {
    char message[17];
    sprintf(message, "  CKOPOCTb %3d  ", (int)(floor(v)), (int)(floor((v - floor(v)) * 100)));
    vivod(message);
    delay(2);
    check_speed();
  }
  hs_prev = hs;
  hs = digitalRead(6);
  delay(5);
}
void taho() {
  if (hs == HIGH && hs != hs_prev) {
    char message[17];
    sprintf(message, " TAXOMETP %3d   ", (int)(floor(v)), (int)(floor((v - floor(v)) * 100)));
    vivod(message);
    delay(2);
    check_speed();
    v = v * 25.16;
  }
  hs_prev = hs;
  hs = digitalRead(6);
  delay(5);
}
void check_speed() {
  unsigned long now;
  unsigned long diff;
  diff = speed_last_pulse - speed_prev_pulse;
  current_speed = 1000000.0 / diff;
  current_speed = current_speed / 1.25;
  if ((micros() - speed_last_pulse) > 500000) v = 0;
  else v = current_speed;
}

ISR(PCINT1_vect) {
  check_3 = 0;
  for (y = 0; y <= 100; y++) {
    check_3 = (1 - bitRead(PINC, pin_in)) + check_3;
  }
  if (check_3 == 101) return;

  running = !running;
  digitalWrite(13, running);
  speed_prev_pulse = speed_last_pulse;
  speed_last_pulse = micros();
}

void temp() {
  if (hs == HIGH && hs != hs_prev) {
    char message[17];
    a0 = analogRead(A0);
   // a00 = analogRead(A0);
   a1 = ((-32.5 + (1024-a0) / float(6.17)));
   a1 =a1 -6;
    sprintf(message, "TEMnEPATYPA %dc ", a1);
    vivod(message);
    delay(2);
  }
  hs_prev = hs;
  hs = digitalRead(6);
  delay(5);
}

void volt() {
  if (hs == HIGH && hs != hs_prev) {
    char message[17];
    v_count = 0;
    v_value = 0;
    do {
      v_value = analogRead(A7) + v_value;
      v_count++;
    } while (v_count != 100);
    //  v_value = (v_value / 100);
    n = v_value;
    //v_value = (v_value / 100);
    n = (n * analogRead_VCC() * 3.0) / 100.0;
    //
    //n=v_value/100;
    n = n * 1.045218680504077;
    n = analog7.filteredTime(n);
    n1 = n / 1000;
    n2 = n / 100 % 10;
    n3 = n / 10 % 10;
    n4 = n % 10 % 10;
    sprintf(message, " Voltage %d,%d%d   ", n1, n2, n3);
    vivod(message);
    delay(2);
  }
  hs_prev = hs;
  hs = digitalRead(6);
  delay(5);
}
void volt_ard() {
  if (hs == HIGH && hs != hs_prev) {
    char message[17];

    float VCC_now = analogRead_VCC();

    n = VCC_now * 1000;
    n1 = n / 1000;
    n2 = n / 100 % 10;
    n3 = n / 10 % 10;
    n4 = n % 10 % 10;
    sprintf(message, "V arduino %d,%d%dv ", n1, n2, n3);
    vivod(message);
    delay(2);
  }
  hs_prev = hs;
  hs = digitalRead(6);
  delay(5);
}
void while_nop() {
  while (1) {
    myButt.tick();
    delay(2);
    if (myButt.isDouble()) {
      return;
    }
  }
}
void while_sped() {
  while (1) {
    sped();
    myButt.tick();
    delay(2);
    if (myButt.isDouble()) {
      while_nop();
    }
    if (myButt.isSingle()) {
      return;
    }
  }
}
void while_taho() {
  while (1) {
    taho();
    myButt.tick();
    delay(2);
    if (myButt.isDouble()) {
      while_nop();
    }
    if (myButt.isSingle()) {
      return;
    }
  }
}
void while_temp() {
  while (1) {
    temp();
    myButt.tick();
    delay(2);
    if (myButt.isDouble()) {
      while_nop();
    }
    if (myButt.isSingle()) {
      return;
    }
  }
}
void while_volt() {
  while (1) {
    volt();
    myButt.tick();
    delay(2);
    if (myButt.isDouble()) {
      while_nop();
    }
    if (myButt.isSingle()) {
      return;
    }
  }
}
void while_volt_ard() {
  while (1) {
    volt_ard();
    myButt.tick();
    delay(2);
    if (myButt.isDouble()) {
      while_nop();
    }
    if (myButt.isSingle()) {
      return;
    }
  }
}
void setup() {

  Serial.begin(9600);
  UCSR0C = (UCSR0C & ~_BV(UPM00) | _BV(UPM01));
  pinMode(A0, INPUT);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  pinMode(A6, INPUT_PULLUP);
  pinMode(A7, INPUT);
  pinMode(3, INPUT_PULLUP);
  pinMode(6, INPUT);
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(7, HIGH);
  //attachPCINT(A1);
  myButt.setClickTimeout(150);
  // установка коэффициента фильтрации (0.0... 1.0). Чем меньше, тем плавнее фильтр
  analog7.setCoef(0.3);

  // установка шага фильтрации (мс). Чем меньше, тем резче фильтр
  analog7.setStep(10);
  // while_nop();
}

void loop() {

  attachPCINT(A3);
  pin_in = 3;
  while_sped();
  dettachPCINT(A3);

  attachPCINT(A2);
  pin_in = 2;
  while_taho();
  dettachPCINT(A2);

  while_temp();
  while_volt();
 // while_volt_ard();
}
uint8_t attachPCINT(uint8_t pin) {  /////////////работает только на смену(???)
  if (pin < 8) {                    // D0-D7 (PCINT2)
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << pin);
    return 2;
  } else if (pin > 13) {  //A0-A5 (PCINT1)
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << pin - 14);
    return 1;
  } else {  // D8-D13 (PCINT0)
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << pin - 8);
    return 0;
  }
}
uint8_t dettachPCINT(uint8_t pin) {  /////////////работает только на смену(???)
  if (pin < 8) {                     // D0-D7 (PCINT2)
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << pin);
    return 2;
  } else if (pin > 13) {  //A0-A5 (PCINT1)
    PCICR &= ~(1 << PCIE1);
    PCMSK1 &= ~(1 << pin - 14);
    return 1;
  } else {  // D8-D13 (PCINT0)
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << pin - 8);
    return 0;
  }
}