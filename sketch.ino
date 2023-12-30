#include <Wire.h> // i2c library

// gravity
#define g 9.80665

// full scale range
#define G 2 // options: 2, 4, 8, 16

#define PERIOD 2 // interrupt period (seconds)

// resolution
#define RES 65536 // 16 bit resolution: 2^16 = 65536

// i2c address of mpu6050
#define MPU_ADD 0x68

// registers addresses
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B // MSB
#define ACCEL_XOUT_L 0x3C // LSB
#define ACCEL_YOUT_H 0x3D // MSB
#define ACCEL_YOUT_L 0x3E // LSB
#define ACCEL_ZOUT_H 0x3F // MSB
#define ACCEL_ZOUT_L 0x40 // LSB

// acceleration variables
byte h=0, l=0;
float acc_g=0, acc_ms2=0;

// update period
int cnt_timer0 = 0;
bool atualiza = false; // flag

// serial monitor
String msg = "";


// * * * * * I2C READ AND WRITE * * * * *

byte read_reg(int slave_add, int reg_add) {
  byte reg;
  Wire.beginTransmission(slave_add);
  Wire.write(reg_add); // which register i want to read from
  Wire.endTransmission();

  Wire.requestFrom(slave_add, 1); // reads 1 byte from reg_add
  while(Wire.available()) {
        reg = Wire.read();
  }
  
  return reg;
}

void write_reg(int slave_add, int reg_add, byte data) {
  Wire.beginTransmission(slave_add);
  Wire.write(reg_add);
  Wire.write(data);
  Wire.endTransmission();
}

// * * * * * TIMER FUNCTIONS * * * * *

void config_Timer0(){
  //clk = 16 MHz
  //prescaler = 256
  //faixa = 125 (contagem de 0 a OCR0A = 124)
  //interrupt period: (prescaler/clk)*faixa = (256/16e6)*(124+1) = 0.002s
  
  //TCCR0A – Timer/Counter Control Register A
  //COM0A1 COM0A0 COM0B1 COM0B0 – – WGM01 WGM00
  //0      0      0      0          1     0
  TCCR0A = 0x02; //clear OC0A on compare match

  //OCR0A – Output Compare Register A
  OCR0A = 124; //OCR0A is continuously compared with the counter value (TCNT0)

  //TIMSK0 – Timer/Counter Interrupt Mask Register
  // – – – – – OCIE0B OCIE0A TOIE0
  // – – – – – 0      1      0
  TIMSK0 = 0x02; //Timer/Counter0 compare match A interrupt is enabled
  
  //TCCR0B – Timer/Counter Control Register B
  //FOC0A FOC0B – – WGM02 CS02 CS01 CS0
  //0     0         0     1    0    0
  TCCR0B = 0x04; //clk/256
}

//rotina de servico de interrupcao do temporizador
ISR(TIMER0_COMPA_vect) {
  cnt_timer0++; //cnt_timer0 incremented every 2ms
  
  if (cnt_timer0 >= PERIOD*500){
    atualiza = true;
    cnt_timer0=0;
  }
}

// * * * * * MPU FUNCTIONS * * * * *

void config_fs_range (int fs) { // full scale range configuration
  byte config;

  config = read_reg(MPU_ADD, ACCEL_CONFIG);

  if (fs==16) {
    config = config | 0x18;
    write_reg(MPU_ADD, ACCEL_CONFIG, config);
    Serial.println("ACCEL CONFIG: 16g");
  }
  else if (fs==8) {
    config = config | 0x10;
    config = config & 0xF7;
    write_reg(MPU_ADD, ACCEL_CONFIG, config);
    Serial.println("ACCEL CONFIG: 8g");
  }
  else if (fs==4) {
    config = config | 0x08;
    config = config & 0xEF;
    write_reg(MPU_ADD, ACCEL_CONFIG, config);
    Serial.println("ACCEL CONFIG: 4g");
  }
  else {
    config = config & 0xE7;
    write_reg(MPU_ADD, ACCEL_CONFIG, config);
    Serial.println("ACCEL CONFIG: 2g (default)");
  }
}

uint16_t mergeHnL(byte H, byte L) { //HIGH: MSB, LOW: LSB
  uint16_t value;
  
  value = H;
  value = value << 8; // moves  HIGH to MSB
  value = value | L; // loads LSB with LOW
  
  return value;
}

float to_float(uint16_t bin) {
  uint16_t comp;
  float comp_f;
  if ((bin/(RES/2)>=1) { //identifies negative number (checks if MSB is 1)
    comp = ~bin;
    comp = comp+1;
    comp_f = float(comp);
    comp_f = comp_f*(-1);
  }
  else {
    comp_f = float(bin);
  }
  return comp_f;
}

float conv(byte H, byte L) {
  uint16_t v_bits;
  float v_float, acc;
  v_bits = mergeHnL(H,L);
  v_float = to_float(v_bits);

  acc = v_float*2*G/RES;
  return acc;
}

float g_to_ms2(float acc) {
  return acc*g;
}

// * * * * * SETUP * * * * *

void setup() {
  // i2c
  Wire.begin();

  // serial monitor
  Serial.begin(9600);

  // ACCEL CONFIG
  config_fs_range(G);

  // timer
  cli();
  config_Timer0();
  sei();
}

void loop() {
  if (atualiza) {

    msg="X: ";
    h = read_reg(MPU_ADD,ACCEL_XOUT_H);
    l = read_reg(MPU_ADD,ACCEL_XOUT_L);
    acc_g = conv(h,l);
    acc_ms2 = g_to_ms2(acc_g);
    msg.concat(String(acc_ms2));
    msg.concat(" m/s^2");
    Serial.println(msg);

    msg="Y: ";
    h = read_reg(MPU_ADD,ACCEL_YOUT_H);
    l = read_reg(MPU_ADD,ACCEL_YOUT_L);
    acc_g = conv(h,l);
    acc_ms2 = g_to_ms2(acc_g);
    msg.concat(String(acc_ms2));
    msg.concat(" m/s^2");
    Serial.println(msg);

    msg="Z: ";
    h = read_reg(MPU_ADD,ACCEL_ZOUT_H);
    l = read_reg(MPU_ADD,ACCEL_ZOUT_L);
    acc_g = conv(h,l);
    acc_ms2 = g_to_ms2(acc_g);
    msg.concat(String(acc_ms2));
    msg.concat(" m/s^2");
    Serial.println(msg);

    Serial.println("- - - - -");
  
    atualiza=false;
  }
}
