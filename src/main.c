/*
00000001	00000000	00000001	горит желтый
00000010	01111100	01111110	красное число 124
00000100	00110100	00110000	зеленое число 52
00001000	00000000	00001000	мигает желтый
*/
#include <Arduino.h>

#define yl_bit 0
#define rl_bit 1
#define gl_bit 2
#define e_state_bit 3

#define dir485 PC4
#define yl PA3
#define rl PD3
#define gl PD4

#define shcp PC5
#define stcp PC6
#define ds PC7

#define E_State_Period 1000
uint8_t E_State = 0;
uint32_t E_State_timer = 0;

uint16_t Displ_data_raw = 0;
uint8_t Displ_number = 0;

#define RData_Timeout 50
#define Connect_Timeout 500

uint32_t RDT_timer = 0;
uint32_t ConnT_timer = 0;

uint8_t RData[3] = {0, 0, 0};

uint8_t num_d_db[10] = {0b01111110, 0b00110000, 0b01101101, 0b01111001, 0b00110011, 0b01011011, 0b01011111, 0b01110000, 0b01111111, 0b01111011};
uint8_t num_e_db[10] = {0b01111110, 0b00000110, 0b01011011, 0b01001111, 0b00100111, 0b01101101, 0b01111101, 0b01000110, 0b01111111, 0b01101111};

uint8_t color_l = 0;

void displ_data_update()
{
  Displ_data_raw = 0;
  if (Displ_number >= 100)
  {
    Displ_data_raw = 1; //рисуем сотни
  }
  Displ_data_raw |= (num_e_db[Displ_number % 10] << 9); //рисуем единицы
  // Displ_data_raw |= (num_d_db[Displ_number % 100 / 10] << 1) | (num_e_db[Displ_number % 10] << 9);
  if (Displ_number > 9) //для вырезания нуля
  {
    Displ_data_raw |= (num_d_db[Displ_number % 100 / 10] << 1); //если больше 9, то рисуем циру десятка числа
  }
}

// the setup routine runs once when you press reset:
void setup()
{
  pinMode(dir485, OUTPUT);

  pinMode(yl, OUTPUT);
  digitalWrite(yl, LOW);
  pinMode(rl, OUTPUT);
  digitalWrite(rl, LOW);
  pinMode(gl, OUTPUT);
  digitalWrite(gl, LOW);

  pinMode(shcp, OUTPUT);
  pinMode(stcp, OUTPUT);
  pinMode(ds, OUTPUT);
  digitalWrite(stcp, LOW);
  digitalWrite(shcp, LOW);
  digitalWrite(ds, LOW);

  Serial_begin(19200);

  // digitalWrite(dir485, HIGH);
  // delay(100);
  // Serial_println_s("YLed started!");
  delay(100);
  digitalWrite(dir485, LOW);
}
void Refr_displ()
{
  digitalWrite(yl, LOW);
  digitalWrite(rl, LOW);
  digitalWrite(gl, LOW);
  //отключили, обновляем
  digitalWrite(stcp, LOW);
  for (uint8_t dbit = 0; dbit < 16; dbit++)
  {
    digitalWrite(shcp, LOW);
    if ((1 << dbit) & Displ_data_raw)
    {
      digitalWrite(ds, HIGH);
    }
    else
    {
      digitalWrite(ds, LOW);
    }
    digitalWrite(shcp, HIGH);
  }
  digitalWrite(stcp, HIGH);
  //теперь включаем цвет
  if (color_l & (1 << yl_bit))
  {
    digitalWrite(yl, HIGH);
  }
  if (color_l & (1 << rl_bit))
  {
    digitalWrite(rl, HIGH);
  }
  if (color_l & (1 << gl_bit))
  {
    digitalWrite(gl, HIGH);
  }
  //включили
}
// the loop routine runs over and over again forever:
void loop()
{
  if (E_State && (millis() - E_State_timer >= E_State_Period))
  {
    digitalWrite(rl, LOW);
    digitalWrite(gl, LOW);
    if (digitalRead(yl))
    {
      digitalWrite(yl, LOW);
    }
    else
    {
      digitalWrite(yl, HIGH);
    }
    E_State_timer = millis();
  }
  static uint8_t RD_id = 0;
  if (millis() - ConnT_timer >= Connect_Timeout)
  {
    E_State = 0xFF;
  }
  if (millis() - RDT_timer >= RData_Timeout)
  {
    HardwareSerial_flush();
    RD_id = 0;
  }
  if ((Serial_available() > 0) && (RD_id < 3))
  {
    if (!RD_id)
    {
      RDT_timer = millis();
    }
    RData[RD_id] = Serial_read();
    RD_id++;
  }
  if (RD_id == 3)
  {
    if ((RData[0] ^ RData[1]) == RData[2])
    {
      color_l = RData[0] & 0b111;
      /*
      if (RData[0] & (1 << yl_bit))
      {
        digitalWrite(yl, HIGH);
      }
      else
      {
        digitalWrite(yl, LOW);
      }
      if (RData[0] & (1 << rl_bit))
      {
        digitalWrite(rl, HIGH);
      }
      else
      {
        digitalWrite(rl, LOW);
      }
      if (RData[0] & (1 << gl_bit))
      {
        digitalWrite(gl, HIGH);
      }
      else
      {
        digitalWrite(gl, LOW);
      }
      */
      if (RData[0] & (1 << e_state_bit))
      {
        E_State = 0xFF;
      }
      else
      {
        E_State = 0;
      }
      Displ_number = RData[1];
      displ_data_update();
      Refr_displ();
      ConnT_timer = millis();
    }
    RD_id = 0;
  }
}
