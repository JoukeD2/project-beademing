// libaries used

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <RotaryEncoder.h>
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>

// installizing datalogger shield rtc and sd card
RTC_DS1307 RTC;
File logfile;
// instalizing screen (20 characters by 4 rules
LiquidCrystal_I2C lcd(0x27, 20, 4);

// installizing rotary encoder used pin 6 and 7
#define PIN_IN1 7
#define PIN_IN2 6
RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

float frequency = 85; // times a minuted s
float volume = 75;    // ml a breath
unsigned long last = 0;
float time_now = 0; // used to keep track of time using millish
float time_previous = 0;
float total_time = (60 / frequency);          // total time it takes to breath in and out
float breath_in_time = total_time / 3;        // times it takes to breath in
float breath_out_time = (total_time / 3) * 2; // times it takes to breath out.
int loop_times = 0;
float air_in_chamber = 0; // keeps track of the amount of air that has entered the lungs
boolean active = 0;       // on off switch of the system
static int pos = 0;       // position of rotary encoder, used to control sysem

int select = 1;                 // 1 =frequency, 2: volume, 3: pressure, 4 disabled. keeps track of location of the cursor
boolean select_active = false;  // false will move the cursor, true will change value
boolean pressure_ready = false; // waits until the tank has some pressure
boolean recent_change = false;  // activa loops when values has changes
boolean print_lcd = false;
float pressure_in_chamber; // tracks the pressure in the container
float zero_press = 0;      // offset pressure of the sensor
float pressure = 0;
float pressure_tank = 0; // pressure in the tank
float pressure_h_in = 0;
float pressure_h_uit = 0;
float pressure_atmos = 0;
float resistance_airway = 0;
float resistance_box = 0;
float tolerance = 1;
float error_in = 0;
float error_out = 0;

float boxzero = 48;
float boxmax = 1000;
float vaczero = 50;
float vacmax = 1000;
float hbrugzero = 40;
float hbrugmax = 1024;
float atmoszero = 90;
float atmosmax = 1000;
float sensormaxpress = 10000;

float tank_min = 90;  // minimaal vacuum in tank in cmh20
float tank_max = 100; // max vacuum in tank in cmh20

// double time_previous = 0;
double pressure_high = 5;
long time_previous_air = 0;
float total_flow_in = 0;  // total inward flow in one breath
float total_flow_out = 0; // total outward flow
long voltage = 0;         // debug for flow sensor
double pressure_set = 0;
boolean mode_select = true; // true = pressure control, false if volume control
int counter = 0;
boolean just_off = true;       // used to place marker in logfile when the system is started and pauzed
boolean just_switched = false; // used to give
boolean breath_in = true;      // used in logfile to keep track of what fase the system is in.
int counter_serial = 0;
boolean print_ready = false;
int interfall = 10; // ms
long last_log = 0;
int breath_counter = 0;

int solvacpin = 29;
int soloutpin = 31;
int pumppin = 30;
int brugpin = 28;
int rotarybuttonpin = 3;
int emergencypin = 8;
int modepin = 5;

int flowsenspin = A15;
int presboxpin = A8;
int presvacpin = A10;
int preshbrug = A13;
int presatmospin = A12;

void setup()
{
  Serial.begin(115200);
  Serial.println("Start");
  pinMode(solvacpin, OUTPUT);             // solinode valve vacuum
  pinMode(soloutpin, OUTPUT);             // solinode valve outside air
  pinMode(rotarybuttonpin, INPUT_PULLUP); // button rotary encoder
  pinMode(brugpin, OUTPUT);               // H-brug aansturing
  pinMode(pumppin, OUTPUT);               // pump
  pinMode(presboxpin, INPUT);             // analog input pressure sensor chamber
  pinMode(flowsenspin, INPUT);            // flow sensor
  pinMode(emergencypin, INPUT_PULLUP);    // emergency
  pinMode(modepin, INPUT_PULLUP);         // mode select switch
  attachInterrupt(digitalPinToInterrupt(rotarybuttonpin), selector, FALLING);
  // Serial.println("Start lcd");
  lcd.init();
  // lcd.init();// initialize the lcd
  // Serial.println("init compl");
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.print("Setup please wait");
  Serial.println("Start wire");
  Wire.begin();
  RTC.begin(); // for rtc, a4 and a5 from datalogger schield needs to be connecter to 24 and 25

  if (!RTC.isrunning())
  { // starting rtc
    Serial.println("RTC is NOT running!");
    // sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

  if (!SD.begin(10, 11, 12, 13))
  {
    Serial.println("Card failed, or not present");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("sd error");
    lcd.setCursor(0, 1);
    lcd.print("please insert sd");
    lcd.setCursor(0, 2);
    lcd.print("and reset");

    while (0 == 0)
    {
    } // if no sd cart is present freeze the whole system
  }
  Serial.println("card initialized."); // creating a new file with a new name
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++)
  {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (!SD.exists(filename))
    {
      logfile = SD.open(filename, FILE_WRITE);
      break;
    }
  }
  Serial.println(filename);
  logfile.println("log file");
  logfile.println("date & time , millish , pressure , pressure wanted , total flow in , total flow out , airway resistance , frequency set , pressure set , volume set , inhale , pressure control , cycle number");
  logfile.flush();

  eepromReader(); // reading our the eeprom memory the setting from the last time the system was shutdown
  recalculator(); // calculationg the time for breathing in and out with the last optained data
  digitalWrite(pumppin, HIGH);
  // while (pressure_ready == false)

  // get_pressure();
  // print_data_serial(); digitalWrite(brugpin, HIGH);
  //   delay(10);

  get_pressure();        // readout the pressure from the sensors
  zero_press = pressure; // storing the zero offset
  // Serial.print("offset: ");
  // Serial.println(boxzero);
  lcd.setCursor(0, 1);
  lcd.print("logfile:");
  lcd.setCursor(0, 2);
  lcd.print(filename);
  delay(1500);
  lcd_printer();
}

void loop()
{

  if (digitalRead(emergencypin) == LOW)
  {
    emergency(); // emergency protocol started
  }
  float flow_in = 0;
  long milisch_start = millis();
  long milisch_stop = millis() + (breath_in_time * 1000);
  float flow_intochamber = volume / breath_in_time;              // calculating the dflow we want to have into the chamber
  float d_pressure = (pressure_set + error_in) / breath_in_time; // calcualtion the d pressure we want to have into the chamber

  if (pressure_tank <= tank_min)
  {
    digitalWrite(pumppin, HIGH);
  }
  if (pressure_tank >= tank_max)
  {
    digitalWrite(pumppin, LOW);
  }

  if (active == 1)
  {
    if (just_off == true)
    {
      DateTime now = RTC.now();
      logfile.print("device started at: ");
      logfile.print(now.day(), DEC);
      logfile.print("/");
      logfile.print(now.month(), DEC);
      logfile.print("/");
      logfile.print(now.year(), DEC);
      logfile.print(" ");
      logfile.print(now.hour(), DEC);
      logfile.print(":");
      logfile.print(now.minute(), DEC);
      logfile.print(":");
      logfile.print(now.second(), DEC);
      logfile.print("\t\t");
      logfile.println(millis());
      logfile.flush();
      just_off = false;
    }
    // digitalWrite(pumppin, HIGH); // activating pump relais
    air_in_chamber = 0;
    pressure_in_chamber = pressure;
    time_previous = millis();
    breath_counter++;
    digitalWrite(brugpin, HIGH);
    while (millis() <= milisch_stop)
    {
      time_now = millis(); //     ---------------------------------------------------- inademen
      breath_in = true;
      digitalWrite(soloutpin, LOW);
      // calculationg the total pressure and air we need to have in the chamber.
      air_in_chamber = air_in_chamber + ((flow_intochamber / 1000) * ((time_now - time_previous)));
      if (pressure_in_chamber <= pressure_set + error_out)
      {
        pressure_in_chamber = pressure_in_chamber + ((d_pressure / 1000) * ((time_now - time_previous)));
      }
      get_pressure(); // getting the pressure
      get_flow();     // and the flow
      datalogger();
      if (mode_select == true)
      {
        if (pressure <= pressure_in_chamber) // if the pressure in the chamber is higher than we want, we activate the solanode, if it is lower, we deactivate the solanode
        {
          digitalWrite(solvacpin, HIGH);
        }
        else
        {
          if (pressure >= pressure_in_chamber)
          {
            digitalWrite(solvacpin, LOW);
          }
        }
      }
      if (mode_select == false)
      {
        if (total_flow_in <= air_in_chamber)
        {
          digitalWrite(solvacpin, HIGH);
        }
        else
        {
          digitalWrite(solvacpin, LOW);
        }
      }

      encoder.tick();       // controls the rotary encoder
      encoder_controller(); // program to control screen and rotary encoder.

      print_data_serial();
      time_previous = time_now;
    }
    error_in = pressure_set - pressure;

    milisch_stop = millis() + (breath_out_time * 1000);
    float flowoutchamber = air_in_chamber / breath_out_time;
    d_pressure = (pressure + error_out) / breath_out_time;
    pressure_in_chamber = pressure;
    digitalWrite(brugpin, LOW);
    while (millis() <= milisch_stop) // -----------------------------uitademen
    {
      time_now = millis();
      digitalWrite(solvacpin, LOW);
      breath_in = false;
      air_in_chamber = air_in_chamber - ((flowoutchamber / 1000) * ((time_now - time_previous)));
      if (pressure_in_chamber > 0)
      {
        pressure_in_chamber = pressure_in_chamber - ((d_pressure / 1000) * ((time_now - time_previous)));
      }
      time_previous = time_now;
      // Serial.println(pressure_in_chamber);
      if (pressure >= pressure_in_chamber)
      {
        digitalWrite(soloutpin, HIGH);
      }
      else
      {
        if (pressure <= pressure_in_chamber)
        {
          digitalWrite(soloutpin, LOW);
        }
      }
      encoder.tick();
      encoder_controller();
      get_pressure();
      get_flow();
      print_data_serial();
      datalogger();
    }
    print_ready = true;
    datalogger();
    time_previous = millis();
    total_flow_in = 0;
    total_flow_out = 0;
    error_out = pressure - pressure_in_chamber;
  }
  else // if the system is not active
  {
    // disabled all the systems
    // digitalWrite(pumppin, LOW);
    // delay(10);
    digitalWrite(solvacpin, LOW);
    delay(10);
    digitalWrite(soloutpin, HIGH);

    if (digitalRead(modepin) == HIGH) // only when the system is not active, you can change from pressure to flow control
    {
      mode_select = true;
      if (just_switched == true)
      {
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("pressure control");
        lcd.setCursor(0, 2);
        lcd.print("selected");
        just_switched = false;
        delay(2000);
        lcd_printer();
      }
    }
    else
    {
      mode_select = false;
      if (just_switched == false)
      {
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("volume control");
        lcd.setCursor(0, 2);
        lcd.print("selected");
        just_switched = true;
        delay(2000);
        lcd_printer();
      }
    }
    if (just_off == false)
    {
      DateTime now = RTC.now();
      logfile.print("device pauzed off at: ");
      logfile.print(now.day(), DEC);
      logfile.print("/");
      logfile.print(now.month(), DEC);
      logfile.print("/");
      logfile.print(now.year(), DEC);
      logfile.print(" ");
      logfile.print(now.hour(), DEC);
      logfile.print(":");
      logfile.print(now.minute(), DEC);
      logfile.print(":");
      logfile.print(now.second(), DEC);
      logfile.print("\t\t");
      logfile.println(millis());
      logfile.flush();
      just_off = true;
    }
  }

  // serial contol system underneath here
  // F: "value" = frequency
  // V: "value" = volume
  // P: "value" = pressure
  // on= starts the system
  // off = stops the system
  // stop = emergency stop system
  if (Serial.available() > 0)
  {
    String input = Serial.readString();
    input.trim();
    if (input.indexOf("F") >= 0)
    {
      input.replace("F:", "");
      Serial.println(input);
      float frequency_new = input.toFloat();
      Serial.print("new frequency: ");
      Serial.println(frequency_new);
      if ((frequency_new > 30) && (frequency_new < 200))
      {
        frequency = frequency_new;
        lcd_printer();
        recalculator();
        eepromWriter();
      }
      else
      {
        Serial.println("warning: data out of save range");
      }
    }
    else if ((input.indexOf("V") >= 0))
    {
      input.replace("V:", "");
      Serial.println(input);
      float volume_new = input.toFloat();
      Serial.print("new volume: ");
      Serial.println(volume_new);
      if ((volume_new > 0) && (volume_new < 10))
      {
        volume = volume_new;
        lcd_printer();
        recalculator();
        eepromWriter();
      }
    }
    else if ((input.indexOf("P") >= 0))
    {
      input.replace("P:", "");
      Serial.println(input);
      float pressure_new = input.toFloat();
      Serial.print("new pressure: ");
      Serial.println(pressure_new);
      if ((pressure_new > 0) && (pressure_new < 30))
      {
        pressure_set = pressure_new;
        lcd_printer();
        recalculator();
        eepromWriter();
      }
    }
    else if (input.indexOf("off") >= 0)
    {
      Serial.println("system disabled");
      active = 0;
      lcd_printer();
    }
    else if (input.indexOf("on") >= 0)
    {
      Serial.println("system enabled");
      active = 1;
      lcd_printer();
    }
    else if (input.indexOf("stop") >= 0)
    {
      emergency();
    }
    else
    {
      Serial.println("syntax error or unknow command");
    }
  }

  encoder.tick();
  encoder_controller();
  // if anything is changes is the settings we store the change into the eeprom and recalculate all the the durations
  if (recent_change == true)
  {
    recalculator();
    recent_change = false;
    eepromWriter();
  }
  if (print_lcd == true) // update the lcd only when values changes
  {
    lcd_printer();
    print_lcd = false;
  }
  get_pressure(); // still measuring all the systems for monitor purposes
  get_flow();
  print_data_serial();
}
void lcd_printer() // printing all the stuff on the lcd
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("frequency: ");
  lcd.print((frequency));
  lcd.setCursor(0, 1);
  lcd.print("volume: ");
  lcd.print(volume);
  lcd.print("ml");
  lcd.setCursor(0, 2);
  lcd.print("pres: ");
  lcd.print(pressure_set);
  lcd.print("H2O");
  if (select_active == true)
  {
    switch (select)
    {
    case 1:
      lcd.setCursor(18, 0);
      lcd.print("<-");
      break;
    case 2:
      lcd.setCursor(18, 1);
      lcd.print("<-");
      break;
    case 3:
      lcd.setCursor(18, 2);
      lcd.print("<-");
      break;
    case 4:
      lcd.setCursor(18, 3);
      lcd.print("<-");
      break;
    }
  }
  else
  {
    switch (select)
    {
    case 1:
      lcd.setCursor(19, 0);
      lcd.print("<");
      break;
    case 2:
      lcd.setCursor(19, 1);
      lcd.print("<");
      break;
    case 3:
      lcd.setCursor(19, 2);
      lcd.print("<");
      break;
    case 4:
      lcd.setCursor(19, 3);
      lcd.print("<");
      break;
    }
  }

  if (active == 0)
  {
    lcd.setCursor(0, 3);
    lcd.print("disabled");
  }
  else
  {
    lcd.setCursor(0, 3);
    lcd.print("active");
  }
}

void recalculator()
{
  total_time = (60 / frequency);
  breath_in_time = total_time / 3;
  breath_out_time = (total_time / 3) * 2;
}

void eepromWriter() // storing the data on the arduino eeprom
{
  // locations data
  //  frequency = 10;
  //  volume = 20;
  //  pressure_set = 30;
  EEPROM.write(10, frequency);
  EEPROM.write(20, volume);
  EEPROM.write(30, pressure_set);
  EEPROM.write(40, active);
}

void eepromReader() // reading the data on on the arudino eeprom
{
  frequency = EEPROM.read(10);
  volume = EEPROM.read(20);
  pressure_set = EEPROM.read(30);
  active = EEPROM.read(40);
}

void encoder_controller() // controles the rotary encoder
{
  int newPos = encoder.getPosition();
  if (pos != newPos)
  {
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println((int)(encoder.getDirection()));
    int directions = newPos - pos;
    pos = newPos;
    if (select_active == false)
    {
      select = select + directions;
      if (select >= 5)
      {
        select = 1;
      }
      else if (select <= 0)
      {
        select = 4;
      }
    }
    else if ((select_active == true) && (select == 1))
    {
      frequency = frequency + directions;
    }
    else if ((select_active == true) && (select == 2))
    {
      volume = volume + (0.1 * directions);
    }
    else if ((select_active == true) && (select == 3))
    {
      pressure_set = pressure_set + (0.1 * directions);
    }
    else if ((select_active == true) && (select == 4))
    {
      if (active == 0)
      {
        active = 1;
        eepromWriter();
      }
      else
      {
        active = 0;
        eepromWriter();
      }
    }
    lcd_printer();
    recent_change = true;
  }
}

void selector() // interrupt for rotary encoder used to select items
{
  if (digitalRead(rotarybuttonpin) == LOW)
  {
    if (select_active == true)
    {
      select_active = false;
    }
    else
    {
      select_active = true;
    }
    print_lcd = true;
  }
}

void get_pressure() // reading the pressure sensors and calculating the pressure
{
  int signal_in = analogRead(presboxpin);
  // Serial.print("pressure_raw: ");
  // Serial.print(signal_in);
  signal_in = ((signal_in - boxzero) * sensormaxpress) / (boxmax - boxzero);
  pressure = (signal_in * 10.19) / 1000;
  // Serial.print("pressure: ");
  // Serial.print (pressure);
  // Serial.print("\t");

  signal_in = analogRead(presvacpin);
  // Serial.print("vac_raw: ");
  // Serial.print(signal_in);
  signal_in = ((signal_in - vaczero) * sensormaxpress) / (vacmax - vaczero);
  pressure_tank = (signal_in * 10.19) / 1000;
  // Serial.print("vac: ");
  // Serial.print (pressure_tank);
  // Serial.print("\t");

  signal_in = analogRead(preshbrug);
  // Serial.print("brug_raw: ");
  // Serial.print(signal_in);
  signal_in = ((signal_in - hbrugzero) * sensormaxpress) / (hbrugmax - hbrugzero);
  pressure_h_in = (signal_in * 10.19) / 1000;
  // Serial.print("h brug: ");
  // Serial.println (pressure_h_in);
  // Serial.println("\t");

  // signal_in = analogRead(A0);    -oude berekening
  // voltage = map(signal_in, 0, 1023, 0, 5000);
  // Serial.print("in: ");
  // Serial.print (voltage);
  // Serial.print("\t");
  // pressure = ((((voltage / 1000) - 0.04) / 0.090)*10.1974);
}

void get_flow() // reading the flow sensors, calculating the flow and intergrating to get the total amount of air getting into the rat
{
  long time_start = millis();
  int adc_value = analogRead(flowsenspin);
  voltage = map(adc_value, 0, 1023, 0, 5000);
  double x_3 = ((2.45 * pow(10, -8)) * (pow(voltage, 3)));
  double x_2 = (1.51 * pow(10, -4)) * (pow(voltage, 2));
  double x_1 = 0.3897 * voltage;
  double x_0 = 2.68 * pow(10, 2);

  double flow_sections = x_3 - x_2 + x_1 - x_0 + 4.64;
  float d_flow = ((flow_sections * ((time_start - time_previous_air)) / 60000));
  resistance_airway = (pressure_h_in / d_flow);
  if (d_flow >= 0)
  {
    if (breath_in == true)
    {
      total_flow_in = total_flow_in + d_flow;
    }
    else
    {
      total_flow_out = total_flow_out + d_flow;
    }
    time_previous_air = millis();
  }
}

void print_data_serial() // serial print a lot of data.
{
  if (counter_serial >= 10)
  {
    // Serial.print((pressure - boxzero), 3);
    // Serial.print("\t");
    // Serial.print(digitalRead(modepin));

    // Serial.print((pressure_tank ), 3);
    // Serial.print("\t");
    // Serial.print(pressure_high);
    // Serial.print("\t");
    // Serial.print(breath_in);
    // Serial.print("\t");
    // Serial.print(total_flow_in);
    // Serial.print("\t");
    // Serial.print(air_in_chamber);
    // Serial.print("\t");
    // Serial.print(voltage);
    // Serial.print("\t");
    // Serial.println(pressure_in_chamber);
    // Serial.print("\t");
    // Serial.println(breath_counter);
    counter_serial = 0;
    // Serial.print("\t");
  }
  counter_serial++;
}

void emergency() // emergency program, removed all the pressure from the system
{
  Serial.println("emergency : repressurization active");
  active = 0;
  digitalWrite(pumppin, LOW);
  digitalWrite(solvacpin, LOW);
  digitalWrite(soloutpin, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("emergency");
  lcd.setCursor(0, 1);
  lcd.print("pressurization");
  lcd.setCursor(0, 2);
  lcd.print("please wait");
  DateTime now = RTC.now();
  logfile.print("emergency stop at:");
  logfile.print(now.day(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.year(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print("\t\t");
  logfile.println(millis());
  logfile.flush();

  delay(10000);
  lcd_printer();
}

void datalogger() // log the data to the sd cart
{
  if (last_log + interfall <= millis())
  {
    last_log = millis();
    DateTime now = RTC.now();
    logfile.print(now.day(), DEC);
    logfile.print("/");
    logfile.print(now.month(), DEC);
    logfile.print("/");
    logfile.print(now.year(), DEC);
    logfile.print(" ");
    logfile.print(now.hour(), DEC);
    logfile.print(":");
    logfile.print(now.minute(), DEC);
    logfile.print(":");
    logfile.print(now.second(), DEC);
    logfile.print("\t\t,");
    logfile.print(millis());
    logfile.print("\t,");
    logfile.print(pressure);
    logfile.print("\t,");
    logfile.print(pressure_in_chamber);
    logfile.print("\t,");
    logfile.print(total_flow_in);
    logfile.print("\t,");
    logfile.print(total_flow_out);
    logfile.print("\t,");
    logfile.print(resistance_airway);
    logfile.print("\t,");
    logfile.print(frequency);
    logfile.print("\t,");
    logfile.print(pressure_set);
    logfile.print("\t,");
    logfile.print(volume);
    logfile.print("\t,");
    logfile.print(breath_in);
    logfile.print("\t,");
    logfile.print(mode_select);
    logfile.print("\t,");
    logfile.print(breath_counter);
    logfile.println();
    if (print_ready == true)
    {
      logfile.flush();
      print_ready = false;
    }
  }
  counter++;
}