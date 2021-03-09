//Including Libraries:

//LCD Dependencies: 
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_INA219.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>

//ESP Definitions
#define BLYNK_PRINT Serial
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <WidgetRTC.h>
#include <WidgetTimeInput.h>

char auth[] = "uy6aArnCIFXfVWBWAu6Mjxya6CNAJyrx";
char ssid[] = "Linksys01995_24";
char pass[] = "jgvn08v94t";
#define EspSerial Serial1
#define ESP8266_BAUD 115200
ESP8266 wifi(&EspSerial);
BlynkTimer timekeeper;
WidgetRTC rtc;
int start_timings[2] = {0,0};
int stop_timings[2] = {0,0};
int timenow[3] = {0,0,0};

//LCD Definitions:
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

/* MCP4725 Definitions:
 * AR1 - 0x61 (97)
 * AR2 - 0x60 (96)
 */
Adafruit_MCP4725 batt_conv;
Adafruit_MCP4725 load_conv;

/* INA219 Definitions:
 *  Load is 0x40 (64)
 *  Batt is 0x41 (65)
 *  Panels is 0x45 (69)
*/
Adafruit_INA219 ina219_load(0x40); 
Adafruit_INA219 ina219_batt(0x41);
Adafruit_INA219 ina219_panels(0x45);

//Rotary Encoder definitions:
long timer = 0;
bool aLastState = false;
byte outputA = 2, outputB = 3, button = 4, modeSwitch = 8;
bool lastButtonState = false, buttonState;
boolean state_2, state_3, mode;

//SMPS definitions
byte switch_load = 6, switch_batt = 7, switch_panels = 5;

/*
 * GUI definitions: 
 * We will create an array which will correspond to the pages (multi-dimensional). 
 * page --> row --> column
 * Each of these positions will hold 3 values -> x coord, y coord
 * Ie: {0,0,0}
 * 
 * There will be another 3 value array that will store our CURRENT position
 * [0,0,0] - [first page, first row, LEVEL which we are currently on (editing or not)]
 */
//[page][row][column][x/y]
int layout[3][4][2][2] = {{{{0,0},{11,0}},{{0,1},{11,1}},{{0,2},{11,2}},{{0,3},{11,3}}},
                         {{{0,0},{11,0}},{{0,1},{11,1}},{{0,2},{11,2}},{{0,3},{11,3}}},
                         {{{0,0},{11,0}},{{0,1},{11,1}},{{0,2},{11,2}},{{0,3},{11,3}}}};
String text[3][4][2] =  {{{"Vin :","Iin :"},{"Vout:","Iout:"},{"VBat:","IBat:"},{"Pow :","Time:"}},
                        {{"Cyc:","NEW:"},{"Flt:","NEW:"},{"OFF:","NEW:"},{"Ld :","NEW:"}},
                        {{"Cap:","NEW:"},{"AbC:","NEW:"},{"ICO:","NEW:"},{"NIL:","NEW:"}}};
//We define an array which stores the data in the form of INT so that we save on memory:
int data[3][4][2] = {{{0,0},{0,0},{0,0},{0,0}},
                     {{144,0},{135,0},{116,0},{120,0}},
                     {{0,0},{0,0},{0,0},{0,0}}};
int data_compare[4][2] = {{0,0},{0,0},{0,0},{0,0}};
int limits[3][4][2] = {{{-1,-1},{-1,-1},{-1,-1},{-1,-1}},
                       {{150,120},{150,120},{140,110},{200,0}},
                       {{1000,100},{100,0},{100,0},{-1,-1}}}; //0 is max, 1 is min
int pos[3] = {0,0,0}; //page number, row, column

//Data for charging sequence
int desired_voltage = 0;
int load_counter = 2000, batt_counter = 1600;
int last_power = 0, last_voltage = 0, last_current = 0;

BLYNK_CONNECTED(){
  rtc.begin();
  //sync the on-board RTC
}

void setup() {
  // Serial Setup:
  Serial.begin(9600);
  Serial.println("ready!");
  
  //MOSFETs setup
  pinMode(switch_load, OUTPUT);
  pinMode(switch_batt, OUTPUT);
  pinMode(switch_panels, OUTPUT);
  //Panel and battery MOSFETs not for switching hence set to high
  digitalWrite(switch_panels, HIGH);

  //MCP4725 Setup
  batt_conv.begin(0x60); //AR2
  load_conv.begin(0x61); //AR1

  //INA219 Setup
  ina219_batt.begin(); 
  ina219_load.begin(); 
  ina219_panels.begin(); 

  //ESP8266 Setup
  EspSerial.begin(ESP8266_BAUD);
  Blynk.begin(auth, wifi, ssid, pass);
  timekeeper.setInterval(600000L, requestTime);
  setSyncInterval(100 * 60);
    
  //LCD I/O pin setup:
  pinMode(button, INPUT_PULLUP);
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  pinMode(modeSwitch, INPUT_PULLUP);

  mode = digitalRead(modeSwitch);

  //LCD Setup:

  //update the screen
  if(mode == 1){
    text[0][2][0] = "Vmpp:"; text[0][2][1] = "Impp";
    text[0][1][0] = "VBat:"; text[0][1][1] = "IBat";
    }
  Serial.println(mode);
  //Splash Screen
  lcd.begin(20,4);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,5); lcd.print("MPPT V1.0");
  lcd.setCursor(1,0); lcd.print("By Ethan Chua, 2020");
  lcd.setCursor(0,2); lcd.print("Mode: "); lcd.print(mode);
  delay(5000);
  lcd.clear();
  updateLCD(0,2,0);

}

void loop() {
  // During the loop, we will constantly check to see if there is a change in the state of either the rotary encoder
  //and or the button. When this happens, we will cycle through our positions in the GUI

  if(!debounce(button, 100)){ //if the button is pressed, then flip the button state
     buttonState = !buttonState;
  }
  if(buttonState != lastButtonState){ // if the last button state is different, update the pos with the status of the button
    if(!buttonState && pos[0] != 0) data[pos[0]][pos[1]][0] = data[pos[0]][pos[1]][1]; //if the button state becomes low and we are not on home screen, update the data
    updateLCD(2,0,0);
    pos[2] = buttonState;
    lastButtonState = buttonState;
  }
  int encoderResult = rotaryEncoder();
  if(encoderResult == 0){
  }else if(encoderResult == -1){
      byte row_cutoff = 3;
      Serial.println("Encoder moves in the CW direction");
      if(pos[2] == 1 && pos[0] != 0){ //if the button state is HIGH and we are not on page 1
        if(data[pos[0]][pos[1]][pos[2]] < limits[pos[0]][pos[1]][0]){ //0 max
          data[pos[0]][pos[1]][pos[2]] += 1;
        } else{
          data[pos[0]][pos[1]][pos[2]] = limits[pos[0]][pos[1]][1]; //1 min
        }
        updateLCD(2,0,pos[2]);
      } else{
        if(pos[0] == 0) row_cutoff = 3;
      if(pos[1] < row_cutoff){
        pos[1] += 1;
        updateLCD(1, pos[1]-1, pos[1]); //Update the position of the *
      }else{
        pos[1] = 0;
        //updateLCD(1, 3, pos[1]); //Update the position of the * 
        if(pos[0] < 2){
          pos[0] += 1;
          updateLCD(0, pos[0]-1, pos[0]); //Update page
        } else{
          pos[0] = 0;
          updateLCD(0, 2, pos[0]); //we pass in the RANK, OLD, NEW
        }
      }
      Serial.print("Position"); Serial.println(String(pos[0])+String(pos[1])+String(pos[2]));
      }
  }else if(encoderResult == 1){
    Serial.println("Encoder moves in the CCW direction");
      if(pos[2] == 1 && pos[0] != 0){ //if the button state is HIGH and we are not on page 1
        if(data[pos[0]][pos[1]][pos[2]] > limits[pos[0]][pos[1]][1]){ //1 min
          data[pos[0]][pos[1]][pos[2]] -= 1;
        } else{
          data[pos[0]][pos[1]][pos[2]] = limits[pos[0]][pos[1]][1]; //1 min
        }
        updateLCD(2,0,pos[2]);
      } else{
        if(pos[1] > 0){
        pos[1] -= 1;
        updateLCD(1, pos[1]+1, pos[1]); //we pass in the RANK, OLD, NEW
      }else{
        if(pos[0] == 1) pos[1] = 2;
        else pos[1] = 3;
        //updateLCD(1, 0, pos[1]); //we pass in the RANK, OLD, NEW
        if(pos[0] > 0){
          pos[0] -= 1;
          updateLCD(0, pos[0]+1, pos[0]); //we pass in the RANK, OLD, NEW
        } else{
          pos[0] = 2;
          updateLCD(0, 0, pos[0]); //we pass in the RANK, OLD, NEW
        }
      }
      Serial.print("Position"); Serial.println(String(pos[0])+String(pos[1])+String(pos[2]));
     }
}
chargeSequence();
Blynk.run();
timekeeper.run();
}

void updateLCD(byte rank, byte from, byte to){
  //We will reflash the LCD FROM the OLD POSITION TO the NEW POSITION: 
  switch(rank){
    case 0: //this means we want to swap pages
      lcd.clear(); //clear the whole screen in this case
      for(byte i = 0; i <= 3; i++){ //row
        for(byte j = 0; j <=1; j++){ //column
          if(layout[to][i][j][0] != -1){
            lcd.setCursor(layout[to][i][j][0],layout[to][i][j][1]);
            lcd.print(text[to][i][j]);
            //lcd.print("     ");
            lcd.setCursor(layout[to][i][j][0]+5,layout[to][i][j][1]);
            lcd.print((data[to][i][j] - data[to][i][j]%10)/ 10); lcd.print("."); lcd.print(data[to][i][j]%10);
            //delay(1000);
          }
        }
      }
      if(from == 0 && to == 2){ //if it's coming from page 1 going to page 3,
        lcd.setCursor(9,3);
        lcd.print("*");
      } else if(from == 2 && to == 0){ //if it's coming from page 3 going to page 1
        lcd.setCursor(9,0);
        lcd.print("*");
      } else if(from < to){
        lcd.setCursor(9,0);
        lcd.print("*");
      } else{
        if(from == 1 && to == 0) lcd.setCursor(9,2);
        else lcd.setCursor(9,3); 
        lcd.print("*");
      }
      break;
    case 1: //this means we want to swap rows (move the cursor indicator down to rows)
    lcd.setCursor(9, from);
    lcd.print(" ");
    lcd.setCursor(9, to);
    lcd.print("*");
      break;

    case 2: //this means we want to increment the values in NEW
    lcd.setCursor(layout[pos[0]][pos[1]][to][0] + 5, layout[pos[0]][pos[1]][to][1]);
    lcd.print("    ");
    lcd.setCursor(layout[pos[0]][pos[1]][to][0] + 5, layout[pos[0]][pos[1]][to][1]);
    lcd.print((data[pos[0]][pos[1]][to] - data[pos[0]][pos[1]][to]%10)/ 10); lcd.print("."); lcd.print(data[pos[0]][pos[1]][to]%10);
      break;
  }
}

int rotaryEncoder(){
  bool aState = digitalRead(outputA);
  bool bState = digitalRead(outputB);
  if(aState != aLastState){
    if(bState != aLastState){
      //this means that the rotary encoder has been rotated in one direction (we dk if it is CW or CCW)
      aLastState = aState;
      return -1;
    } else{
      //this means that the rotary encoder has been rotated in the other direction (we dk if it is CW or CCW)
      aLastState = aState;
      return 1;
    }       
  }
  else{
    //this means that the rotary encoder has not moved
    return 0;
  } 
}
void chargeSequence(){
  /*
   * String text[3][4][2] =  {{{"Vin :","Iin :"},{"Vout:","Iout:"},{"VBat:","IBat:"},{"Pow :","Time:"}},
                              {{"Cyc:","NEW:"},{"Flt:","NEW:"},{"OFF:","NEW:"},{"Ld :","NEW:"}},
                              {{"Cap:","NEW:"},{"AbC:","NEW:"},{"ICO:","NEW:"},{"NIL:","NEW:"}}};
   */
   //Setting the desired voltages
    if(data[0][2][0] < data[1][0][0] && data[0][2][1] > round(data[2][0][0] * data[2][1][0])){ //if VBat < cycling voltage && Current is larger than Cap * AbC
    desired_voltage = data[1][0][0]; //Set the VBat to cycling voltage (BULK / ABSORPTION)
  }else{
    desired_voltage = data[1][1][0]; //set the Vout to the float voltage
  }

  //Collecting Data from sensors
  data[0][0][0] = round(ina219_panels.getBusVoltage_V()*10+ina219_panels.getShuntVoltage_mV()/100); //Take reading of Vin
  //Serial.println("Panels:");
  //Serial.println(round(ina219_panels.getBusVoltage_V()*10+ina219_panels.getShuntVoltage_mV()/100));
  data[0][0][1] = round(ina219_panels.getShuntVoltage_mV()/1.267857143); //Take reading of Iin
  if(data[0][0][1] < 0) data[0][0][1] = 0;
  //Serial.println(data[0][0][1]);
  //Serial.println();
  data[0][1][0] = round(ina219_load.getBusVoltage_V()*10+ina219_load.getShuntVoltage_mV()/100); //Take reading of Vout
  //Serial.println("Load:");
  //Serial.println(round(ina219_load.getBusVoltage_V()*10+ina219_load.getShuntVoltage_mV()/100));
  data[0][1][1] = round(ina219_load.getShuntVoltage_mV()/2.685185185); //Take reading of Iout
  if(data[0][1][1] < 0) data[0][1][1] = 0;
  //Serial.println(data[0][1][1]);
  //Serial.println();
  data[0][2][0] = round(ina219_batt.getBusVoltage_V()*10+ina219_batt.getShuntVoltage_mV()/100); //Take reading of Vout
  //Serial.println("Battery:");
  //Serial.println(round(ina219_batt.getBusVoltage_V()*10+ina219_batt.getShuntVoltage_mV()/100));
  data[0][2][1] = round(ina219_batt.getShuntVoltage_mV()/1.225225225); //Take reading of Iout
  if(data[0][2][1] < 0) data[0][0][1] = 0;
  //Serial.println(data[0][2][1]);  
  //Serial.println();
  
  data[0][3][0] = round(data[0][0][0] * data[0][0][1]/10);

 //Updating the LCD

/*
 * text[3][4][2] =  {{{"Vin :","Iin :"},{"Vout:","Iout:"},{"VBat:","IBat:"},{"Pow :","Time:"}},
                        {{"Cyc:","NEW:"},{"Flt:","NEW:"},{"OFF:","NEW:"},{"Ld :","NEW:"}},
                        {{"Cap:","NEW:"},{"AbC:","NEW:"},{"ICO:","NEW:"},{"NIL:","NEW:"}}};
 */
 //Actuating the SMPS
  
 if(mode == 0){
    if(data[0][1][0] < data[1][3][0] && data[0][1][0] <= 200){ //if Vload < Ld and < 20.0
      if(load_counter > 0 && data[0][0][0] > 50) load_counter--;  
    }else if(data[0][1][0] > data[1][3][0] && data[0][1][0] >= 0){
      if(load_counter < 4095) load_counter++;
    }
  load_conv.setVoltage(load_counter, false);

 if(data[0][2][0] < desired_voltage && data[0][2][0] <= 200){ //if VBatt < desired voltage and < 20.0
 if(batt_counter > 0 && data[0][2][0] > 50) batt_counter--;  
  }else if(data[0][2][0] > desired_voltage && data[0][2][0] >= 0){
    if(batt_counter < 4095) batt_counter++;
}
  batt_conv.setVoltage(batt_counter, false);

  //Battery management measures:
  if(data[0][2][0] < data[1][2][0]){ //if VBatt < OFF
    digitalWrite(switch_batt, LOW);
  }
  else{
    digitalWrite(switch_batt, HIGH);
  }
  //Load management measures:
  if(start_timings[1] < timenow[1] < stop_timings[1]){
    digitalWrite(switch_load, HIGH);
  } else{
    digitalWrite(switch_load, LOW);
  }
 } else{
    if(data[0][1][0] < desired_voltage && data[0][1][0] <= 200){ //if Vload < desired voltage and <= 20.0
      if(load_counter > 0 && data[0][1][0] > 50) load_counter--;  
    }else if(data[0][1][0] > desired_voltage && data[0][1][0] >= 0){ //if Vload > desired voltage and >= 0
      if(load_counter < 4095) load_counter++;
    }
    load_conv.setVoltage(load_counter, false);
    //MPPT magic happens here
   /*
    * The basic idea is that we want to set the voltage of converter one to the voltage which maximizes the power 
    * derived from the solar panel. We do this via a bunch of algorithms:
    * Perturb and Observe (P&O) - 0
    * Incremental Conductance (IC) - 1
    */
   boolean algorithm = 0; //change the value to try out the different algorithms
   if(algorithm == 0){
    int MPPT_Step = 10;
    if(data[0][3][0] - last_power > 10){ //if power increased by more than 1W
       // do nothing to the step sign as we are heading in the right track;
    } else if(data[0][3][0] == last_power){
      MPPT_Step = 0;
      } else{
      //flip the sign as we are moving away
      MPPT_Step = -MPPT_Step;
    }
    batt_counter += MPPT_Step;
    last_power = data[0][3][0];
    batt_conv.setVoltage(batt_counter, false);
   } else if(algorithm == 1){
    int MPPT_Step = 10;
    int delta_I = data[0][0][1] - last_current;
    int delta_V = data[0][0][0] - last_voltage;
    int delta_P = data[0][3][0] - last_power;

    if(delta_V == 0){
      if(delta_I == 0){
       //do nothing 
      } else if(delta_I > 0){
        batt_counter -= MPPT_Step; //increase voltage 
      } else batt_counter += MPPT_Step; //decrease voltage
    } else{
      if(delta_P == 0){
        // do nothing
      } else{
        if(delta_P > 0){
          batt_counter -= MPPT_Step; //increase voltage
        } else batt_counter += MPPT_Step; //decrease voltage
      }
    }
    last_voltage = data[0][0][0];
    last_current = data[0][0][1];
    last_power = data[0][3][0];
    batt_conv.setVoltage(batt_counter, false);
   }
   //Battery management measures:
  if(data[0][1][0] < data[1][2][0]){ //if VBatt < OFF
    digitalWrite(switch_batt, LOW);
  }
  else{
    digitalWrite(switch_batt, HIGH);
  }
  //Load management measures:
  if(start_timings[1] < timenow[1] < stop_timings[1]){
    digitalWrite(switch_load, HIGH);
  } else{
    digitalWrite(switch_load, LOW);
  }
 }

  if(pos[0] == 0) refreshLCD(); //refresh LCD with values if we are on page 0
}

boolean debounce(byte pin, int wait){
    boolean state_1 = digitalRead(pin);
    if(state_1 != state_2) timer = millis();
     if(millis()- timer > wait && state_1 != state_3){
      state_3 = state_1;
      return state_3;
     }
    state_2 = state_1;
    return true;
}
void refreshLCD(){
  /*
   * int data[3][4][2] = {{{0,0},{0,0},{0,0},{0,0}},
                     {{144,0},{135,0},{116,0},{120,0}},
                     {{0,0},{0,0},{0,0},{0,0}}};
   */
  //manually compare over page one of data:
  for(byte i = 0; i < 4; i++){
    for(byte j = 0; j < 2; j++){
      if(abs(data[0][i][j] - data_compare[i][j])>5){
        lcd.setCursor(layout[pos[0]][i][j][0] + 5, layout[pos[0]][i][j][1]);
        lcd.print("    ");
        lcd.setCursor(layout[pos[0]][i][j][0] + 5, layout[pos[0]][i][j][1]);
        lcd.print((data[pos[0]][i][j] - data[pos[0]][i][j]%10)/ 10); lcd.print("."); lcd.print(data[pos[0]][i][j]%10);
          
        data_compare[i][j] = data[0][i][j]; //update values
         
      }
    }
  }
}
void requestTime(){
  timenow[0] = hour();
  timenow[1] = minute();
  timenow[2] = second();
  Serial.print("Time: ");
  for(byte j = 0; j < 3; j++){
    Serial.print(timenow[j]);
    if(j < 2) Serial.print(":");
  }
  Serial.println();
}
/*
 * V0 - Panel Voltage
 * V1 - Panel Current
 * V2 - Battery Voltage
 * V3 - Battery Current
 * V4 - Load Voltage (Not used) 
 * V5 - Load Current (Not used)
 * V6 - Load Switch (Not used) 
 * V7 - Time Input
 * String text[3][4][2] =  {{{"Vin :","Iin :"},{"Vout:","Iout:"},{"VBat:","IBat:"},{"Pow :","Time:"}},
                             {{"Cyc:","NEW:"},{"Flt:","NEW:"},{"OFF:","NEW:"},{"Ld :","NEW:"}},
                             {{"Cap:","NEW:"},{"AbC:","NEW:"},{"ICO:","NEW:"},{"NIL:","NEW:"}}};
 */
BLYNK_READ(V0){
  Blynk.virtualWrite(V0,float(data[0][0][0] / 10));
}
BLYNK_READ(V1){
  Blynk.virtualWrite(V1,float(data[0][0][1] / 10));
}
BLYNK_READ(V2){
  Blynk.virtualWrite(V2,float(data[0][2][0] / 10));
}
BLYNK_READ(V3){
  Blynk.virtualWrite(V3,float(data[0][2][1] / 10));
}
BLYNK_READ(V4){
  Blynk.virtualWrite(V4,float(data[0][1][0] / 10));
}
BLYNK_READ(V5){
  Blynk.virtualWrite(V5,float(data[0][1][1] / 10));
}
BLYNK_WRITE(V6){
  
}
BLYNK_WRITE(V7){
  TimeInputParam t(param);
    if (t.hasStartTime()){
    start_timings[0] = t.getStartHour();
    start_timings[1] = t.getStartMinute();
  }
  // Process stop time
  if (t.hasStopTime())
  {
    stop_timings[0] = t.getStopHour();
    stop_timings[1] = t.getStopMinute();
  }
}
