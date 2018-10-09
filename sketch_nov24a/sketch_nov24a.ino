// include the library code:
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <SHT1x.h>

#define dataPin  A4
#define clockPin A5

SHT1x sht1x(dataPin, clockPin);
#define lcdBack 6

float setPoint;

LiquidCrystal lcd(12, 11, 10, 9, 8, 7);

int myButton[] = {0,1,3,4,5};
float vOutLm, valLm;
int posisiKursor=0;
char lcdBuff[16];

float temp_c;
float temp_f;
float humidity;

////##### Variable fuzzyfikasi #####/////
float error=0, lastError=0;         // variable untuk menyimpan error dan lastError, lastError = deltaError
int kondisi[5], deltaKondisi[5];    // variable kondisi digunakan untuk menyimpan posisi nilai error sekarang, dan deltaKondisi nilai deltaError sekarang
int counterKondisi = 0;
float valKondisi[5];                //valKondisi = menampung nilai keluaran defuzzyfikasi error
float valDeltaKondisi[5];           //valDeltaKondisi = menampung nilai keluaran defuzzyfikasi deltaError
float dkKondisi[25];                 //dkKondisi = menampung nilai kombinasi / clipping pada error dan deltaError
float valZ[40];                      //valZ = menampung nilai output defuzzyfikasi

float maxVS = 132;
float minVS = 66;

float maxSWu = 132;
float minSWu = 66;

float maxSWd = 66;
float minSWd = 132;

float maxMIu = 132;
float minMIu = 66;

float maxMId = 132;  
float minMId = 398; //264

float maxFAu = 398;
float minFAu = 132;

float maxFAd = 132;
float minFAd = 796;

float maxVF = 132;
float minVF = 796;

float timeControl;

////##### Variable Triac Controll #####/////
int valMillis = 59;                 //tidak dipakai

int countZero = 0;

////##### Variable Menu #####/////
int posCursor = 6;
unsigned char valPoint = 0;
int minLm;
int maxLm;

int readLM;

boolean firstTime = false;
double millisNow = 0;

byte iconCursor[8] = {    //emoticon 
  0b00000,
  0b00000,
  0b01110,
  0b01110,
  0b01110,
  0b11111,
  0b01110,
  0b00100
};

void setup() {
  // set up the LCD's number of columns and rows:
  Serial.begin(9600);
  lcd.begin(16, 2);
  pinMode(lcdBack, OUTPUT);
  pinMode(A1, OUTPUT);
  digitalWrite(lcdBack, HIGH);

  attachInterrupt(0, flagZeroCrossing, FALLING);
  
  lcd.createChar (0, iconCursor);   
  
  for(int i=0; i<=4; i++)
  {
    pinMode(myButton[i], INPUT_PULLUP);
  }
}
void flagZeroCrossing()
{
  countZero++;
}
/// ### function/ method untuk membaca sensor LM
void readLMSens(boolean show)
{
  float dataAnalog = analogRead(A0);
  float readLMVout = (dataAnalog / 1024)*5;
  readLM = readLMVout*100;
  if(show == true && ((millis() - millisNow) > 2000 ))
  {
    lcd.clear();
    lcd.setCursor(0,0);
    //lcd.print(readLM);
    sprintf(lcdBuff, "Suhu:%3d'C", readLM);
    lcd.print(lcdBuff);
    lcd.print(" ");
    lcd.print(timeControl);
    lcd.setCursor(0,1);
    lcd.print(error,1); lcd.print(" "); lcd.print(lastError,1);
    Serial.println(readLM);
    millisNow = millis();
  }
}
void readSHT(boolean show)
{
  // Read values from the sensor
  temp_c = sht1x.readTemperatureC();
  temp_f = sht1x.readTemperatureF();
  humidity = sht1x.readHumidity();
  //temp_c = analogRead(A0);
  temp_c = ((temp_c/1024)*5)/10;
  valLm = temp_c;
  if(show == true)
  {
    lcd.setCursor(0,0);
    lcd.print(temp_c,2);
    lcd.print(" ");
    lcd.print(temp_f,2);
  }
}

// ### function fuzzyfikasi
void fuzzyfikasi()
{
  setPoint = EEPROM.read(0);
  //setPoint = 70;
  //readSHT(false);      // baca sensor LM
  readLMSens(true);
  error = setPoint - readLM;   // cari nilai error
  //error = setPoint - valMillis;
  if(error <= -10)    //jika error di bawah -10 maka...
  {
    kondisi[0] = 1;   //posisi sensor berada pada kondisi[0] = 1
    valKondisi[0] = 1;  //nilai keluaran fuzzyfikasi valKondisi[0] = 1
  }
  
  else if(error > -10 && error <= -5)  // jika error antara 0 - (-10) maka...
  {
    kondisi[0] = kondisi[1] = 1;      //sensor kondisi[0] dan kondisi[1] = 1

    valKondisi[0] = abs(((-5) - error))/5;  //mencari nilai output fuzzyfikasi kondisi[0]
    valKondisi[1] = abs((error - (-10)))/5;  // mencari nilai output fuzzyfikasi kondisi[1]
  }
  
  else if(error > -5 && error <= 0) //jika error antara 0 - 10 maka..
  {
    kondisi[1] = kondisi[2] = 1;      //posisi sensor berada di kondisi[1] dan kondisi[2]

    valKondisi[1] = abs((0 - error))/5; // mencari nilai output fuzzyfikasi valkondisi[1]
    valKondisi[2] = abs((error - (-5)))/5; // mencari nilai output fuzzyfikasi valkondisi[2]
  }
  else if(error > 0 && error <= 5) //jika error antara 0 - 10 maka..
  {
    kondisi[2] = kondisi[3] = 1;      //posisi sensor berada di kondisi[1] dan kondisi[2]

    valKondisi[2] = abs((5 - error))/5; // mencari nilai output fuzzyfikasi valkondisi[1]
    valKondisi[3] = abs((error - 0))/5; // mencari nilai output fuzzyfikasi valkondisi[2]
  }
  else if(error > 5 && error <= 10) //jika error antara 0 - 10 maka..
  {
    kondisi[3] = kondisi[4] = 1;      //posisi sensor berada di kondisi[1] dan kondisi[2]

    valKondisi[3] = abs((10 - error))/5; // mencari nilai output fuzzyfikasi valkondisi[1]
    valKondisi[4] = abs((error - 5))/5; // mencari nilai output fuzzyfikasi valkondisi[2]
  }
  
  else  // pengecualian
  {
    kondisi[4] = 1;   // posisi sensor berada di kondisi[2] = 1
    valKondisi[4] = 1; // output fuzzyfikasi valkondisi[2] = 1
  }

  //## proses defuzzyfikasi deltaError
  if(lastError <= -10)    //jika lastError di bawah -10 maka...
  {
    deltaKondisi[0] = 1;   //posisi sensor berada pada deltaKondisi[0] = 1
    valDeltaKondisi[0] = 1;  //nilai keluaran fuzzyfikasi valDeltaKondisi[0] = 1
  }
  
  else if(lastError > -10 && lastError <= -5)  // jika lastError antara 0 - (-10) maka...
  {  
    deltaKondisi[0] = deltaKondisi[1] = 1;      //sensor deltaKondisi[0] dan deltaKondisi[1] = 1

    valDeltaKondisi[0] = abs(((-5) - lastError))/5;  //mencari nilai output fuzzyfikasi deltaKondisi[0]
    valDeltaKondisi[1] = abs((lastError - (-10)))/5;  // mencari nilai output fuzzyfikasi deltaKondisi[1]
  }
  
  else if(lastError > -5 && lastError <= 0) //jika lastError antara 0 - 10 maka..
  {
    deltaKondisi[1] = deltaKondisi[2] = 1;      //posisi sensor berada di deltaKondisi[1] dan deltaKondisi[2]

    valDeltaKondisi[1] = abs((0 - lastError))/5; // mencari nilai output fuzzyfikasi valDeltaKondisi[1]
    valDeltaKondisi[2] = abs((lastError - (-5)))/5; // mencari nilai output fuzzyfikasi valDeltaKondisi[2]
  }
  else if(lastError > 0 && lastError <= 5) //jika lastError antara 0 - 10 maka..
  {
    deltaKondisi[2] = deltaKondisi[3] = 1;      //posisi sensor berada di deltaKondisi[1] dan deltaKondisi[2]

    valDeltaKondisi[2] = abs((5 - lastError))/5; // mencari nilai output fuzzyfikasi valDeltaKondisi[1]
    valDeltaKondisi[3] = abs((lastError - 0))/5; // mencari nilai output fuzzyfikasi valDeltaKondisi[2]
  }
  else if(lastError > 5 && lastError <= 10) //jika lastError antara 0 - 10 maka..
  {
    deltaKondisi[3] = deltaKondisi[4] = 1;      //posisi sensor berada di deltaKondisi[1] dan deltaKondisi[2]

    valDeltaKondisi[3] = abs((10 - lastError))/5; // mencari nilai output fuzzyfikasi valDeltaKondisi[1]
    valDeltaKondisi[4] = abs((lastError - 5))/5; // mencari nilai output fuzzyfikasi valDeltaKondisi[2]
  }
  
  else  // pengecualian
  {
    deltaKondisi[4] = 1;   // posisi sensor berada di deltaKondisi[2] = 1
    valDeltaKondisi[4] = 1; // output fuzzyfikasi valDeltaKondisi[2] = 1
  }
  /*
  lcd.setCursor(0,0);
  lcd.print(valDeltaKondisi[0],1);
  lcd.print(" ");
  
  lcd.print(valDeltaKondisi[1],1);
  lcd.print(" ");
  
  lcd.print(valDeltaKondisi[2],1);

  lcd.setCursor(0,1);
  lcd.print(valDeltaKondisi[3],1);
  lcd.print(" ");
  
  lcd.print(valDeltaKondisi[4],1);
  lcd.print(" ");
  
  lcd.print(lastError,1);
  */
  
  /*
  for(int i=0; i<=2; i++)
  {
    lcd.setCursor(i, 0);
    lcd.print(kondisi[i]);
    lcd.setCursor(i,1);
    lcd.print(deltaKondisi[i]);
  }
  lcd.setCursor(4,1);
  lcd.print(error);
  lcd.print(" "); 
  lcd.print(lastError);
  */
  //zeroVariable();
}

//## function untuk mengembalikan seluruh variable menjadi 0, jika tidak di return 0 maka nilai lama tidak terhapus
void zeroVariable()
{
  for(int i=0; i<=4; i++)
  {
    valKondisi[i] = 0.0;
    valDeltaKondisi[i] = 0.0;
    deltaKondisi[i] = 0;
    kondisi[i] = 0;
  }
  for(int i=0; i<=24; i++)
  {
    dkKondisi[i] = 0;
    valZ[i] = 0;
  }
  for(int i=0; i<=39; i++)
  {
    valZ[i] = 0;
  }
  lastError = error;    // mencari nilai lastError / deltaError
}

// ## function ruleBase, mencari aturan - aturan yang digunakan
void ruleBase()
{
  counterKondisi = 0;
  for(int i=0; i<=4; i++) // diulang sampai 3 buah kondisi untuk nilai error
  {
    for(int j=0; j<=4; j++) // diulang sampai 3 buah kondisi untuk nilai deltaError
    {
      if(kondisi[i] == 1 && deltaKondisi[j] == 1) //jika nilai error berada di kondisi[ke-i] == 1 dan delta error di kondisi[ke-i] == 1 maka...
      {
        dkKondisi[counterKondisi] = min(valKondisi[i], valDeltaKondisi[j]);// cari nilai minimum dari nilai defuzzyfikasi error dan deltaError

        /*
        lcd.setCursor(0,0);
        sprintf(lcdBuff, "%i%i ", i,j);
        lcd.print(lcdBuff);
        lcd.print(dkKondisi[counterKondisi],1);
        //Serial.print(counterKondisi);Serial.print(" "); Serial.print(dkKondisi[counterKondisi],1); Serial.print(" error-> ");Serial.print(error); Serial.print(" "); Serial.println(lastError);
        
        lcd.setCursor(0,1);
        lcd.print(valKondisi[i],1);
        lcd.print(" ");
        lcd.print(valDeltaKondisi[j],1);
        lcd.print(" ");
        lcd.print(error,1);
        lcd.print(" ");
        lcd.print(lastError,1);
        */
        
      }
      counterKondisi++;
    }
  }

//#### derajat keanggotaan untuk keluaran VS ###////     error ==== deltaError
  valZ[0] = maxVS - ((maxVS-minVS) * dkKondisi[0]);// VL x VL = VS
  valZ[1] = maxVS - ((maxVS-minVS) * dkKondisi[1]);// VL x LW = VS
  valZ[2] = maxVS - ((maxVS-minVS) * dkKondisi[2]);// VL x MD = VS

  //#### derajat keanggotaan untuk keluaran SW ###////     error ==== deltaError
  valZ[3] = maxSWu - ((maxSWu-minSWu) * dkKondisi[3]);// VL x HI = SW up
  valZ[4] = maxSWu - ((maxSWd-minSWd) * dkKondisi[3]);// VL x HI = SW down

  //#### derajat keanggotaan untuk keluaran MI ###////     error ==== deltaError
  valZ[5] = maxMIu - ((maxMIu-minMIu) * dkKondisi[4]);// VL x VH = MI up
  valZ[6] = maxMId - ((maxMId-minMId) * dkKondisi[4]);// VL x VH = MI down

  //#### derajat keanggotaan untuk keluaran VS ###////     error ==== deltaError
  valZ[7] = maxVS - ((maxVS-minVS) * dkKondisi[5]);// LW x VL = VS

  //#### derajat keanggotaan untuk keluaran SW ###////     error ==== deltaError
  valZ[8] = maxSWu - ((maxSWu-minSWu) * dkKondisi[6]);// LW x LW = SW up
  valZ[9] = maxSWu - ((maxSWd-minSWd) * dkKondisi[6]);// LW x LW = SW down

  //#### derajat keanggotaan untuk keluaran SW ###////     error ==== deltaError
  valZ[10] = maxSWu - ((maxSWu-minSWu) * dkKondisi[7]);// LW x MD = SW up
  valZ[11] = maxSWu - ((maxSWd-minSWd) * dkKondisi[7]);// LW x MD = SW down

  //#### derajat keanggotaan untuk keluaran MI ###////     error ==== deltaError
  valZ[12] = maxMIu - ((maxMIu-minMIu) * dkKondisi[8]);// LW x HI = MI up
  valZ[13] = maxMId - ((maxMId-minMId) * dkKondisi[8]);// LW x HI = MI down

  //#### derajat keanggotaan untuk keluaran MI ###////     error ==== deltaError
  valZ[14] = maxFAu - ((maxFAu-minFAu) * dkKondisi[9]);// LW x VH = FA up
  valZ[15] = maxFAd - ((maxFAd-minFAd) * dkKondisi[9]);// LW x VH = FA down

  //#### derajat keanggotaan untuk keluaran VS ###////     error ==== deltaError
  valZ[16] = maxVS - ((maxVS-minVS) * dkKondisi[10]);// MD x VL = VS

  //#### derajat keanggotaan untuk keluaran SW ###////     error ==== deltaError
  valZ[17] = maxSWu - ((maxSWu-minSWu) * dkKondisi[11]);// MD x LW = SW up
  valZ[18] = maxSWu - ((maxSWd-minSWd) * dkKondisi[11]);// MD x LW = SW down

  //#### derajat keanggotaan untuk keluaran MI ###////     error ==== deltaError
  valZ[19] = maxMIu - ((maxMIu-minMIu) * dkKondisi[12]);// MD x MD = MI up
  valZ[20] = maxMId - ((maxMId-minMId) * dkKondisi[12]);// MD x MD = MI down

  //#### derajat keanggotaan untuk keluaran MI ###////     error ==== deltaError
  valZ[21] = maxFAu - ((maxFAu-minFAu) * dkKondisi[13]);// MD x HI = FA up
  valZ[22] = maxFAd - ((maxFAd-minFAd) * dkKondisi[13]);// MD x HI = FA down

  //#### derajat keanggotaan untuk keluaran MI ###////     error ==== deltaError
  valZ[23] = maxVF - ((maxVF-minVF) * dkKondisi[14]);// MD x VH = VF

  //#### derajat keanggotaan untuk keluaran SW ###////     error ==== deltaError
  valZ[24] = maxSWu - ((maxSWu-minSWu) * dkKondisi[15]);// HI x VL = SW up
  valZ[25] = maxSWu - ((maxSWd-minSWd) * dkKondisi[15]);// HI x VL = SW down

  //#### derajat keanggotaan untuk keluaran MI ###////     error ==== deltaError
  valZ[26] = maxMIu - ((maxMIu-minMIu) * dkKondisi[16]);// HI x LW = MI up
  valZ[27] = maxMId - ((maxMId-minMId) * dkKondisi[16]);// HI x LW = MI down

  //#### derajat keanggotaan untuk keluaran MI ###////     error ==== deltaError
  valZ[28] = maxFAu - ((maxFAu-minFAu) * dkKondisi[17]);// HI x MD = FA up
  valZ[29] = maxFAd - ((maxFAd-minFAd) * dkKondisi[17]);// HI x MD = FA down

  //#### derajat keanggotaan untuk keluaran MI ###////     error ==== deltaError
  valZ[30] = maxFAu - ((maxFAu-minFAu) * dkKondisi[18]);// HI x HI = FA up
  valZ[31] = maxFAd - ((maxFAd-minFAd) * dkKondisi[18]);// HI x HI = FA down

  //#### derajat keanggotaan untuk keluaran MI ###////     error ==== deltaError
  valZ[32] = maxVF - ((maxVF-minVF) * dkKondisi[19]);// HI x VH = VF
  valZ[37] = maxVF - ((maxVF-minVF) * dkKondisi[22]);// VH x MD = VF
  valZ[38] = maxVF - ((maxVF-minVF) * dkKondisi[23]);// VH x LW = VF
  valZ[39] = maxVF - ((maxVF-minVF) * dkKondisi[24]);// VH x VH = VF

  //#### derajat keanggotaan untuk keluaran MI ###////     error ==== deltaError
  valZ[33] = maxMIu - ((maxMIu-minMIu) * dkKondisi[20]);// VH x VL = MI up
  valZ[34] = maxMId - ((maxMId-minMId) * dkKondisi[20]);// VH x VL = MI down

  //#### derajat keanggotaan untuk keluaran MI ###////     error ==== deltaError
  valZ[35] = maxFAu - ((maxFAu-minFAu) * dkKondisi[21]);// VH x LW = FA up
  valZ[36] = maxFAd - ((maxFAd-minFAd) * dkKondisi[21]);// VH x LW = FA down
 
  /*
  Serial.print(valZ[0],1);Serial.print(" ");
  Serial.print(valZ[1],1);Serial.print(" ");
  Serial.print(valZ[2],1);Serial.print(" ");
  Serial.print(valZ[3],1);Serial.print(" ");
  Serial.print(valZ[4],1);Serial.print(" ");
  Serial.print(valZ[5],1);Serial.print(" ");
  Serial.print(valZ[6],1);Serial.print(" ");
  Serial.print(valZ[7],1);Serial.print(" ");
  Serial.print(valZ[8],1);Serial.print(" ");Serial.println();
  */

  //valMillis++;
  //delay(2000);
  
  
}
//## function defuzzyfikasi 
void defuzzyfikasi()
{
  timeControl = ((dkKondisi[0]*valZ[0]) + (dkKondisi[1]*valZ[1]) + (dkKondisi[2]*valZ[2]) +
                (dkKondisi[3]*valZ[3]) + (dkKondisi[3]*valZ[4]) + (dkKondisi[4]*valZ[5]) + (dkKondisi[4]*valZ[6]) +
                (dkKondisi[5]*valZ[7]) + (dkKondisi[6]*valZ[8]) + (dkKondisi[6]*valZ[9]) + (dkKondisi[7]*valZ[10]) + (dkKondisi[7]*valZ[11]) +
                (dkKondisi[8]*valZ[12]) + (dkKondisi[8]*valZ[13]) + (dkKondisi[9]*valZ[14]) + (dkKondisi[9]*valZ[15]) +
                (dkKondisi[10]*valZ[16]) + (dkKondisi[11]*valZ[17]) + (dkKondisi[11]*valZ[18]) + (dkKondisi[12]*valZ[19]) + (dkKondisi[12]*valZ[20]) +
                (dkKondisi[13]*valZ[21]) + (dkKondisi[13]*valZ[22]) + (dkKondisi[14]*valZ[23]) + (dkKondisi[15]*valZ[24]) + (dkKondisi[15]*valZ[25]) +
                (dkKondisi[16]*valZ[26]) + (dkKondisi[16]*valZ[27]) + (dkKondisi[17]*valZ[28]) + (dkKondisi[17]*valZ[29]) + 
                (dkKondisi[18]*valZ[30]) + (dkKondisi[18]*valZ[31]) + (dkKondisi[19]*valZ[32]) + (dkKondisi[20]*valZ[33]) + (dkKondisi[20]*valZ[34]) +
                (dkKondisi[21]*valZ[35]) + (dkKondisi[21]*valZ[36]) + (dkKondisi[22]*valZ[37]) + (dkKondisi[23]*valZ[38]) +(dkKondisi[24]*valZ[39])
                ) / 
                (dkKondisi[0] + dkKondisi[1] + dkKondisi[2] 
                 + dkKondisi[3] + dkKondisi[3] + dkKondisi[4] + dkKondisi[4]
                 + dkKondisi[5] + dkKondisi[6] + dkKondisi[6] + dkKondisi[7] + dkKondisi[7]
                 + dkKondisi[8] + dkKondisi[8] + dkKondisi[9] + dkKondisi[9]
                 + dkKondisi[10] + dkKondisi[11] + dkKondisi[11] + dkKondisi[12] + dkKondisi[12]
                 + dkKondisi[13] + dkKondisi[13] + dkKondisi[14] + dkKondisi[15] + dkKondisi[15]
                 + dkKondisi[16] + dkKondisi[16] + dkKondisi[17] + dkKondisi[17]
                 + dkKondisi[18] + dkKondisi[18] + dkKondisi[19] + dkKondisi[20] + dkKondisi[20]
                 + dkKondisi[21] + dkKondisi[21] + dkKondisi[22] + dkKondisi[23] + dkKondisi[24]
                );

                //lcd.setCursor(0,0);
                //lcd.print(timeControl,1);
                //lcd.print(" ");
                //lcd.print(error,1);
  //Serial.print(" hasil defuzzy: ");Serial.println(timeControl,1);
}
void loop() {
  millis();
awal:

  if(Serial.available())
  {
    char dataSer=Serial.read();
    if(dataSer=='a')
    {
      lcd.clear(); goto startFuzzy; millisNow = millis();
    }
  }
  lcd.setCursor(posCursor, 0);
  lcd.write(byte(0));

  if(digitalRead(myButton[0]) == LOW)
  {
    delay(250);
    lcd.clear();
    posCursor++;
  }
  if(digitalRead(myButton[4]) == LOW)
  {
    delay(250);
    lcd.clear();
    posCursor--;
  }
  if(posCursor<6)
  {
    posCursor = 10;
  }
  if(posCursor>10)
  {
    posCursor = 6;
  }
  lcd.setCursor(0,0);
  lcd.print("Menu: ");
  switch(posCursor)
  {
    case 6: lcd.setCursor(0,1); lcd.print("ON/OFF"); if(digitalRead(myButton[2]) == LOW) {delay(250); lcd.clear(); goto startONOFF;}break;
    case 7: lcd.setCursor(0,1); lcd.print("Fuzzy"); if(digitalRead(myButton[2]) == LOW) {delay(250); lcd.clear(); goto startFuzzy; millisNow = millis();}break;
    case 8: lcd.setCursor(0,1); lcd.print("Set Point"); if(digitalRead(myButton[2]) == LOW) {delay(250); lcd.clear(); valPoint = 0; goto setPoint;}break;
    case 9: lcd.setCursor(0,1); lcd.print("Max Suhu"); if(digitalRead(myButton[2]) == LOW) {delay(250); lcd.clear(); valPoint = 0; goto maxSuhu;}break;
    case 10: lcd.setCursor(0,1); lcd.print("Min Suhu"); if(digitalRead(myButton[2]) == LOW) {delay(250); lcd.clear(); valPoint = 0; goto minSuhu;}break;
  }
  
goto awal;

startONOFF:
  minLm = EEPROM.read(2);
  maxLm = EEPROM.read(1);
  readLMSens(true);
  if(readLM < minLm)
  {
    lcd.setCursor(0,1);
    lcd.print("ON ");
    digitalWrite(A1, HIGH);
  }
  if(readLM > maxLm)
  {
    lcd.setCursor(0,1);
    lcd.print("OFF");
    digitalWrite(A1, LOW);
  }
  if(digitalRead(myButton[2]) == LOW)
  {
    delay(250);
    lcd.clear();
    digitalWrite(A1, LOW);
    goto awal;
  }
goto startONOFF;
startFuzzy:
  fuzzyfikasi();

  //lcd.setCursor(0,1);
  //lcd.print(valLm, 1);
  //lcd.print(" ");lcd.print(error,1); lcd.print(" "); lcd.print(lastError,1);
  
  ruleBase();
  defuzzyfikasi();

  if(countZero == 2)
  {
    if(!firstTime)
    {
      if(timeControl <= 0)
      
      {
        digitalWrite(A1, LOW);
      }
      else
      {
        digitalWrite(A1, HIGH);
        delayMicroseconds(timeControl*15);//8500 // 4250
        firstTime = true;
      }
    }
    else
    {
      digitalWrite(A1, LOW);
    }
  }
  else
  {
    firstTime = false;
    countZero = 2;
  }
  
  //lcd.setCursor(0,1);
  //lcd.print(humidity,1);
  //lcd.print(" ");
  //lcd.print(timeControl,1);
  
  zeroVariable();
  
  if(digitalRead(myButton[2]) == LOW)
  {
    delay(250);
    lcd.clear();
    digitalWrite(A1, LOW);
    goto awal;
  }
  
goto startFuzzy;
setPoint:
  valPoint = EEPROM.read(0);
  lcd.setCursor(0,0);
  lcd.print("Set Point: ");
  lcd.setCursor(0,1);
  sprintf(lcdBuff, "%3d", valPoint);
  lcd.print(lcdBuff);
  if(digitalRead(myButton[1]) == LOW)
  {
    delay(250);
    lcd.clear();
    valPoint--;
  }
  if(digitalRead(myButton[3]) == LOW)
  {
    delay(250);
    lcd.clear();
    valPoint++;
  }
  
  if(digitalRead(myButton[0]) == LOW)
  {
    EEPROM.write(0, valPoint);
    delay(250);
    lcd.clear();
    posCursor = 6;
    goto awal;
  }
  if(digitalRead(myButton[4]) == LOW)
  {
    EEPROM.write(0, valPoint);
    delay(250);
    lcd.clear();
    posCursor = 6;
    goto awal;
  }
  EEPROM.write(0, valPoint);
goto setPoint;
maxSuhu:
  valPoint = EEPROM.read(1);
  lcd.setCursor(0,0);
  lcd.print("Max Suhu: ");
  lcd.setCursor(0,1);
  sprintf(lcdBuff, "%3d", valPoint);
  lcd.print(lcdBuff);
  if(digitalRead(myButton[0]) == LOW)
  {
    delay(250);
    lcd.clear();
    posCursor = 6;
    goto awal;
  }
  if(digitalRead(myButton[4]) == LOW)
  {
    delay(250);
    lcd.clear();
    posCursor = 6;
    goto awal;
  }
  if(digitalRead(myButton[1]) == LOW)
  {
     EEPROM.write(1, valPoint);
    delay(250);
    lcd.clear();
    valPoint--;
  }
  if(digitalRead(myButton[3]) == LOW)
  {
     EEPROM.write(1, valPoint);
    delay(250);
    lcd.clear();
    valPoint++;
  }
 
  EEPROM.write(1, valPoint);
goto maxSuhu;
minSuhu:
  valPoint = EEPROM.read(2);
  lcd.setCursor(0,0);
  lcd.print("Min Suhu: ");
  lcd.setCursor(0,1);
  sprintf(lcdBuff, "%3d", valPoint);
  lcd.print(lcdBuff);
  if(digitalRead(myButton[0]) == LOW)
  {
    delay(250);
    lcd.clear();
    posCursor = 6;
    goto awal;
  }
  if(digitalRead(myButton[4]) == LOW)
  {
    delay(250);
    lcd.clear();
    posCursor = 6;
    goto awal;
  }
  if(digitalRead(myButton[1]) == LOW)
  {
     EEPROM.write(2, valPoint);
    delay(250);
    lcd.clear();
    valPoint--;
  }
  if(digitalRead(myButton[3]) == LOW)
  {
     EEPROM.write(2, valPoint);
    delay(250);
    lcd.clear();
    valPoint++;
  }
 
  EEPROM.write(2, valPoint);
goto minSuhu;
  
  /*
  int outputFuzzy = 128;
  if(countZero != 0)
  {
    digitalWrite(A1, HIGH);
    delay(outputFuzzy);
    digitalWrite(A1, LOW);
    countZero = 0;
  }
  */
  /*
  while(1)
  {
    fuzzyfikasi();

    lcd.setCursor(0,0);
    lcd.print(valLm, 1);
    lcd.print(" ");lcd.print(error,1); lcd.print(" "); lcd.print(lastError,1);
    
    ruleBase();
    defuzzyfikasi();

    lcd.setCursor(0,1);
    lcd.print(timeControl,1);
    
    zeroVariable();
  }
  */

}

