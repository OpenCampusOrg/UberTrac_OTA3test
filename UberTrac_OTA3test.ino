// Solar Geyser Controller with ESP32 using Temperature and Light LDR.
// 100k Ohm resistors used to reduce ADC voltage to below 1V, this lowers the
// amps through Thermistor so power ON/OFF multiplexer is not required.
// by James Trace Nov2018, update Mar2019.
const char* upDated = "2019/06/10"; //version

#define DoBUG // comment out to disable Serial
#define DoOTA // comment out to disable OTA 192.168.0.120

#ifdef DoOTA
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#endif
#include <WiFi.h>
#include <WiFiClient.h>
#include <ThingSpeak.h>
#include <ThingsBoard.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <WiFiUdp.h>

#define DoSEND // comment out to disable Sending to ThingSpeak and ThingsBoard
// for ThingsBoard
#define token "A4I2hkDVaUO13s8HPeVn"// UberTrac Test, "ujkNqsZpmmEPABGvFA79" // Main House Solar Geyser
#define thingsboardServer "ec2-35-175-242-204.compute-1.amazonaws.com"   // "demo.thingsboard.io" // gateway.connection.port

// for ThingSpeak
unsigned long myChannelNumber = 165321; // Test, 700467; // entre your channel's Number
const char * myWriteAPIKey = "141K163R0MBQ1GMI"; // Test, "BIDVZY1NGF76ZWK8"; // enter your channel's Write API Key

const char* host = "esp32"; // name
const char* ssid = "NETGEAR"; const char* pass = "mel0d1cda1sy";

// Setup ESP32 variables
#define Vcc 3.329
#define Vmax 3.3
#define ADCmax 4095
#define GPIO0 0
#define S0 25 // control Multiplexer S0
#define S1 26 // control Multiplexer S1
#define S2 27 // control Multiplexer S2
#define E 14 // ON OFF Multiplexer E
#define LED 2 // LED pin
#define LDRPOWER 15 // Power for LDR
#define PUMP 16 // Pump relay control Normally ON
#define ELEMENT 17 // Element relay control Normally Off
#define VOLT 32 // Measure Panel voltage
#define TEMPS 36 // ADC0 for thermistors
#define LDR 39 // ADC3 for LDR

// kOhm, Resistance used in voltage divider calculation. Max 40mA for output pins.
//Resistance with Thermistors calibrated from 0degC to 100degC, worst case.
// 100k Thermistor, 0.4-2.8V  I=V/(Rref+R) = 3.3/(47k+6975) = 0.06mA P=VI, 0.2mW each // Ah/day = Ix24h = 4.8mAh/day // coincell @240mAh approx 50 days life
// 47k ADC 534 = 100*C, ADC 3624 = 0*C, 1ADC res 0.08*C@100*C
// 10k Thermistor, 0.4-2.9V  I=V/(Rref+R) = 3.3/(4.7k+668) = 0.61mA 2mW each // Ah/day = Ix24h = 49mAh/day // coincell @240mAh approx 4.9 days life
// 4.7k ADC 510 = 100*C, ADC 3580 = 0*C, 1ADC res 0.08*C@100*C
// ESP32 ADC not accurate from 0 to 0.15V and 3.1 to 3.3V
#define R0 46.3 //k 46.3 - use to calibrate temperatures
#define R1 1.002 //k 0.998 for LDR use in upper part of divider (turn ON with GPIO to save power)
// 0.18-2.8V  I=V/(Rref+R) = 3.3/(1000+59) = 3.1mA each // Ah/day = Ix24h = 75mAh/day // coincell @240mAh approx 3.2 days life
// 1k ADC 230 = 5100Lux, ADC 3500 = 13Lux, 1ADC res 30Lux@5000Lux
#define R2 100.0 // Solar Panel voltage divider upper
#define R3 16.622 // 22+68=16.622 Solar Panel voltage divider lower

/*   #define ConsA 3.354016E-03 // 1.1335111E-03 // NB Check constants for Thermister used in function
   #define ConsB 2.569850E-04 // 2.3317823E-04
   #define ConsC 2.620131E-06 // 9.3245863E-08
   #define ConsD 6.383091E-08 //*/
   #define B 4085.0 // Thermistor Beta value
   #define Rref 100.0 //k Thermistor ref Resistance at ref temp ºC
   #define To 25.0 //ºC Thermistor ref temperature 

#define powerDelay 30 // 30, Sensor Power ON delay ms
#define forDelay 1000 // 5000, powerDelay used 16 times in for loop *nn
#define loopDelay 550 // 4150, powerDelay used 8+8 times (change for readA0 function)
// set loopDelay for total of 120sec
/* Delay Timming in main loop when program took 58.45sec to loop on average over 1hr
 * inputs nn = 10 iterations, powerDelay 5, forDelay 4760, loopDelay 6000.
 * 1000 for Wifi check, est program 10% = 300
 * 5*16=(80+4760)*10 = 48400 in for loop, est program 60% = 1800
 * 5*(8+8)=80+6000 = 6080 in calculations, est program 30% = 870
 * Total delays = 55480 leaving program useing 2970 to run.
 * 
 * New calculations to run every 120sec Sept18, Staff solar.
 * inputs nn = 20 iterations, powerDelay 10, forDelay 4760, loopDelay 6000.
 * 1000 for Wifi check, est program 10% = 300
 * 30*16=(480+5000)*20 = 109600 in for loop, est program 60% = 1800*2 = 3600
 * 30*(8+8)=480+4150 = 4630 in calculations, est program 30% = 870
 * Total delays = 115230 leaving program useing 4770 to run.
 * Total 120000 or 0 short. Use 336 powerDelays.
 */

const byte nn = 20; // 20, Averaging - number of iterations in forLoop
int rssi;
byte k = 0, tempOff; // keeps track of Wifi failures to reset ESP after 1hr
boolean elementState = LOW, pumpState = LOW, lastpumpState  = LOW, holiday;
unsigned long elementMillis, elementPeriod, send_millis=0, for_millis=0;
const int luxLimit = 100; // Lux treashold when element will turn ON
int luxPrevious = 0;
//char ipchar;
String chipIP;
// Set to true if application is subscribed for the RPC messages.
bool subscribed = false;
// We assume that all GPIOs are LOW
//boolean gpioState[] = {false, false}; //,  pumpState = false; // array for pin state
//int knobValue = 0;
byte l=0, m=0; // used for collection and sending timing

// main veriables
float adcA0=0, adcA1=0, adcA2=0, adcA3=0, adcA4=0, adcA5=0, adcA6=0, adcA7=0, adcA8=0, adcA9=0;
float tempA0=0, tempA1=0, tempA2=0, tempA3=0, tempA4=0, tempA5=0, tempA6=0, tempA7=0, luxA8 = 0;
float voltA9=0, intTemp=0, hours=0, tempAvg=0; // pumpState=0,
int ADCA0=0, ADCA3=0, ADCA4=0, ADCA6=0;

/*// Timer variables
const char* Days[4] = { "Weekdays", "Weekends", "Friday", "Everyday" };
const char* timeON[4] = { "5:00", "6:00", "14:30", "Sunset" };
const char* temp[4] = { "50", "45", "40", "55" };
const char* period[4] = { "1.0", "1.0", "0.5", "2.0" };
byte l = 0; //*/

// Internal Temperature sensor setup
uint8_t temprature_sens_read();


// Initialize Wifi Client.
int status = WL_IDLE_STATUS;
WiFiClient wifiClient;
WiFiClient speakClient;

// Initialize ThingsBoard instance
ThingsBoard tb(wifiClient);


// For OTA programming
#ifdef DoOTA
WebServer server(80); 
// * Login page

const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32Test Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='UberTest' && form.pwd.value=='jcgt')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";
 #endif

// NTP Servers:
static const char ntpServerName[] = "us.pool.ntp.org";
//static const char ntpServerName[] = "time.nist.gov";
//static const char ntpServerName[] = "time-a.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-b.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-c.timefreq.bldrdoc.gov";

const int timeZone = 2;     // Central African Time

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

time_t getNtpTime();
void digitalClockDisplay();
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);

time_t prevDisplay = 0; // when the digital clock was displayed


// Helper macro to calculate array size
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

// Processes functions for RPC callbacks


RPC_Response processSetElement(const RPC_Data &data) {
  Serial.println("Received the set Element RPC method");
  bool enabled = data; //["params"];
    Serial.print("Set Element to state: ");
    Serial.println(enabled);
    if (enabled == HIGH) {
      tempOff = 25; elementPeriod = 1.5*60*1000UL; //0.5hr
      elementMillis = millis();
      Serial.print("elementMillis="); Serial.println(elementMillis); }
      digitalWrite(ELEMENT, enabled); elementState = digitalRead(ELEMENT);
  return RPC_Response(NULL, (bool)data); }
  
RPC_Response processGetElement(const RPC_Data &data) {
  Serial.println("Received the get Element request");
  bool enabled = digitalRead(ELEMENT);
    Serial.print("Get Element state: ");
    Serial.println(enabled);
  return RPC_Response(NULL, enabled); }

RPC_Response processSetHoliday(const RPC_Data &data) {
  Serial.println("Received the set Holiday RPC method");
  bool enabled = data; //["params"];
    Serial.print("Set Holiday to state ");
    Serial.println(enabled);
    holiday = enabled;
  return RPC_Response(NULL, (bool)data);
}

RPC_Response processGetHoliday(const RPC_Data &data) {
  Serial.println("Received the get Holiday request");
    Serial.print("Get Holiday state: ");
    Serial.println(holiday);
  return RPC_Response(NULL, holiday);
}

// RPC handlers
RPC_Callback callbacks[] = {
  { "setElement",      processSetElement },
  { "getElement",      processGetElement },
  { "setHoliday",      processSetHoliday },
  { "getHoliday",      processGetHoliday },
};

void setup() { // #############################################################

#ifdef DoBUG
  Serial.begin(115200); // enable DoBUG serial
/*    Serial.println();
    Serial.print("FreeHeap size:  "); Serial.println(ESP.getFreeHeap());
    Serial.print("Chip Hz:  "); Serial.println(ESP.getFlashChipSpeed()); //*/
//    Serial.print("Vcc:  "); Serial.print(ESP.getVcc()/1000); Serial.println("V\n"); // Set ADC Mode
#endif
  
// ESP32 pinouts
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(E, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(TEMPS, INPUT); // A0 ADC1 CH0 GPIO36 VP, Input only
//  pinMode(37, INPUT); // A1 ADC1 CH1 GPIO37 (not on 30 pin board??), Input only
//  pinMode(38, INPUT); // A2 ADC1 CH2 GPIO38 (not on 30 pin board??), Input only
  pinMode(LDR, INPUT); // A3 ADC1 CH3 GPIO39 VN, Input only
  pinMode(VOLT, INPUT); // A4 ADC1 CH4 GPIO32
//  pinMode(33, INPUT); // A5 ADC1 CH5 GPIO33
//  pinMode(34, INPUT); // A6 ADC1 CH6 GPIO34, Input only (internal temp sensor??)
//  pinMode(35, INPUT); // A7 ADC1 CH7 GPIO35, Input only
//  pinMode(4, INPUT); // A10 ADC2 CH0 GPIO4
//  pinMode(0, INPUT); // A11 ADC2 CH1 GPIO0 - BOOT LOW do NOT use, Output only.
//  pinMode(2, INPUT); // A12 ADC2 CH2 GPIO2 - Boot leave floating
  pinMode(LDRPOWER, OUTPUT); // A13 ADC2 CH3 GPIO15 - turn on LDR
//  pinMode(13, INPUT); // A14 ADC2 CH4 GPIO13
//  pinMode(12, INPUT); // A15 ADC2 CH5 GPIO12
//  pinMode(14, INPUT); // A16 ADC2 CH6 GPIO14
//  pinMode(27, INPUT); // A17 ADC2 CH7 GPIO27
//  pinMode(25, INPUT); // A18 ADC2 CH8 GPIO25
//  pinMode(26, INPUT); // A19 ADC2 CH9 GPIO26
  pinMode(PUMP, OUTPUT); // GPIO16 - Control Circulation Pump
  pinMode(ELEMENT, OUTPUT); // GPIO17 - Control Geyser Element
  // Pins 6 to 11 are for SPI only

// Connect to WiFi
  WiFi.begin(ssid, pass); //Connect to a single Access Point.
    delay(1000); //*/
  #ifdef DoBUG
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());
  #endif

  checkWiFi(); // Check for the connetion else Reset

//  chipIP  =  WiFi.localIP().toString().c_str();

  // once you are connected :
  #ifdef DoBUG
//  Serial.print("You're connected to the network");
    printCurrentSSID();
    printCurrentRSSI(); Serial.print("\t");
    printWifiIP();
    printWifiMAC();
  #endif

// Start OTA
#ifdef DoOTA
/*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    #ifdef DoBUG
    Serial.println("Error setting up MDNS responder!");
    #endif
    while (1) {
      delay(500);
    }
  }
  #ifdef DoBUG
  Serial.println("mDNS responder started");
  #endif
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      #ifdef DoBUG
      Serial.printf("Update: %s\n", upload.filename.c_str());
      #endif
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        #ifdef DoBUG
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        #endif
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin(); // End OTA
  #endif

  ThingSpeak.begin(speakClient);

// Get time for Element control
  Udp.begin(localPort);
  #ifdef DoBUG
  Serial.println("Starting UDP");
  Serial.println("waiting for sync");
  #endif
  setSyncProvider(getNtpTime);
  setSyncInterval(14400); // reset every 4 hrs, 4*60*60.
  delay(100);

// ThingsBoard Send once
#ifdef DoSEND
  if (!tb.connected()) { subscribed = false;
    if (!tb.connect(thingsboardServer, token)) {
      Serial.println("Failed to connect"); } }
  // Subscribe for RPC, if needed
  if (!subscribed) {
    Serial.println("Subscribing for RPC...");
    // Perform a subscription. All consequent data processing will happen in callbacks as denoted by callbacks[] array.
    if (!tb.RPC_Subscribe(callbacks, COUNT_OF(callbacks))) {
      Serial.println("Failed to subscribe for RPC"); }
    Serial.println("Subscribe done"); subscribed = true; }
      
    tb.sendTelemetryString("Dated", upDated); 
        String ipaddress = WiFi.localIP().toString();
        char ipchar[ipaddress.length()+1];
        ipaddress.toCharArray(ipchar,ipaddress.length()+1);
    tb.sendTelemetryString("chipIP", ipchar);
    tb.loop();
#endif //*/
  
// create the alarms, to trigger at specific times
//  Alarm.alarmRepeat(5,0,0, ElementOn5060);  // 5:00am every day ON
//  Alarm.alarmRepeat(5,0,0, ElementOn4560);  // 5:00am every day ON
      Alarm.alarmRepeat(dowMonday, 5,0,0, ElementOn5090);
      Alarm.alarmRepeat(dowTuesday, 5,0,0, ElementOn5090);
      Alarm.alarmRepeat(dowWednesday, 5,0,0, ElementOn5090);
      Alarm.alarmRepeat(dowThursday, 5,0,0, ElementOn5090);
      Alarm.alarmRepeat(dowFriday, 5,0,0, ElementOn5090);
      Alarm.alarmRepeat(dowSaturday, 6,0,0, ElementOn5090);
      Alarm.alarmRepeat(dowSunday, 7,0,0, ElementOn5090);

//      Alarm.alarmRepeat(dowSunday, 18,52,0, ElementTest2504);
  Alarm.alarmRepeat(10,30,0, ElementOn4030);  // 2:30pm every day ON
  Alarm.alarmRepeat(12,30,0, ElementTest2504);
  Alarm.alarmRepeat(14,30,0, ElementOn4030);  // 2:30pm every day ON
//  Alarm.alarmRepeat(18,30,0, ElementOff);  // 6:30pm every day OFF
//  Alarm.alarmRepeat(dowTuesday,11,15,30,WeeklyAlarm);  // 8:30:30 every Saturday */

} // End Setup ############################################

// Control Element Functions
void ElementOn50120Sun() { tempOff = 50; elementPeriod = 2*60*60*1000UL; // 2hrs
  if ( tempAvg < tempOff & holiday == LOW) {  elementMillis = millis();
      digitalWrite(ELEMENT, HIGH); elementState = digitalRead(ELEMENT); } }

void ElementOn55120Sun() { tempOff = 55; elementPeriod = 2*60*60*1000UL; // 2hrs
  if ( tempAvg < tempOff & holiday == LOW) {  elementMillis = millis();
      digitalWrite(ELEMENT, HIGH); elementState = digitalRead(ELEMENT); } }

void ElementOn5060() { tempOff = 50; elementPeriod = 60*60*1000UL; // 1hr
  if ( tempAvg < tempOff & holiday == LOW) {  elementMillis = millis();
      digitalWrite(ELEMENT, HIGH); elementState = digitalRead(ELEMENT); } }

void ElementOn5090() { tempOff = 50; elementPeriod = 90*60*1000UL; // 1hr
  if ( tempAvg < tempOff & holiday == LOW) {  elementMillis = millis();
      digitalWrite(ELEMENT, HIGH); elementState = digitalRead(ELEMENT); } }

void ElementOn4590() { tempOff = 45; elementPeriod = 90*60*1000UL; // 1hr
  if ( tempAvg < tempOff & holiday == LOW) {  elementMillis = millis();
      digitalWrite(ELEMENT, HIGH); elementState = digitalRead(ELEMENT); } }

void ElementOn4030() { tempOff = 40; elementPeriod = 30*60*1000UL; //0.5hr
  if ( tempAvg < tempOff & holiday == LOW) {  elementMillis = millis();
      digitalWrite(ELEMENT, HIGH); elementState = digitalRead(ELEMENT); } }

void ElementOff() { if (elementState) { 
      if (millis()-elementMillis>elementPeriod|tempAvg>tempOff|holiday==HIGH ) { 
      digitalWrite(ELEMENT, LOW); elementState = digitalRead(ELEMENT);
      Serial.println("ElementOff"); } } }

void ElementTest2504() { tempOff = 25; elementPeriod = 4*60*1000UL; //0.5hr
  if ( tempAvg < tempOff & holiday == LOW) {  elementMillis = millis();
      Serial.print("elementMillis="); Serial.println(elementMillis);
      digitalWrite(ELEMENT, HIGH); elementState = digitalRead(ELEMENT); } }
// End Alarms */
  
void loop() { //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
#ifdef DoOTA
  server.handleClient(); // OTA server
#endif

Alarm.delay(10);
if (timeStatus() == timeNotSet) { setSyncProvider(getNtpTime); setSyncInterval(14400); } // get time if not set

#ifdef DoBUG
//    Serial.println("DoBUG: Loop");
//  Serial.print("MAC ID: "); Serial.print(stringMAC); Serial.print(" \t\n");
#endif

if (millis() > send_millis + loopDelay & m == 0) { m=1;
    adcA0=0; adcA1=0; adcA2=0; adcA3=0; adcA4=0; adcA5=0; adcA6=0; adcA7=0; adcA8=0; adcA9=0;
    tempA0=0; tempA1=0; tempA2=0; tempA3=0; tempA4=0; tempA5=0; tempA6=0; tempA7=0; luxA8 = 0;
    voltA9=0; intTemp=0; // pumpState=0,
    ADCA0=0; ADCA3=0; ADCA4=0; ADCA6=0;
    l=0;
    Serial.print("reset variables, ");
}

if (millis() > for_millis + forDelay & l<nn) {
    digitalWrite(LED, HIGH); // LED off
//    for (byte i=0; i<nn; i++) { //FOR Averaging Loop 
     
    digitalWrite(S0, LOW); // S0 Multi control
    digitalWrite(S1, LOW); // S1
    digitalWrite(S2, LOW); // S2  A0 ON
    digitalWrite(E, LOW); // Multiplexers ON
    adcA0 += readADC(TEMPS); // Mid Geyser Water Temp
//    Serial.print("DoBUG: adcA0 "); Serial.println(adcA0);

    digitalWrite(S0, HIGH); // S0  Multi control
    digitalWrite(S1, LOW); // S1
    digitalWrite(S2, LOW); // S2  A1 ON
    digitalWrite(E, LOW); // Multiplexers ON
    adcA1 += readADC(TEMPS); // Top Geyser Temp

    digitalWrite(S0, LOW); // S0  Multi control
    digitalWrite(S1, HIGH); // S1
    digitalWrite(S2, LOW); // S2  A2 ON
    digitalWrite(E, LOW); // Multiplexers ON
    adcA2 += readADC(TEMPS); // Bottom Geyser Temp

    digitalWrite(S0, HIGH); // S0  Multi control
    digitalWrite(S1, HIGH); // S1
    digitalWrite(S2, LOW); // S2  A3 ON
    digitalWrite(E, LOW); // Multiplexers ON
    adcA3 += readADC(TEMPS); //  Outlet to Solar Temp

    digitalWrite(S0, LOW); // S0  Multi control
    digitalWrite(S1, LOW); // S1
    digitalWrite(S2, HIGH); // S2  A4 ON
    digitalWrite(E, LOW); // Multiplexers ON
    adcA4 += readADC(TEMPS); // Inlet from Solar Temp

    digitalWrite(S0, HIGH); // S0  Multi control
    digitalWrite(S1, LOW); // S1
    digitalWrite(S2, HIGH); // S2  A5 ON
    digitalWrite(E, LOW); // Multiplexers ON
    adcA5 += readADC(TEMPS); // Roof Air Temp

    digitalWrite(S0, LOW); // S0  Multi control
    digitalWrite(S1, HIGH); // S1
    digitalWrite(S2, HIGH); // S2  A6 ON
    digitalWrite(E, LOW); // Multiplexers ON
    adcA6 += readADC(TEMPS); // Manifold Temp

    digitalWrite(S0, HIGH); // S0  Multi control
    digitalWrite(S1, HIGH); // S1
    digitalWrite(S2, HIGH); // S2  A6 ON
    digitalWrite(E, LOW); // Multiplexers ON
    adcA7 += readADC(TEMPS); // Outside Air Temp 

    digitalWrite(LDRPOWER, HIGH); // Turn ON LDR circit
    adcA8 += readADC(LDR); // Light Radiation Lux
    digitalWrite(LDRPOWER, LOW); // Turn ON LDR circit

    adcA9 += analogRead(VOLT); // get Panel Voltage

    intTemp += (temprature_sens_read()-32)/1.8; // Measure internal temp of ESP32

/*    ADCA0 = adcA0 / (l+1); // Mid looks at a running average
    ADCA3 = adcA3 / (l+1); // Out
    ADCA4 = adcA4 / (l+1); // In
    ADCA6 = adcA6 / (l+1); // Manifold

     // Control Circulation Pump 
      if (ADCA6 < ADCA0 - 50) { // 50 -1.5*C; pump ON when Manifold A6 is hotter than Mid A0 + 1.5*C
        digitalWrite(PUMP, LOW); // Fail safe Pump ON
        pumpState += !digitalRead(PUMP); lastpumpState = LOW;} // Pump On, pump will not run at below 800Lux
      else if (ADCA6 > ADCA3 | ADCA6 > ADCA4 | ADCA3 < ADCA4-30 | ADCA4 < 850) { // pump OFF if Manifold A6 < OUT A3 or < IN A4 or OUT > IN +1*C or IN > 80*C
        digitalWrite(PUMP, HIGH); // Pump Off  
        pumpState += !digitalRead(PUMP); lastpumpState = HIGH; } 
      else { digitalWrite(PUMP, lastpumpState); // pump same, maintain last pump state
        pumpState += !digitalRead(PUMP); } // keep checking and report actual state */
      
      
    //OTAdelay (forDelay);
      
      
    for_millis = millis();
//    Serial.print(l); Serial.print("mloop, "); 
    l++; m=2;
    digitalWrite(LED, LOW); // LED on
    } // end if loop
//    } // End FOR Averaging loop
  
    
if (l==nn & m==2) { Serial.println(""); //Serial.print("l="); Serial.println(l);
    adcA0 = adcA0 / nn; // Mid temp
    tempA0 = ThermistorGlass100k(analogCalibration(adcA0), R0);
    
    adcA1 = adcA1 / nn; // Top temp
    tempA1 = ThermistorGlass100k(analogCalibration(adcA1), R0);
    
    adcA2 = adcA2 / nn; // Bot temp
    tempA2 = ThermistorGlass100k(analogCalibration(adcA2), R0);

    adcA3 = adcA3 / nn; // sOUTlet temp
    tempA3 = ThermistorGlass100k(analogCalibration(adcA3), R0);
    
    adcA4 = adcA4 / nn; // sINlet temp
    tempA4 = ThermistorGlass100k(analogCalibration(adcA4), R0);

    adcA5 = adcA5 / nn; // Roof temp
    tempA5 = ThermistorGlass100k(analogCalibration(adcA5), R0);

    adcA6 = adcA6 / nn; // Manifold
    tempA6 = ThermistorGlass100k(analogCalibration(adcA6), R0);

    adcA7 = adcA7 / nn; // Out Air
    tempA7 = ThermistorGlass100k(analogCalibration(adcA7), R0);

    adcA8 = adcA8 / nn; // Light Lux
    luxA8 = ldrLux10k(analogCalibration(adcA8), R1); 

    adcA9 = adcA9 / nn; // Panel V
//    pumpState = pumpState / nn;
    voltA9 = solarV(analogCalibration(adcA9), R2, R3);

    intTemp = intTemp / nn;

    tempAvg = (tempA0*3+tempA1*3+tempA2)/7; // Average temp 4 2 1

     // Control Circulation Pump 
      if (tempA6 > tempA0 + 3) { // 50 -1.5*C; pump ON when Manifold A6 is hotter than Mid A0 + 1.5*C
        digitalWrite(PUMP, LOW); lastpumpState = LOW;} // Pump ON, Fail safe 
      else if (tempA6 < tempA3 | tempA6 < tempA4 | tempA3 > tempA4 - 1 | tempA4 > 80) { // pump OFF if Manifold A6 < OUT A3 or < IN A4 or OUT > IN -1*C or IN > 80*C
        digitalWrite(PUMP, HIGH); lastpumpState = HIGH; } // Pump Off  
      else { digitalWrite(PUMP, lastpumpState); } // pump same, maintain last pump state
        
    // Turn on Element after sun goes down if avg temp less than 50ºC
    if (luxA8<luxLimit & luxA8<luxPrevious & luxPrevious>=luxLimit & elementState == LOW & tempAvg<55) { 
      ElementOn55120Sun(); }
    

#ifdef DoBUG
/*    Serial.print("adcA0 = "); Serial.print(adcA0); Serial.print("\t"); Serial.println(ADCA0);
    Serial.print("adcA1 = "); Serial.print(adcA1); Serial.print("\t"); Serial.println(ADCA1);
    Serial.print("adcA2 = "); Serial.print(adcA2); Serial.print("\t"); Serial.println(ADCA2);
    Serial.print("adcA3 = "); Serial.print(adcA3); Serial.print("\t"); Serial.println(ADCA3);
    Serial.print("adcA4 = "); Serial.print(adcA4); Serial.print("\t"); Serial.println(ADCA4);
    Serial.print("adcA5 = "); Serial.print(adcA5); Serial.print("\t"); Serial.println(ADCA5);
    Serial.print("adcA6 = "); Serial.print(adcA6); Serial.print("\t"); Serial.println(ADCA6);
    Serial.print("adcA7 = "); Serial.print(adcA7); Serial.print("\t"); Serial.println(ADCA7); //*/

/*    Serial.println();
    Serial.print("tempA0 = "); Serial.print(tempA0); Serial.println(" *C");
    Serial.print("tempA1 = "); Serial.print(tempA1); Serial.println(" *C");
    Serial.print("tempA2 = "); Serial.print(tempA2); Serial.println(" *C");
    Serial.print("tempA3 = "); Serial.print(tempA3); Serial.println(" *C");
    Serial.print("tempA4 = "); Serial.print(tempA4); Serial.println(" *C");
    Serial.print("tempA5 = "); Serial.print(tempA5); Serial.println(" *C");
    Serial.print("tempA6 = "); Serial.print(tempA6); Serial.println(" *C");
//    Serial.print("ldrA7 = "); Serial.print(luxA7,0); Serial.println(" Lux  ");
    Serial.print("intTemp = "); Serial.print(intTemp); Serial.println(" *C");
    Serial.print("voltA9 = "); Serial.print(voltA9); Serial.println(" V");
//    Serial.print(" deltaTime = "); Serial.print(deltaT,2); Serial.print(" ms  ");
    Serial.print("RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm "); //*/
#endif

    rssi = WiFi.RSSI(); // read RSSI
    hours = millis()/3600000.0;
      pumpState = !digitalRead(PUMP);  // keep checking and report actual state */
      elementState = digitalRead(ELEMENT);
      
    #ifdef DoBUG
    Serial.print(" millis="); Serial.print(millis());
    Serial.print(" luxA8="); Serial.print(luxA8);
    Serial.print(" luxPrevious="); Serial.print(luxPrevious);
    Serial.print(" elState="); Serial.print(elementState);
    Serial.print(" pumpState="); Serial.print(pumpState);
    Serial.print(" Holiday="); Serial.print(holiday);
    Serial.print(" hours="); Serial.println(hours);
    #endif
    luxPrevious = luxA8; // update last values

m = 3;
Serial.print("average loop, ");
} 

  ElementOff(); // Check to turn Element OFF

// Send DATA to Server
  checkWiFi(); // Check for the connetion else Reset

// ThingsBoard RPC
if (!tb.connected()) { subscribed = false;
    if (!tb.connect(thingsboardServer, token)) {
      Serial.println("Failed to connect"); } }
  // Subscribe for RPC, if needed
  if (!subscribed) {
    Serial.println("Subscribing for RPC...send");
    checkWiFi();
    // Perform a subscription. All consequent data processing will happen in callbacks as denoted by callbacks[] array.
    if (!tb.RPC_Subscribe(callbacks, COUNT_OF(callbacks))) {
      Serial.println("Failed to subscribe for RPC"); }
    Serial.println("Subscribe done"); subscribed = true; }

if (millis() > send_millis + loopDelay & m==3) {

  #ifdef DoSEND
    digitalWrite(LED, HIGH); //Turn ON indicator LED
// ESP32 to ThingSpeak Comunication
//    digitalWrite(4, HIGH);
  ThingSpeak.setField(1,tempA0); // Water Mid Temp
  ThingSpeak.setField(2,tempA1); // Hot pipe Top Geyser
  ThingSpeak.setField(3,tempA2); // Cold pipe Bottom Geyser
  ThingSpeak.setField(4,tempA3); // Solar pipe Warm Out Geyser
  ThingSpeak.setField(5,tempA4); // Solar pipe Hot In Geyser
  ThingSpeak.setField(6,tempA5); // Roof Temp
  ThingSpeak.setField(7,tempA6); // Manifold Temp
//  ThingSpeak.setField(8,tempA7); // Outside Air Temp
  ThingSpeak.setField(8,luxA8);  // LDR Light Lux
  // Write the fields that you've set all at once to ThingSpeak.
  ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey); 
  
/*  ThingSpeak.setField(1,adcA9); // Panel Voltage adc
//  ThingSpeak.setField(2,adcA3); // Out to solar adc
//  ThingSpeak.setField(3,adcA6); // Header adc
  ThingSpeak.writeFields(165321, "141K163R0MBQ1GMI"); // test channel */
    
    tb.sendTelemetryFloat("Mid temp", tempA0);
    tb.sendTelemetryFloat("Top temp", tempA1);
    tb.sendTelemetryFloat("Bot temp", tempA2);
    tb.sendTelemetryFloat("sOUTlet temp", tempA3);
    tb.sendTelemetryFloat("sINlet temp", tempA4);
    tb.sendTelemetryFloat("Roof temp", tempA5);
    tb.sendTelemetryFloat("Manifold", tempA6);
    tb.sendTelemetryFloat("Out Air", tempA7);
    tb.sendTelemetryFloat("Light Lux", luxA8);
    tb.sendTelemetryFloat("Chip temp", intTemp);
    tb.sendTelemetryFloat("Avg", tempAvg);
    tb.sendTelemetryFloat("Panel V", voltA9);
    tb.sendTelemetryFloat("Hours", hours);
    tb.sendTelemetryInt("rssi", rssi);
    tb.sendTelemetryInt("Pump", pumpState);
    tb.sendTelemetryInt("Element", elementState); // tb.sendTelemetryBool( )
    tb.sendTelemetryInt("min", minute());
    tb.sendTelemetryInt("Holiday", holiday);
/*    if (hours < 0.05) { tb.sendTelemetryString("Dated", upDated); 
        String ipaddress = WiFi.localIP().toString();
        char ipchar[ipaddress.length()+1];
        ipaddress.toCharArray(ipchar,ipaddress.length()+1);
    tb.sendTelemetryString("chipIP", ipchar); } //*/
//    tb.sendTelemetryFloat("adcA3", adcA3);
//    tb.sendTelemetryFloat("adcA6", adcA6);

/*  if ( l > 3 ) { l = 0;}
    tb.sendTelemetryString("Day", Days[l]);
    tb.sendTelemetryString("timeON", timeON[l]);
    tb.sendTelemetryString("tempSet", temp[l]);
    tb.sendTelemetryString("Period", period[l]); //*/
  tb.sendTelemetryFloat("temperature", random(10,40));

  tb.sendAttributeString("Timer1", upDated);
  
    
//    tb.loop();

    digitalWrite(LED, LOW);
#endif //*/

//  l++; // increase Timer index
Serial.println("send loop.");
m = 0; // reset averaging if loop to run
send_millis = millis(); } // end send loop

// OTAdelay(loopDelay);
      // Process messages
//      RPC_Response processGetElement(); // send Element State
      tb.loop();

  // Send additional data to test channel
/*  ThingSpeak.setField(2,adcA7); // Outside Air Temp
  ThingSpeak.setField(3,tempA7); // Outside Air Temp
  ThingSpeak.writeFields(myChannelNumber2, myWriteAPIKey2); //*/
  
if (hours > 24 & elementState == LOW & holiday == LOW) { ESP.restart(); } // reset every 1 days

} // end Loop #############################################################################

// Function - check for WiFi connetion to be established, max j sec then reset.
void checkWiFi() {
  int j = 0;
  while (WiFi.status() != WL_CONNECTED) { 
    j ++;
    Alarm.delay(1000);
    #ifdef DoBUG
    Serial.print(j); Serial.print(" ");
    #endif
      if (j > 14) { k ++; break; }
  } // end while
  if (k > 30) { ESP.restart(); }
} // End Function

#ifdef DoBUG
// Function - 
void printWifiIP() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.print(ip);
} // End Function

// Function - 
void printWifiMAC() {
  // print your MAC address:
    byte mac[6];  
    WiFi.macAddress(mac);
    Serial.print("\tMAC address: ");
    String stringMAC = String(mac[0],HEX);
    stringMAC += ":";
    stringMAC += String(mac[1],HEX);
    stringMAC += ":";
    stringMAC += String(mac[2],HEX);
    stringMAC += ":";
    stringMAC += String(mac[3],HEX);
    stringMAC += ":";
    stringMAC += String(mac[4],HEX);
    stringMAC += ":";
    stringMAC += String(mac[5],HEX);
    Serial.println(stringMAC);
} // End Function

// Function - 
void printCurrentSSID() {
  Serial.print(" SSID: "); Serial.println(WiFi.SSID());
} // End Function

// Function - 
void printCurrentRSSI() {
  Serial.print("signal strength (RSSI): "); Serial.print(WiFi.RSSI()); Serial.print("dBm ");
} // End Function
#endif

// Function - ReadA0 and use avg
float readADC(byte pin) {
      Alarm.delay(powerDelay);
  analogRead(pin);
  #ifdef DoBUG
//  Serial.print("DoBUG: pin "); Serial.println(pin);
  #endif
  unsigned long measurement = 0;
  for (int i = 0 ; i < 2047 ; i++) {
    measurement += analogRead(pin); }
    return measurement / 2047; // 5000 65C, 1000 67C, 100 69C, 4095 67C,
    digitalWrite(E, HIGH); // Multiplexers OFF
} // End Function

// Function - Performs a polynomial fit to linearize the awful ADC
double analogCalibration(double adcRaw) {
      double adcCalibrated; // from https://github.com/terryjmyers/analogReadNonBlocking
    //Note that these default values were the average of a couple ESP32's I had and linearized the 0-4095 12-bit raw counts.
    //They will most likely give you a useable linear response accurate to 100mV (over a 0-3.3V range, or 11dB attenuation)
    //However you can perform your own calibration with excel and a trendline to obtain the coefficients and therefore characterize each individual ESP32.  See spreadsheet provided in the library
    double X6 = 3.07344E-18, X5 = -3.28432E-14, X4 = 1.16038E-10;
    double X3 = -1.531075E-07, X2 = 0.0000357848, X = 1.06235, b = 180.189;   
      if (X6 != 0) adcCalibrated += X6 * pow(adcRaw, 6);
      if (X5 != 0) adcCalibrated += X5 * pow(adcRaw, 5);
      if (X4 != 0) adcCalibrated += X4 * pow(adcRaw, 4);
      if (X3 != 0) adcCalibrated += X3 * pow(adcRaw, 3);
      if (X2 != 0) adcCalibrated += X2 * pow(adcRaw, 2);
      if (X != 0) adcCalibrated += X * adcRaw;
      if (b != 0) adcCalibrated += b;
      return adcCalibrated; //*/
//      return adcRaw;
} // End Function

// Function - Convert ADC reading from 10k Thermistor to Temperature *C
float ThermistorGlass100k(float ADCvalue, float Rdiv) {
      Alarm.delay(powerDelay);
       float Vout = ADCvalue*Vmax/ADCmax;
//   float thermR = Rdiv*(Vcc/Vout-1); // Thermistor in upper part of diver
   float thermR = Rdiv/(Vcc/Vout-1); // Thermistor in lower part of divider - choose one!
   float tempC = 1/(1/(To+273.15)+log(thermR/Rref)/B) - 273.15; // When using Beta value for Thermistors
//   float tempC = 1/(ConsA+(ConsB*log(thermR))+(ConsC*pow(log(thermR),3)))-273.15;
//   float tempC = 1/(ConsA+(ConsB*log(thermR/Rref))+(ConsC*pow(log(thermR/Rref),2))+(ConsD*pow(log(thermR/Rref),3)))-273.15; // use for NTCLE100E3 3977 B25/85
   Alarm.delay(powerDelay);
   return tempC;
}  // End Function

// Function - Convert ADC reading from LDR to Lux
float ldrLux10k(float ADCvalue, float Rdiv) { 
      Alarm.delay(powerDelay);
    float Vout = ADCvalue*Vmax/ADCmax;
//    float ldrR = Rdiv*(Vcc/Vout-1); // LRD in upper part of divider. Rdiv*((Vcc-Vout)/Vout)
    float ldrR = Rdiv/(Vcc/Vout-1); // LDR in lower part of divider - choose one! Rdiv*Vout/(Vcc-Vout)
    float ldrLux = 260/pow(ldrR,1.3); // Lux=500/Rldr^1.3 changed to 130 to match Solar geyser
    Alarm.delay(powerDelay);
    return ldrLux;
} // End Function

float solarV(float ADCvalue, float Rup, float Rlow) { 
      Alarm.delay(powerDelay);
    float Vout = ADCvalue*Vmax/ADCmax;
    float Vsol = Rup*Vout/Rlow+Vout; // max 18V
    Alarm.delay(powerDelay);
    return Vsol;
} // End Function


/*-------- NTP code ----------*/
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}


/*// Function - non-blocking delay while checking OTA update
void OTAdelay (unsigned int mstime) {
  unsigned long previousMillis = millis();
  while (millis() - previousMillis <= mstime) {
    #ifdef DoOTA
    server.handleClient();
    #endif
    Alarm.delay(10);
} } // End Function */
