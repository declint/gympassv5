/*
 Gympass v5.1
*/
/*
#pragma message ( "Location: Tannefors" )
#define PASSAGE_POINT "TANNEFORS"
#define ONEDOORMODE 1
#define TFHACK 1
//pinout
#define PIN_CV_RX 10
#define PIN_CV_TX 11
#define PIN_CV_LED 9
#define PIN_CV_SUMMER 8
#define PIN_GP_DOOR1_LOCK 12
#define PIN_GP_DOOR2_LOCK 13
#define PIN_GP_DOOR1_BUTTON A0
#define PIN_GP_DOOR2_BUTTON A1
#define PIN_GP_DOOR1_SENSOR A2
#define PIN_GP_DOOR2_SENSOR A3
#define PIN_GP_DOOR1_SENSOR_OPEN LOW
#define PIN_GP_DOOR2_SENSOR_OPEN LOW
  */
  
#pragma message ( "Location: Skäggetorp" )
#define PASSAGE_POINT "SKAGGET"
#define ONEDOORMODE 0
#define PIN_CV_RX 10
#define PIN_CV_TX 11
#define PIN_CV_LED 9
#define PIN_CV_SUMMER 8
#define PIN_GP_DOOR1_LOCK 12
#define PIN_GP_DOOR2_LOCK 13
#define PIN_GP_DOOR1_BUTTON A0
#define PIN_GP_DOOR2_BUTTON A1
#define PIN_GP_DOOR1_SENSOR A2
#define PIN_GP_DOOR2_SENSOR A3
#define PIN_GP_DOOR1_SENSOR_OPEN HIGH
#define PIN_GP_DOOR2_SENSOR_OPEN HIGH
  
/*
#pragma message ( "Location: Ryd" )
#define PASSAGE_POINT "TANNEFORS"
#define ONEDOORMODE 1
*/

#define TIME_FOR_SERVER_FAILSAFE 3000
#define TIME_TO_IDLE_MODE 10000
#define TIME_DOOR1_UNLOCK 10000
#define TIME_DOOR2_UNLOCK 10000
#define TIME_WEB_PING 30000
#define URL_DOORPASS "http://intranet.strongest.se/gympassv4/index.php/DoorpassV5"

#include <avr/wdt.h>
#include <Process.h>
#include <SoftwareSerial.h>

//Store states
uint8_t PIN_GP_DOOR1_BUTTON_state;
uint8_t PIN_GP_DOOR2_BUTTON_state;
uint8_t PIN_GP_DOOR1_SENSOR_state;
uint8_t PIN_GP_DOOR2_SENSOR_state;


//Setup software serial
SoftwareSerial cvSerial(PIN_CV_RX, PIN_CV_TX); // RX, TX

//Tempstring
char tempstr[128];
char tempstr2[64];
char tempstr3[64];
uint8_t tempstr_i;
uint8_t tempstr2_i;
uint8_t tempstr3_i;

//Tempcounter
uint8_t u8tmp;

//Timers and stuff
unsigned long tmr_last_ping;
unsigned long tmr_web_ping;

// cv5600 stuff
#define CV5600_TIMEOUT_CMD 1500
#define CV5600_BAUDRATE 9600
uint8_t cv5600_rx_buf[16];
uint8_t cv5600_rx_buf_i;
unsigned long cv5600_tmr_last_rx;
enum cv5600_rx_statuses{RX_IDLE = 0, RX_RECV = 10, RX_DONE = 100, RX_PARSED = 200};
uint8_t cv5600_rx_status;
enum cv5600_cmds{CV5600_READERROR = 0, CV5600_IDLE = 1, CV5600_MIFARE = 10, CV5600_KEYPAD = 20};
uint8_t cv5600_cmd;
uint8_t cv5600_cmd_buf[8];
enum cv5600_LED{CV5600_LED_GREEN = 0, CV5600_LED_RED = 1};
unsigned long tmr_cv5600_blink;

/*  Program looping
  MODULE_RFID_READ -> MODULE_RFID_WAIT_FOR_PIN -> MODULE_RFID_PIN_READ -> MODULE_PASSAGE_QUERY_SENT
*/

enum module_statuses{MODULE_IDLE=0, 
                     MODULE_RFID_READ=20, MODULE_RFID_WAIT_FOR_PIN=25, MODULE_RFID_PIN_READ=29, 
                     MODULE_KEYPAD_READ=40,
                     MODULE_PASSAGE_QUERY_SENT=50, MODULE_PASSAGE_QUERY_RECV=55,
                     MODULE_PASSAGE_1_DOOR1_UNLOCKED, MODULE_PASSAGE_2_DOOR1_OPEN, MODULE_PASSAGE_3_DOOR1_CLOSED, MODULE_PASSAGE_4_DOOR2_OPEN,
                     MODULE_WEB_PING_SENT
                     };
                     
uint8_t module_status;
uint8_t module_status_last;
unsigned long tmr_module_status_change;

//Passage
#define KEYSTR_LEN 16
uint8_t cv5600_pin[4];
uint8_t cv5600_pin_i;
uint8_t cv5600_rfid[4];
uint8_t cv5600_keystr[KEYSTR_LEN];
uint8_t cv5600_keystr_i;


//Door status
enum door_statuses{DOORLOCK_OPEN=1, DOORLOCK_CLOSED=2};
uint8_t door1_status;
uint8_t door2_status;
unsigned long tmr_door1_open;
unsigned long tmr_door2_open;

Process p;

void setup() 
{
  pinMode(PIN_CV_SUMMER, OUTPUT);
  pinMode(PIN_CV_LED, OUTPUT);
  pinMode(PIN_GP_DOOR1_LOCK, OUTPUT);
  pinMode(PIN_GP_DOOR2_LOCK, OUTPUT);

  pinMode(PIN_GP_DOOR1_BUTTON, INPUT_PULLUP);
  pinMode(PIN_GP_DOOR2_BUTTON, INPUT_PULLUP);
  pinMode(PIN_GP_DOOR1_SENSOR, INPUT_PULLUP);
  pinMode(PIN_GP_DOOR2_SENSOR, INPUT_PULLUP);

  digitalWrite(PIN_GP_DOOR1_BUTTON, HIGH);
  digitalWrite(PIN_GP_DOOR2_BUTTON, HIGH);
  digitalWrite(PIN_GP_DOOR1_SENSOR, HIGH);
  digitalWrite(PIN_GP_DOOR2_SENSOR, HIGH);

  digitalWrite(PIN_CV_SUMMER, HIGH);
  digitalWrite(PIN_CV_LED, HIGH);
  
  Bridge.begin();  // Initialize the Bridge
  Console.begin();

  // set the data rate for the SoftwareSerial port
  cvSerial.begin(CV5600_BAUDRATE);
  cv5600_cmd = CV5600_IDLE;

  //Play init tune
  cvLED(CV5600_LED_RED);
  cvSummer(250);
  delay(250);
  cvLED(CV5600_LED_GREEN);
  cvSummer(500);
  cvLED(CV5600_LED_RED);
  delay(250);
  cvSummer(250);

  //Init program loop
  module_status = MODULE_IDLE;

  //Enable watchdog
  wdt_enable(WDTO_8S);

  //Reset timers
  tmr_web_ping = millis();
  tmr_door1_open = millis();
  tmr_door2_open = millis();
  tmr_module_status_change = millis();
  tmr_last_ping = millis();

  Console.println(F("Gympassv5 - init done"));      // print the number as well
}

void loop() 
{
  //Program flow handling
  switch(module_status)
  {
  case MODULE_IDLE:
    if (timer_expired(tmr_web_ping, TIME_WEB_PING))
    {
      tmr_web_ping = millis();
      web_ping();
    }
    break;

  case MODULE_WEB_PING_SENT:
    if (! p.running())
      change_module_status(MODULE_IDLE);

    if (timer_expired(tmr_module_status_change, TIME_FOR_SERVER_FAILSAFE))
      change_module_status(MODULE_IDLE);

    break;

  case MODULE_RFID_READ:
    cv5600_pin_i = 0;
    cv5600_rfid[0] = cv5600_cmd_buf[0];
    cv5600_rfid[1] = cv5600_cmd_buf[1];
    cv5600_rfid[2] = cv5600_cmd_buf[2];
    cv5600_rfid[3] = cv5600_cmd_buf[3];
    change_module_status(MODULE_RFID_WAIT_FOR_PIN);
    break;

  case MODULE_RFID_WAIT_FOR_PIN:
    if (cv5600_pin_i >= 4)
      change_module_status(MODULE_RFID_PIN_READ);

    if (timer_expired(tmr_cv5600_blink, 100))
    {
      cvToggleLED();
      tmr_cv5600_blink = millis();
    }
    
    break;

  case MODULE_RFID_PIN_READ:
    //Need to do the query
    sprintf(tempstr2, "%02x%02x%02x%02x/%c%c%c%c",
            cv5600_rfid[0], cv5600_rfid[1], cv5600_rfid[2], cv5600_rfid[3],
            cv5600_pin[0] + 48, cv5600_pin[1] + 48, cv5600_pin[2] + 48, cv5600_pin[3] + 48);

    sprintf(tempstr3, "/hasvalidcard/%s/%s", tempstr2, PASSAGE_POINT);
    run_curl_asynch(tempstr3);

    change_module_status(MODULE_PASSAGE_QUERY_SENT);
    break;

  case MODULE_KEYPAD_READ:
    break;

  case MODULE_PASSAGE_QUERY_SENT:
    // do nothing until the process finishes, so you get the whole output:
    if (! p.running())
      change_module_status(MODULE_PASSAGE_QUERY_RECV);

    //Failsafe if no response
    if (timer_expired(tmr_module_status_change, TIME_FOR_SERVER_FAILSAFE))
    {
      passage_ok();
      break;
    }

    if (timer_expired(tmr_cv5600_blink, 25))
    {
      cvToggleLED();
      tmr_cv5600_blink = millis();
    }

    break;

  case MODULE_PASSAGE_QUERY_RECV:
    // Read command output. runShellCommand() should have passed "Signal: xx&":
    tempstr_i = 0;
    while (p.available()>0) 
    {
      tempstr[tempstr_i] = p.read();
      tempstr_i++;
      if (tempstr_i>sizeof(tempstr))
        tempstr_i = sizeof(tempstr) - 1;
    }
    tempstr[tempstr_i] = 0;
    Console.print(F("Query recv "));
    Console.print(tempstr_i);
    Console.println(F(" bytes"));
    Console.println(tempstr);

    parse_passage_query_recv();
    break;

  case MODULE_PASSAGE_1_DOOR1_UNLOCKED: //Wait for door to open, then go to next state
#if (TFHACK == 1)
#pragma message ( "Tannefors mode" )
    door1_openlock();
    door2_openlock();
    delay(250);
    door1_closelock();
    door2_closelock();
    change_module_status(MODULE_IDLE);

#elif (ONEDOORMODE == 1)
#pragma message ( "One door mode" )
    door1_openlock();
    door2_openlock();
    change_module_status(MODULE_IDLE);
    break;
#else
#pragma message ( "Two door mode" )
    if (door1_sensor_open())
    {
      change_module_status(MODULE_PASSAGE_2_DOOR1_OPEN);
    }
    break;
#endif
    
  case MODULE_PASSAGE_2_DOOR1_OPEN: //Door1 is open, wait for it to close
    if (! door1_sensor_open())
    {
      change_module_status(MODULE_PASSAGE_3_DOOR1_CLOSED);
    }
    break;

  case MODULE_PASSAGE_3_DOOR1_CLOSED: //Door1 is closed, unlock door2
    door2_openlock();
    delay(500);
    change_module_status(MODULE_PASSAGE_4_DOOR2_OPEN);
    break;

  case MODULE_PASSAGE_4_DOOR2_OPEN: //Door2 has been opened. Go to idle mode
    if (door2_sensor_open())
    {
      change_module_status(MODULE_IDLE);
    }
    break;

  default:
    module_status = MODULE_IDLE;
    break;
  }

  switch (cv5600_cmd)
  {
    case CV5600_IDLE:
      break;

    case CV5600_READERROR:
      cv5600_cmd = CV5600_IDLE;
      break;

    case CV5600_MIFARE:
      cv5600_cmd = CV5600_IDLE;
      change_module_status(MODULE_RFID_READ);
      break;

    case CV5600_KEYPAD:
      cv5600_cmd = CV5600_IDLE;
      if (cv5600_pin_i < 4)
        cv5600_pin[cv5600_pin_i] = cv5600_cmd_buf[0];
      cv5600_pin_i++;

      //For the pincode login
      if (cv5600_cmd_buf[0] == 0x0a) // *
        cv5600_keystr_i = 0;
      else if (cv5600_cmd_buf[0] == 0x0b) // #
      {
        Console.println(F("# pressed, querying server"));

        //Store pincode in tempstr2
        for(u8tmp=0; u8tmp < cv5600_keystr_i; u8tmp++)
        {
          tempstr2[u8tmp] = cv5600_keystr[u8tmp] + 48;
        }
        tempstr2[cv5600_keystr_i] = 0;

        sprintf(tempstr3, "/pincode/%s/%s", tempstr2, PASSAGE_POINT);
        run_curl_asynch(tempstr3);
    
        change_module_status(MODULE_PASSAGE_QUERY_SENT);
      }
      //Store to string
      else if (cv5600_keystr_i < KEYSTR_LEN)
      {
        cv5600_keystr[cv5600_keystr_i] = cv5600_cmd_buf[0];
        cv5600_keystr_i++;
      }
      break;

    default:
      cv5600_cmd = CV5600_IDLE;
      break;
  }
  
  cvHandler();
  
  //Go to idle if nothing has happened
  if (module_status != MODULE_IDLE)
    if (timer_expired(tmr_module_status_change, TIME_TO_IDLE_MODE))
      change_module_status(MODULE_IDLE);

  if (door1_status == DOORLOCK_OPEN)
  {
    if(timer_expired(tmr_door1_open, TIME_DOOR1_UNLOCK))
    {
      Console.println(F("Door1 timeout, locking door"));
      door1_closelock();
    }      

    if(door1_sensor_open())
    {
      Console.println(F("Door1 open, locking door"));
      door1_closelock();
    }
  }

  if (door2_status == DOORLOCK_OPEN)
  {
    if (timer_expired(tmr_door2_open, TIME_DOOR2_UNLOCK))
    {
      Console.println(F("Door2 timeout, locking door"));
      door2_closelock();
    }      

    if (door2_sensor_open())
    {
      Console.println(F("Door2 open, locking door"));
      door2_closelock();
    }
  }

  //Send ping to show we are alive
  if (abs(millis() - tmr_last_ping) > 15000)
  {
    tmr_last_ping = millis();
    Console.print(F("Gympassv5: "));      // print the number as well
    getStateStr();
    Console.println(tempstr);
    //Dump sensors
    usbprint_sensors();
  }

  //Check USB Serial
  usbserial_handler();

  //Reset watchdog
  wdt_reset();

  //Check inputs, print messages if change is detected
  if (digitalRead(PIN_GP_DOOR1_BUTTON) != PIN_GP_DOOR1_BUTTON_state)
  {
    update_inputs();

    if (PIN_GP_DOOR1_BUTTON_state == LOW)
      if ((! door1_sensor_open()) && (! door2_sensor_open()))
        door1_openlock();
  }
  if (digitalRead(PIN_GP_DOOR2_BUTTON) != PIN_GP_DOOR2_BUTTON_state)
  {
    update_inputs();

    if (PIN_GP_DOOR2_BUTTON_state == LOW)
//      if ((! door1_sensor_open()) && (! door2_sensor_open()))
        door2_openlock();
  }
  
  if (digitalRead(PIN_GP_DOOR1_SENSOR) != PIN_GP_DOOR1_SENSOR_state)
    update_inputs();
  if (digitalRead(PIN_GP_DOOR2_SENSOR) != PIN_GP_DOOR2_SENSOR_state)
    update_inputs();
}

/*
 RX_IDLE -> RX_RECV -> RX_DONE
 */

void update_inputs()
{
  PIN_GP_DOOR1_BUTTON_state = digitalRead(PIN_GP_DOOR1_BUTTON);
  PIN_GP_DOOR2_BUTTON_state = digitalRead(PIN_GP_DOOR2_BUTTON);
  PIN_GP_DOOR1_SENSOR_state = digitalRead(PIN_GP_DOOR1_SENSOR);
  PIN_GP_DOOR2_SENSOR_state = digitalRead(PIN_GP_DOOR2_SENSOR);

  usbprint_sensors();
}

void usbserial_handler()
{
  if (Console.available())
  {
    tempstr_i = Console.read();

    switch(tempstr_i)
    {
      case '1':
        door1_openlock();
        break;
      case '2':
        door2_openlock();
        break;
      case 'd':
      case 'D':
        usbprint_sensors();
        break;
      default:
        break;
    }
  }
}

int cvHandler()
{
  if ((cv5600_rx_status == RX_RECV) ||
      (cv5600_rx_status == RX_IDLE))
  {
    if (cvSerial.available()) 
    {
      cv5600_rx_status = RX_RECV;
      cv5600_tmr_last_rx = micros();
      cv5600_rx_buf[cv5600_rx_buf_i] = cvSerial.read();
      cv5600_rx_buf_i++;
  
      if (cv5600_rx_buf_i >= 15)
        cv5600_rx_buf_i = 15;
    }
  }
  
  if (cv5600_rx_status == RX_RECV)
  {
    if (cv5600_rx_buf_i == 8)
    {
      if ((cv5600_rx_buf[3] == 0x02) && (cv5600_rx_buf[7] == 0x03))
        cv5600_rx_status = RX_DONE;
    }   
    
    if (timer_expired_micros(cv5600_tmr_last_rx,CV5600_TIMEOUT_CMD))
      cv5600_rx_status = RX_DONE;
  }
  
  if (cv5600_rx_status == RX_DONE)
  {
    Console.println(F("cv5600 RX_DONE"));

    sprintf(tempstr, "Bytes recv:%i", cv5600_rx_buf_i);
    Console.println(tempstr);

    for(int i=0; i<cv5600_rx_buf_i; i++)
    {
      sprintf(tempstr, " %02X", cv5600_rx_buf[i]);
      Console.print(tempstr);
    }
    Console.println("");

    //Send it to the parser
    cvHandler_parse();

    // Reset pointers
    cv5600_rx_buf_i = 0;

    cv5600_rx_status = RX_IDLE;
  }
}

int cvHandler_parse()
{
  //Check if mifare card
  if ((cv5600_rx_buf_i == 11)
      &&
      (cv5600_rx_buf[0] == 0x02)
      &&
      (cv5600_rx_buf[10] == 0x03))
  {
    cv5600_cmd = CV5600_MIFARE;
    cv5600_cmd_buf[0] = cv5600_rx_buf[5];
    cv5600_cmd_buf[1] = cv5600_rx_buf[6];
    cv5600_cmd_buf[2] = cv5600_rx_buf[7];
    cv5600_cmd_buf[3] = cv5600_rx_buf[8];

    cvLED(CV5600_LED_GREEN);
    cvSummer(50);
    delay(100);
    cvSummer(100);
    cvLED(CV5600_LED_RED);

    sprintf(tempstr, "Mifare [%02x%02x%02x%02x]", cv5600_cmd_buf[0], cv5600_cmd_buf[1], cv5600_cmd_buf[2], cv5600_cmd_buf[3]);
    Console.println(tempstr);
  }
  //Check if keypad
  else if ((cv5600_rx_buf_i == 8)
      &&
      (cv5600_rx_buf[0] == 0x02)
      &&
      (cv5600_rx_buf[7] == 0x03))
  {
    cv5600_cmd = CV5600_KEYPAD;
    cv5600_cmd_buf[0] = cv5600_rx_buf[5];
    sprintf(tempstr, "Keypad [%02x]", cv5600_cmd_buf[0]);
    Console.println(tempstr);
  }
  else
  {
    cv5600_cmd = CV5600_READERROR;
    Console.println(F("CV5600 read error"));
  }
}

void cvSummer(int ms)
{
  digitalWrite(PIN_CV_SUMMER, LOW);
  delay(ms);
  digitalWrite(PIN_CV_SUMMER, HIGH);
}

void cvLED(cv5600_LED led)
{
  if (led == CV5600_LED_GREEN)
    digitalWrite(PIN_CV_LED, LOW);
  else
    digitalWrite(PIN_CV_LED, HIGH);
}

void cvToggleLED()
{
  digitalWrite(PIN_CV_LED, (! digitalRead(PIN_CV_LED)));
}


uint8_t getCheckSum(char *string, int strlength)
{
  int XOR = 0;  
  
  for (int i = 0; i < strlength; i++) 
  {
    XOR = XOR ^ string[i];
  }
  
  return XOR;
}

bool timer_expired(unsigned long tmr, unsigned int time_ms)
{
  if (abs(millis() - tmr) > time_ms)
  {
    return true;
  }
  
  return false;
}

bool timer_expired_micros(unsigned long tmr, unsigned int time_ms)
{
  if (abs(micros() - tmr) > time_ms)
  {
    return true;
  }
  
  return false;
}

void getStateStr()
{
  //Module state
  switch (module_status)
  {
    case MODULE_IDLE:
      strcpy_PF(tempstr2, PSTR("Module status: MODULE_IDLE"));
      break;
    case MODULE_RFID_READ:
      strcpy_PF(tempstr2, PSTR("Module status: MODULE_RFID_READ"));
      break;
    case MODULE_RFID_WAIT_FOR_PIN:
      strcpy_PF(tempstr2, PSTR("Module status: MODULE_RFID_WAIT_FOR_PIN"));
      break;
    case MODULE_RFID_PIN_READ:
      strcpy_PF(tempstr2, PSTR("Module status: MODULE_RFID_PIN_READ"));
      break;
    case MODULE_KEYPAD_READ:
      strcpy_PF(tempstr2, PSTR("Module status: MODULE_KEYPAD_READ"));
      break;
    case MODULE_PASSAGE_QUERY_SENT:
      strcpy_PF(tempstr2, PSTR("Module status: MODULE_PASSAGE_QUERY_SENT"));
      break;
    case MODULE_PASSAGE_QUERY_RECV:
      strcpy_PF(tempstr2, PSTR("Module status: MODULE_PASSAGE_QUERY_RECV"));
      break;
    case MODULE_PASSAGE_1_DOOR1_UNLOCKED:
      strcpy_PF(tempstr2, PSTR( "Module status: MODULE_PASSAGE_1_DOOR1_UNLOCKED"));
      break;
    case MODULE_PASSAGE_2_DOOR1_OPEN:
      strcpy_PF(tempstr2, PSTR("Module status: MODULE_PASSAGE_2_DOOR1_OPEN"));
      break;
    case MODULE_PASSAGE_3_DOOR1_CLOSED:
      strcpy_PF(tempstr2, PSTR("Module status: MODULE_PASSAGE_3_DOOR1_CLOSED"));
      break;
    case MODULE_PASSAGE_4_DOOR2_OPEN:
      strcpy_PF(tempstr2, PSTR("Module status: MODULE_PASSAGE_4_DOOR2_OPEN"));
      break;

    default:
      sprintf(tempstr2, "Module status: UNDEFINED_STATE:%i", module_status);
      break;
  }

  switch (cv5600_rx_status)
  {
    case RX_IDLE:
      strcpy_PF(tempstr3, PSTR("CV5600 status: RX_IDLE"));
      break;
    case RX_RECV:
      strcpy_PF(tempstr3, PSTR("CV5600 status: RX_RECV"));
      break;
    case RX_DONE:
      strcpy_PF(tempstr3, PSTR("CV5600 status: RX_DONE"));
      break;
    case RX_PARSED:
      strcpy_PF(tempstr3, PSTR("CV5600 status: RX_PARSED"));
      break;
  }

  sprintf(tempstr, "%s %s", tempstr2, tempstr3);
}

void change_module_status(uint8_t new_status)
{
  //Store old module status
  module_status_last = module_status;
  
  module_status = new_status;
  tmr_module_status_change = millis();

  sprintf(tempstr, "[05%i]", millis()%10000);
  Console.print(tempstr);
  
  Console.print(F("Statechange: "));      // print the number as well
  getStateStr();
  Console.println(tempstr);

  //Dump sensors
  usbprint_sensors();
}

void usbprint_sensors()
{
  //Dump sensors
  sprintf(tempstr,"Door 1 - button[%i] sensor[%s] : Door 2 button[%i] sensor[%s]", digitalRead(PIN_GP_DOOR1_BUTTON), (door1_sensor_open()?"open":"closed"), digitalRead(PIN_GP_DOOR2_BUTTON), (door2_sensor_open()?"open":"closed"));
  Console.println(tempstr);

}

void parse_passage_query_recv()
{
  int pos;

  //failsafe
  if (tempstr_i < 2)
  {
    passage_ok();
    return;
  }
  
  pos = strstr(tempstr, "<dpv5>") - tempstr;

  sprintf(tempstr2, "Result: %c", tempstr[pos+6]);
  Console.println(tempstr2);

  if (tempstr[pos+6] == '0') // Passage denied
  {
    passage_denied();
  }
  else
  {
    passage_ok();
  }
}

void passage_ok()
{
  door1_openlock();

  change_module_status(MODULE_PASSAGE_1_DOOR1_UNLOCKED);

  //Play init tune
  cvLED(CV5600_LED_GREEN);
  cvSummer(100);
  delay(100);
  cvSummer(100);
  delay(100);
  cvSummer(100);
}

void passage_denied()
{
  door1_closelock();
  door2_closelock();

  //Go to idle mode
  change_module_status(MODULE_IDLE);

  cvLED(CV5600_LED_RED);

  cvSummer(100);
  delay(100);
  cvSummer(250);
  delay(250);
  cvSummer(500);
}

void door1_openlock()
{
  tmr_door1_open = millis();
  door1_status = DOORLOCK_OPEN;
  digitalWrite(PIN_GP_DOOR1_LOCK, HIGH);
  Console.println("Door1 unlock");
}

void door1_closelock()
{
  tmr_door1_open = millis();
  door1_status = DOORLOCK_CLOSED;
  digitalWrite(PIN_GP_DOOR1_LOCK, LOW);
  cvLED(CV5600_LED_RED);
  Console.println("Door1 lock");
}

void door2_openlock()
{
  tmr_door2_open = millis();
  door2_status = DOORLOCK_OPEN;
  digitalWrite(PIN_GP_DOOR2_LOCK, HIGH);
  Console.println("Door2 unlock");
}

void door2_closelock()
{
  tmr_door2_open = millis();
  door2_status = DOORLOCK_CLOSED;
  digitalWrite(PIN_GP_DOOR2_LOCK, LOW);
  cvLED(CV5600_LED_RED);
  Console.println("Door2 lock");
}

inline bool door1_sensor_open()
{
  if (PIN_GP_DOOR1_SENSOR_OPEN == digitalRead(PIN_GP_DOOR1_SENSOR))
    return true;
  
  return false;
}

inline bool door2_sensor_open()
{
  if (PIN_GP_DOOR2_SENSOR_OPEN == digitalRead(PIN_GP_DOOR2_SENSOR))
    return true;

  return false;
}

inline bool door1_button_pushed()
{
  return (! digitalRead(PIN_GP_DOOR1_BUTTON));
}

inline bool door2_button_pushed()
{
  return (! digitalRead(PIN_GP_DOOR2_BUTTON));
}

void web_ping()
{
  sprintf(tempstr3, "/doormodule_ping/%s", PASSAGE_POINT);
  run_curl_asynch(tempstr3);
  
  change_module_status(MODULE_WEB_PING_SENT);
}

int run_curl_asynch(const char* cmd)
{
  strcpy_PF(tempstr, PSTR(URL_DOORPASS));
  strcat(tempstr, cmd);
  Console.println(tempstr);
  
  p.begin("curl");
  p.addParameter(F("-u"));
  p.addParameter(F("gym:muskler"));
  p.addParameter(tempstr);
  p.runAsynchronously();
  
}

