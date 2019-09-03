/*********************************************************************
Description: A set of simple serial commands to control a omniWheel drive
  robot and receive back sensor and odometry data. Default configuration
  assumes use of an Arduino Mega 2560 + motor driver board. Edit the
  readEncoder() and setMotorSpeed() wrapper functions if using different
  motor controller or encoder method.
Author: www.corvin.cn
History: 20171129:init code;
         20180209:增加代码版本号,定义初始版本号1.0,通过v命令来获取;
*********************************************************************/
#include "commands.h"
#include "sensors.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "pid_controller.h"
#include "sound.h"

/******************** USER AREAR ********************/
/* current code version */
#define VERSION    1.0

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
#define  AUTO_STOP_INTERVAL  260
/******************** USER END **********************/

/* Serial port baud rate */
#define BAUDRATE   57600

/* Maximum PWM signal */
#define MAX_PWM    255

/* Run the PID loop at 30 times per second -Hz */
#define PID_RATE   30

static const int PowerContro_PIN = 52; //Mosfet Power controller sensor pin
static const int alarm_powerControl_pin = 51;  //alarm light powercontrol sensor pin

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

long lastMotorCommand = AUTO_STOP_INTERVAL;

/* Variable initialization */
// A pair of varibles to help parse serial commands (thanks Fergs)
int arg   = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first,second,third arguments
char argv1[48];
char argv2[48];
char argv3[48];

// The arguments converted to integers
long arg1 = 0;
long arg2 = 0;
long arg3 = 0;

/* Clear the current command parameters */
void resetCommand()
{
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));

  arg1  = 0;
  arg2  = 0;
  arg3  = 0;
  arg   = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand()
{
  int i   = 0;
  char *p = argv1; //p pointer for update pid parameter
  char *str;
  int pid_args[12];

  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);

  switch (cmd)
  {
    case GET_BAUDRATE: //'b'
      Serial.println(BAUDRATE);
      break;

    case ANALOG_READ:  //'a'
      Serial.println(analogRead(arg1));
      break;

    case DIGITAL_READ: //'d'
      Serial.println(digitalRead(arg1));
      break;

    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;

    case DIGITAL_WRITE: //'w'
      if (arg2 == 0)
      {
        digitalWrite(arg1, LOW);
      }
      else if (arg2 == 1)
      {
        digitalWrite(arg1, HIGH);
      }
      Serial.println("OK");
      break;

    case PIN_MODE:
      if (arg2 == 0)
      {
        pinMode(arg1, INPUT);
      }
      else if (arg2 == 1)
      {
        pinMode(arg1, OUTPUT);
      }
      Serial.println("OK");
      break;

    case READ_ENCODERS:  //'e'
      Serial.print(readEncoder(A_WHEEL));
      Serial.print(" ");
      Serial.print(readEncoder(B_WHEEL));
      Serial.print(" ");
      Serial.println(readEncoder(C_WHEEL));
      break;

    case RESET_ENCODERS: //'r'
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;

    case MOTOR_SPEEDS: //'m'
      lastMotorCommand = millis();  /* Reset the auto stop timer */
      if (arg1 == 0 && arg2 == 0 && arg3 == 0)
      {
        setMotorSpeeds(0, 0, 0);
        resetPID();
        setMoveStatus(0);
      }
      else
      {
        setMoveStatus(1);
      }
      setWheelPIDTarget(arg1, arg2, arg3);
      Serial.println("OK");
      break;

    case UPDATE_PID: //'u'
      while ((str = strtok_r(p, ":", &p)) != '\0')
      {
        pid_args[i] = atoi(str);
        i++;
      }
      updatePIDParam(PID_A, pid_args[0], pid_args[1], pid_args[2], pid_args[3]);
      updatePIDParam(PID_B, pid_args[4], pid_args[5], pid_args[6], pid_args[7]);
      updatePIDParam(PID_C, pid_args[8], pid_args[9], pid_args[10], pid_args[11]);
      Serial.println("OK");
      break;

    case SOUND_BEEP: //'f'
      if (arg1 == BASE_POWERON_BEEP)
      {
        basePowerOnBeep();
      }
      else if (arg1 == BASE_POWEROFF_BEEP)
      {
        basePowerOffBeep();
      }
      Serial.println("OK");
      break;

    case READ_PIDIN:
      Serial.print(readPidIn(PID_A));
      Serial.print(" ");
      Serial.print(readPidIn(PID_B));
      Serial.print(" ");
      Serial.println(readPidIn(PID_C));
      break;

    case READ_PIDOUT:
      Serial.print(readPidOut(PID_A));
      Serial.print(" ");
      Serial.print(readPidOut(PID_B));
      Serial.print(" ");
      Serial.println(readPidOut(PID_C));
      break;
      
    case CODE_VERSION: 
      Serial.println(VERSION);
      break;
        
    default:
      Serial.println("Invalid Command");
      break;
  }

  return 0;
}

void initSensorsPin()
{
  pinMode(PowerContro_PIN, OUTPUT);
  digitalWrite(PowerContro_PIN, HIGH); //default enable Power controller

  pinMode(alarm_powerControl_pin, OUTPUT);
  digitalWrite(alarm_powerControl_pin, LOW); //default disable alarm light
}

/* Setup function--runs once at startup. */
void setup()
{
  Serial.begin(BAUDRATE);

  initSensorsPin();
  initSoundPin();
  initCurrentSensor();
  initEncoders();
  initMotorController();
  resetPID();
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.*/
void loop()
{
  while (Serial.available() > 0)
  {
    chr = Serial.read();  //Read the next character

    if (chr == 13)  //Terminate a command with a CR
    {
      if (arg == 1)
      {
        argv1[index] = '\0';
      }
      else if (arg == 2)
      {
        argv2[index] = '\0';
      }
      else if (arg == 3)
      {
        argv3[index] = '\0';
      }
      runCommand();
      resetCommand();
    }
    else if (chr == ' ') // Use spaces to delimit parts of the command
    {
      // Step through the arguments
      if (arg == 0)
      {
        arg = 1;
      }
      else if (arg == 1)
      {
        argv1[index] = '\0';
        arg   = 2;
        index = 0;
      }
      else if (arg == 2)
      {
        argv2[index] = '\0';
        arg   = 3;
        index = 0;
      }
      continue;
    }
    else // process single-letter
    {
      if (arg == 0)
      {
        cmd = chr;  //The first arg is the single-letter command
      }
      else if (arg == 1)
      {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2)
      {
        argv2[index] = chr;
        index++;
      }
      else if (arg == 3)
      {
        argv3[index] = chr;
        index++;
      }
    }
  }//end while()

  //run a PID calculation at the appropriate intervals
  if (millis() > nextPID)
  {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  //Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
  {
    setMotorSpeeds(0, 0, 0);
    resetPID();
    setMoveStatus(0);
    resetEncoders();
  }
}


