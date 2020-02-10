#include "ZMCRobot.h"
#include "robot.h"
#include "BleFuncs.h"


// #include <SoftwareSerial.h>

bool connectToHM10();


#define bluetoothTx  10
#define bluetoothRx  11

byte bleBuffer[30];
byte bleBufLen;

//settings parameters
// double atObstacle = 0.25, unsafe = 0.1, angleOff = 0, velocity = 0.5, dfw = 0.15, wheelSyncKp = 10;
// int pwm_zero = 0, pwm_diff = 0, max_pwm = 150, max_rpm = 150, min_rpm = 50;

//defined in BalancePID and SpeedPID
// extern double sKp, sKi, sKd, bKp, bKi, bKd;
#define bluetooth Serial2  //17 Rx 16 Tx

// SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

bool bleConnected = false;

// remote clients will be able to get notifications if this characteristic changes
// the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications

void enumBlooth()
{
      while (bluetooth.available()) {
        char chr = bluetooth.read();
        Serial.write(chr);
      }

}

void initBluetooth()
{

  Serial.println("init BLE HM10 ...");
  bluetooth.begin(19200);
  bleConnected = false;
  /* Now activate the BLE device. It will start continuously transmitting BLE
    advertising packets and will be visible to remote BLE central devices
    until it receives a new connection */

    bool ret = connectToHM10();
    
  // assign event handlers for connected, disconnected to peripheral
  if( ret == true )
  {
    // bluetooth.write("AT+NAMEZMCRobot-UNO");
    // delay(100);
    // enumBlooth();
    // Serial.write('\n');

    bluetooth.write("AT");
    delay(200);
    enumBlooth();

    Serial.println("\nBLE ready...");
  }
  bleBufLen = 0;
}

void doBleHM10Loop()
{
  char chr;


  bool newChars = false;
  while (bluetooth.available()) {
    chr = bluetooth.read();
    bleBuffer[bleBufLen++] = chr; //bluetooth.read();
    // Serial.write(chr);
//    Serial.print(':');
//    Serial.print((char)chr);
//    Serial.print("; ");
    if ( bleBufLen > 29 )
    {
      Serial.println("BLE over flow!");
      bleBuffer[bleBufLen] = 0;
      Serial.println( (char *)bleBuffer );
      bleBufLen = 0;
      newChars = false;
    }
    else
      newChars = true;
  }

  if( !newChars )
    return;

  //OK+LOST
  if( bleBufLen >=7 )
  { //  bleBuffer[0] == 'O' && bleBuffer[1] == 'K' && bleBuffer[2] == '+' && bleBuffer[3]=='L') // the disconnect info OK+LOST
      if(  strstr((char *)bleBuffer, "OK+LOST") != NULL )
      {
        bleConnected = false;
        bleBufLen = 0;
        Serial.println("BLE disconnect!");
        return;
      }
      else if(  strstr((char *)bleBuffer, "OK+CONN") != NULL ) //== (char *)bleBuffer )//OK+CONN
      { 
        bleConnected = true;
        bleBufLen = 0;
        Serial.println("BLE connected!");
        return;
        
      }
      if( bleConnected == false ) //
      {
        if( strstr((char *)bleBuffer, "OK+") != NULL  )
        {
            bleBuffer[bleBufLen] = 0;
            Serial.println((char *)bleBuffer );
            bleBufLen = 0;
        }
      }
    
  }
  
  if ( ! bleConnected ) //check the OK+CONN
  {
    if( bleBufLen > 3 && bleBuffer[0] == bleBufLen-2 ) //get a full package!
    {
      bleConnected = true;
      Serial.println("BLE pkg con ...");
    }

    // if( strstr((char *)bleBuffer, "OK+") == (char *)bleBuffer )
    // {
    //     bleBuffer[bleBufLen] = 0;
    //     Serial.println( (char *)bleBuffer);
    //     enumBlooth();
    //     bleBufLen = 0;
    //     return;
    // }
  }

  if( !bleConnected )
  {
    return;
  }

  while ( bleBufLen > 3 )
  {

    //pkgType pkglen
    byte pkgLen = bleBuffer[0];
    byte pkgType = bleBuffer[1];
    Serial.print("BLE:");
    Serial.print( bleBufLen );
    Serial.write(',');
    Serial.println( pkgLen );

    if( strstr((char *)bleBuffer, "OK+") == (char *)bleBuffer ) //OK+L ....
      break;

    if( pkgLen > 20 || pkgLen < 0 ) // error
    {
        
      Serial.println("ble pkg error!");
      bleBufLen = 0;
      break;
    }
    if ( bleBufLen >= (pkgLen + 2))
    {
      if ( pkgType == 0 ) //cmd
      {
        processBleCommandPackage(bleBuffer + 2 );
      }
      else if( pkgType > 0 && pkgType < 6)
      {
        Serial.println("Cfg pkg!");
        setConfigValue(bleBuffer + 1);
      }
      else
      {
        Serial.println("Error pkg!");
        //bleBufLen = 0;
      }
      if( (bleBufLen - pkgLen - 2) > 0)
         memmove(bleBuffer, bleBuffer + pkgLen + 2, bleBufLen - pkgLen - 2);
      bleBufLen = bleBufLen - pkgLen - 2;
    }
    else  //wait for next package......
    {
      break;
    }
  }
}




void SendBlePackage(byte len, byte *buf)
{
  if ( bleConnected == false )
    return;

  bluetooth.write((byte)len);
  // bluetooth.write( (byte)pkgType);
  for (int i = 0; i < len; i++)
    bluetooth.write((byte) *(buf + i));
}

void sendStatePkgToBle(byte *buf, int len )
{
  SendBlePackage(len, buf);
}

void sendSettingsPkgToBle(byte *settingsArray, int len)
{
  SendBlePackage(len, settingsArray);
}




bool connectToHM10(){
  bool found = false;
 
    // 0 9600 1 19200 2 38400 3 57600 4 115200  5 4800 6 2400 7 1200 8 230400
   long baudrate[6] ={4800,9600,19200,38400,57600,115200};

  for(int j=0; j<6; j++)
   {
      bluetooth.begin(baudrate[j]);
      delay(100);
      Serial.print("BR:");
      Serial.println( baudrate[j] );
     // Serial.println("");
      bluetooth.write("AT");
      delay(500);
      while (bluetooth.available()) {
        found = true;
        Serial.write(bluetooth.read());
       }
       if( found == true )
       {
        Serial.println("OK!");
        return true;
       }
       delay(100);
   }
   return false;
}    

