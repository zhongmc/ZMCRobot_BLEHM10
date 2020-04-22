#include "ZMCRobot.h"
#include "robot.h"
#include "commFunctions.h"


#define BLE 1
// #include <SoftwareSerial.h>

bool connectToHM10();


#define bluetoothTx  10
#define bluetoothRx  11

char bleBuffer[100];
byte bleBufLen;
long lastPkgMillis;

//settings parameters
// double atObstacle = 0.25, unsafe = 0.1, angleOff = 0, velocity = 0.5, dfw = 0.15, wheelSyncKp = 10;
// int pwm_zero = 0, pwm_diff = 0, max_pwm = 150, max_rpm = 150, min_rpm = 50;

//defined in BalancePID and SpeedPID
// extern double sKp, sKi, sKd, bKp, bKi, bKd;


// SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

bool bleConnected = false;

uint8_t queueLen = 0;
bool queueIdle = true; 

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
  queueLen = 0;
  queueIdle = true; 
  lastPkgMillis = millis();

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
    bluetooth.write("AT");
    delay(20);
    enumBlooth();
    Serial.println("\nBLE ready...");
  }
 
  bleBufLen = 0;
}

void BLEHM10Loop()
{
  char chr;
  doSendBlePkg();

  while( bluetooth.available()){
    chr = bluetooth.read();
    if( chr == ';' || chr =='\r' || chr == '\n')
    {
        bleBuffer[bleBufLen] = 0;
        Serial.println(bleBuffer);
        processCommand(bleBuffer, bleBufLen, BLE);
        bleBufLen = 0;
        continue;
    }

    bleBuffer[bleBufLen++] = chr;
    
    //check the OK+CONN OK+LOST
    if( bleBufLen == 7 )
    {
        bleBuffer[bleBufLen] = 0;
        if(bleConnected &&  strstr((char *)bleBuffer, "OK+LOST") != NULL )
          {
            bleConnected = false;
            bleBufLen = 0;
            Serial.println("BLE disconnect!");
          }
          else if(!bleConnected &&  strstr((char *)bleBuffer, "OK+CONN") != NULL ) //== (char *)bleBuffer )//OK+CONN
          { 
            bleConnected = true;
            bleBufLen = 0;
            Serial.println("BLE connected!");
          }
    }

    if( bleBufLen > 99 )
    {
      Serial.println("BLE over flow!");
      bleBuffer[bleBufLen] = 0;
      Serial.println( (char *)bleBuffer );
      bleBufLen = 0;
    }
  }
}

#define SEND_INTERVAL 10

#define BLE_PKGLEN 20

void sendBleMessages(byte *tmp, uint8_t len )
{
  int ll;
  if( queueIdle == true )
  {
    queueIdle = false;
    if( len > BLE_PKGLEN )
      ll = BLE_PKGLEN;
    else 
      ll = len;
    for( int i=0; i<ll; i++)
    {
      bluetooth.write((byte)*(tmp + i));
    }    
    bluetooth.flush();
    lastPkgMillis = millis();
  }
  else
  {
     ll = 0;
  }
  //将数据写入队列，等待发送
  while( ll < len )
  {
    uint8_t l = len - ll;
    if( l > BLE_PKGLEN )
      l = BLE_PKGLEN;
    addData((tmp + ll), l );
    ll = ll + l; 
  }
}



void doSendBlePkg()
{
  if( isEmpty() )
  {
    if( millis() - lastPkgMillis > SEND_INTERVAL )
      queueIdle = true;
    return;
  }

  queueIdle = false;
  if( millis() - lastPkgMillis < SEND_INTERVAL )
  {
    return;
  }
  byte buf[20];
  int len = pullData( buf );
  if( len <= 0 )
    return;

  for (int i = 0; i < len; i++)
  {
    bluetooth.write((byte) *(buf + i));
  }

  bluetooth.flush();
  lastPkgMillis = millis();
}


#define QUE_SIZE 10
//BLE分包，待发送数据队列实现
// uint8_t queueLen = 0;
// bool queueIdle = true; 
char dataBuf[QUE_SIZE][20];
uint8_t dataLens[QUE_SIZE];

int addData(char *buf, uint8_t len )
{
  if( queueLen >= QUE_SIZE )
  {
    Serial.println("Ble Que full ...");
    return -1; //full
  }
  memset(dataBuf[queueLen], 0, 20);
  memcpy( dataBuf[queueLen], buf, len );
  dataLens[queueLen] = len;
  queueLen++;
  return queueLen;
}

int pullData(char *buf )
{
  
    if( queueLen == 0 )
      return -1; //empty
    
    uint8_t len = dataLens[0];
    memcpy( buf, dataBuf[0], len );
    for( int i=0; i<queueLen - 1; i++ )
    {
      dataLens[i] = dataLens[i+1];
      memcpy(dataBuf[i], dataBuf[i+1], 20);
    }
    queueLen--;
    return len;
}

bool isEmpty()
{
  return queueLen == 0;
}



/*
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


*/


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


