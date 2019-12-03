#include "BlinkMatrixLed.h"

BlinkMatrixLed::BlinkMatrixLed()
{
}

void BlinkMatrixLed::init()
{
  matrix.begin(0x70); // pass in the address

  Wire.begin();
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  delay(200);

  prevMillis = millis();
  showBackgroud();
  curState = 0; //-1;
}
void BlinkMatrixLed::beSureToBlink()
{

  if (millis() - prevMillis < 600)
    return;
  normal = !normal;

  switch (curState)
  {
  case 0:
    showIdle();
    break;
  case 1:
    showDead();
    break;
  case 2:
    showBalance();
    break;
  case 3:
    showLaydown();
    break;
  case 4:
    showRunning();
    break;
  }
  prevMillis = millis();
}

//idle
void BlinkMatrixLed::normalBlink()
{
  if (curState != 0)
  {
    showBackgroud();
    prevMillis = 0;
    showIdle();
    curState = 0;
    prevMillis = millis();
  }
}

//bat low
void BlinkMatrixLed::slowBlink()
{

  if (curState != 1)
  {
    showBackgroud();
    prevMillis = 0;
    showDead();
    curState = 1;
    prevMillis = millis();
  }
}

//balance
void BlinkMatrixLed::balanceBlink()
{

  if (curState != 2)
  {
    showBackgroud();
    prevMillis = 0;
    showBalance();
    curState = 2;
    prevMillis = millis();
  }
}

void BlinkMatrixLed::laydownBlink()
{
  if (curState != 3)
  {
    showBackgroud();
    prevMillis = 0;
    showLaydown();
    curState = 3;
    prevMillis = millis();
  }
}

void BlinkMatrixLed::runingBlink()
{
  if (curState != 4)
  {
    runIdx = 0;
    showBackgroud();
    prevMillis = 0;
    showRunning();
    curState = 4;
    prevMillis = millis();
  }
}

static const uint8_t smile_bmp[] =
    {
        B01100110,
        B01100110,
        B01111110,
        B10000001,
        B10000001,
        B10000001,
        B01000010,
        B00100100},

                     s_bmp[] =
                         {
                             B00000000,
                             B00000000,
                             B00000000,
                             B00000000,
                             B00000000,
                             B00000000,
                             B00011000,
                             B00000000};

void BlinkMatrixLed::showBackgroud()
{

  matrix.clear();
  matrix.setRotation(1);                             //设置矩阵
  matrix.drawBitmap(0, 0, smile_bmp, 8, 8, LED_RED); //描绘形状与设置颜色，（行起始，列起始；矩阵设置；行数，列数；颜色）
  matrix.writeDisplay();                             //渲染上点阵

  matrix.setRotation(1);
  matrix.drawBitmap(0, 0, s_bmp, 8, 8, LED_YELLOW);
  matrix.writeDisplay();
}

void BlinkMatrixLed::showIdle()
{
  if (normal == true)
  {
    matrix.setRotation(1);
    matrix.drawPixel(2, 4, LED_OFF);
    matrix.drawPixel(5, 4, LED_OFF);

    matrix.drawPixel(3, 4, LED_GREEN);
    matrix.drawPixel(4, 4, LED_GREEN);

    // matrix.drawBitmap(0, 0, neutral_bmp, 8, 8, LED_GREEN);
    matrix.writeDisplay();
  }
  else
  {
    matrix.setRotation(1);
    matrix.drawPixel(2, 4, LED_GREEN);
    matrix.drawPixel(5, 4, LED_GREEN);
    matrix.drawPixel(3, 4, LED_OFF);
    matrix.drawPixel(4, 4, LED_OFF);
    //    matrix.drawBitmap(0, 0, neutral_bmp1, 8, 8, LED_GREEN);
    matrix.writeDisplay();
  }
}

void BlinkMatrixLed::showDead()
{

  if (normal == true)
  {
    matrix.setRotation(1);
    matrix.drawPixel(3, 4, LED_GREEN);
    matrix.drawPixel(4, 4, LED_GREEN);

    // matrix.drawBitmap(0, 0, neutral_bmp, 8, 8, LED_GREEN);
    matrix.writeDisplay();
  }
  else
  {
    matrix.setRotation(1);
    matrix.drawPixel(3, 4, LED_OFF);
    matrix.drawPixel(4, 4, LED_OFF);
    //    matrix.drawBitmap(0, 0, neutral_bmp1, 8, 8, LED_GREEN);
    matrix.writeDisplay();
  }
}

void BlinkMatrixLed::showBalance()
{

  if (normal == true)
  {
    matrix.setRotation(1);
    matrix.drawPixel(2, 3, LED_GREEN);
    matrix.drawPixel(2, 4, LED_OFF);
    matrix.drawPixel(5, 3, LED_OFF);
    matrix.drawPixel(5, 4, LED_GREEN);
    matrix.writeDisplay();
  }
  else
  {
    matrix.setRotation(1);
    matrix.drawPixel(2, 3, LED_OFF);
    matrix.drawPixel(2, 4, LED_GREEN);
    matrix.drawPixel(5, 3, LED_GREEN);
    matrix.drawPixel(5, 4, LED_OFF);
    matrix.writeDisplay();
  }
}

void BlinkMatrixLed::showLaydown()
{

  if (normal == true)
  {
    matrix.setRotation(1);
    // matrix.drawPixel(2, 3, LED_GREEN);
    matrix.drawPixel(2, 4, LED_GREEN);
    matrix.drawPixel(3, 4, LED_GREEN);
    matrix.drawPixel(5, 3, LED_OFF);
    matrix.drawPixel(5, 4, LED_GREEN);
    matrix.writeDisplay();
  }
  else
  {
    matrix.setRotation(1);
    matrix.drawPixel(2, 4, LED_GREEN);
    matrix.drawPixel(3, 4, LED_GREEN);
    matrix.drawPixel(5, 3, LED_GREEN);
    matrix.drawPixel(5, 4, LED_OFF);
    matrix.writeDisplay();
  }
}
void BlinkMatrixLed::showRunning()
{
  matrix.setRotation(1);
  matrix.drawPixel(2 + runIdx, 4, LED_OFF);
  runIdx++;
  if (runIdx > 3)
    runIdx = 0;
  matrix.drawPixel(2 + runIdx, 4, LED_GREEN);
  matrix.writeDisplay();
}
