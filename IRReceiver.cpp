#include "IRReceiver.h"

IRReceiver::IRReceiver()
{
    irpin = 12;
    pinMode(irpin, INPUT);
    ir_addr = 0x00;   // 清零
    ir_code_h = 0x00; // 清零
    ir_code_l = 0x00; // 清零
}

IRReceiver::IRReceiver(int pin)
{
    irpin = pin;
    pinMode(irpin, INPUT);
    ir_addr = 0x00;   // 清零
    ir_code_h = 0x00; // 清零
    ir_code_l = 0x00; // 清零
}

int IRReceiver::readIRCode(IRCode &code)
{
    int in = digitalRead(irpin);
    int count = 0;
    // if (in == 0)
    //     Serial.println("IR.");
    // return 0;

    if (in == 1)
    {
        digitalWrite(13, !in);
        return -1;
    }
    prevMicros = micros();
    digitalWrite(13, !in);
    // code.addr = 0;
    // code.code = 0;

    ir_addr = 0x00;   // 清零
    ir_code_h = 0x00; // 清零
    ir_code_l = 0x00; // 清零

    while (!digitalRead(irpin))
    {
        count++;
        if (count > 20000)
            break;
    }

    curMicros = micros();
    pulse_width_l = curMicros - prevMicros;
    prevMicros = curMicros;
    count = 0;
    while (digitalRead(irpin))
    {
        count++;
        if (count > 20000)
            break;
    }
    curMicros = micros();
    pulse_width_h = curMicros - prevMicros;
    prevMicros = curMicros;

    if (pulse_width_l > 8000 && pulse_width_h > 2500 && pulse_width_h < 5000) //start pulse 9ms 4.5ms
    {
        readCode();
        code.addr = ir_addr;
        code.code_l = ir_code_l;
        code.code_h = ir_code_h;
        Serial.print("ir rec:");
        Serial.print(ir_addr);
        Serial.print(", ");
        Serial.print(ir_code_l);
        Serial.print(", ");
        Serial.println(ir_code_h);
        return 0;
    }
    else //9ms 2.25ms 560us repeat
    {
        count = 0;
        while (!digitalRead(irpin))
        {
            count++;
            if (count > 20000)
                break;
        }
        curMicros = micros();

        // Serial.print(pulse_width_l);
        // Serial.print(",");
        // Serial.print(pulse_width_h);
        // Serial.print(",");
        // Serial.println(curMicros - prevMicros);
        return -1;
    }
}

void IRReceiver::readCode()
{

    int i;
    int j;
    //解析遥控器编码中的用户编码值
    for (i = 0; i < 16; i++)
    {
        if (logicValue() == 1)   //是1
            ir_addr |= (1 << i); //保存键值
    }
    //解析遥控器编码中的命令码
    for (i = 0; i < 8; i++)
    {
        if (logicValue() == 1)     //是1
            ir_code_l |= (1 << i); //保存键值
    }
    //解析遥控器编码中的命令码反码
    for (j = 0; j < 8; j++)
    {
        if (logicValue() == 1)     //是1
            ir_code_h |= (1 << j); //保存键值
    }
}

int IRReceiver::logicValue()
{
    int count = 0;
    while (!digitalRead(irpin))
    {
        count++;
        if (count > 20000)
            break;
    }
    curMicros = micros();
    pulse_width_l = curMicros - prevMicros;
    prevMicros = curMicros;
    count = 0;
    while (digitalRead(irpin))
    {
        count++;
        if (count > 20000)
            break;
    }
    curMicros = micros();
    pulse_width_h = curMicros - prevMicros;
    prevMicros = curMicros;

    // Serial.print(pulse_width_l);
    // Serial.print("_");
    // Serial.println(pulse_width_h);

    if (pulse_width_l < 700 && pulse_width_h < 700) //560us, 1.2ms
    {
        return 0;
    }
    else if (pulse_width_l < 700 && pulse_width_h > 900) //560us 2.25ms
        return 1;
    else
    {
        Serial.print(pulse_width_l);
        Serial.print("_");
        Serial.println(pulse_width_h);
        return -1;
    }
}