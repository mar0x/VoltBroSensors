#include "VB_BMP280.h"
//#include "VoltBroSensors.h"
#include "arduino_mpu9250_VB_routines.h"

uint16_t VB_BMP280::read16_LE(uint8_t reg) {
  uint8_t temp[2];
  arduino_i2c_read(dev_addr, reg, 2, temp);
  //uint16_t temp = VoltBroSensors::I2C_getRegister(dev_addr, reg, 2);
  //return (temp >> 8) | (temp << 8);
  uint16_t ret_val = temp[1];
  ret_val <<= 8;
  ret_val |= temp[0];
  return ret_val;

}

int16_t VB_BMP280::readS16_LE(uint8_t reg)
{
  return (int16_t)read16_LE(reg);

}

bool VB_BMP280::begin() {
    return begin(BMP280_DEFAULT_ADDRESS);
}

bool VB_BMP280::begin(uint8_t address, uint8_t config, uint8_t control) {
    Wire.begin();
    dev_addr = address;
    dev_control = control;
    if (testConnection()) {
        initialize(config);
        return true;
    } else {
        return false;
    }
}

void VB_BMP280::initialize(uint8_t config) {
    // Читаем калибровочные коэффициенты (константы для корректировки показаний температуры и давления) из EEPROM
    uint16_t *v = &dig_T1;
    for (uint8_t reg = BMP280_REGISTER_DIG_T1; reg <= BMP280_REGISTER_DIG_P9; reg += 2, v += 1) {
        *v = read16_LE(reg);
    }
    SLP = 0;
    /*Serial.print("t1 = ");
    Serial.println(dig_T1);
    Serial.print("t2 = ");
    Serial.println(dig_T2);
    Serial.print("t3 = ");
    Serial.println(dig_T3);
    Serial.print("p1 = ");
    Serial.println(dig_P1);
    Serial.print("p2 = ");
    Serial.println(dig_P2);
    Serial.print("p3 = ");
    Serial.println(dig_P3);
    Serial.print("p4 = ");
    Serial.println(dig_P4);
    Serial.print("p5 = ");
    Serial.println(dig_P5);
    Serial.print("p6 = ");
    Serial.println(dig_P6);
    Serial.print("p7 = ");
    Serial.println(dig_P7);
    Serial.print("p8 = ");
    Serial.println(dig_P8);
    Serial.print("p9 = ");
    Serial.println(dig_P9);*/

    //VoltBroSensors::I2C_WriteReg(dev_addr, BMP280_REGISTER_CONFIG, BMP280_CONFIG);
    arduino_i2c_write_byte(dev_addr, BMP280_REGISTER_CONFIG, config);

    if ((dev_control & BMP280_MODE_NORMAL) == BMP280_MODE_NORMAL) {
        arduino_i2c_write_byte(dev_addr, BMP280_REGISTER_CONTROL, dev_control);
    }

    // инициируем чтение для получения текущего давления
    read();

    // расчетное давление на уровне моря в мм.рт.ст.
    SLP = pres / pow(1 - (start_altitude / 44330), 5.255);
}

// Проверка соединения с девайсом
bool VB_BMP280::testConnection() {
    //VoltBroSensors::I2C_ReadBytes(dev_addr, BMP280_REGISTER_CHIPID, 1, buffer);
    return 0x58 == arduino_i2c_read_byte(dev_addr, BMP280_REGISTER_CHIPID);
}

int32_t VB_BMP280::readTemperature(uint8_t *buffer)
{
  int32_t var1, var2;

  int32_t adc_T;
  adc_T = (((int32_t)buffer[0]) << 12)|(((int32_t)buffer[1]) << 4)|(buffer[2] >> 4);

  var1  = ((((adc_T>>3) - ((int32_t)dig_T1 <<1))) *
       ((int32_t)dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)dig_T1)) *
         ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
       ((int32_t)dig_T3)) >> 14;

  t_fine = var1 + var2;

  return (t_fine * 5 + 128) >> 8;
}


boolean VB_BMP280::read() {
    float var1, var2;
    int32_t adc_P;
    uint8_t buffer[6];

    //VoltBroSensors::I2C_WriteReg(dev_addr, BMP280_REGISTER_CONTROL, BMP280_MEAS);
    if ((dev_control & BMP280_MODE_NORMAL) == BMP280_MODE_FORCED) {
        arduino_i2c_write_byte(dev_addr, BMP280_REGISTER_CONTROL, dev_control);
        arduino_i2c_write_byte(dev_addr, BMP280_REGISTER_CONTROL, dev_control);
    }
    DelayWhileMeasuring();

    // Запускаем измерение всего (по отдельности нельзя)
    arduino_i2c_read(dev_addr, BMP280_REGISTER_PRESSUREDATA, 6, buffer);

    //int32_t temperature;
    //float pressure;
    temp = readTemperature(buffer + 3)/100.0;

    adc_P = (((int32_t)buffer[0]) << 12)|(((int32_t)buffer[1]) << 4)|(buffer[2] >> 4);

    var1 = ((float)t_fine/2.0)-64000.0;
    var2 = var1 * var1 * ((float)dig_P6) / 32768.0;
    var2 = var2 + var1 * ((float)dig_P5) * 2.0;
    var2 = (var2/4.0)+(((float)dig_P4) * 65536.0);
    var1 = (((float)dig_P3) * var1 * var1 / 524288.0 + ((float)dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0)*((float)dig_P1);
    if (var1 == 0.0)
    {
        pres = 0;
        return false; // avoid exception caused by division by zero
    }
    pres = 1048576.0-(float)adc_P;
    pres = (pres-(var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((float)dig_P9) * (pres) * (pres) / 2147483648.0;
    var2 = pres * ((float)dig_P8) / 32768.0;
    pres = pres + (var1 + var2 + ((float)dig_P7)) / 16.0;


    //высоты
    alti = 44330 * (1 - pow(pres / SLP, 1 / 5.255));
    return true;
}

uint8_t I2C_getRegister(uint8_t dev_addr, uint8_t reg_addr)
{
    return arduino_i2c_read_byte(dev_addr, reg_addr);
}

void VB_BMP280::DelayWhileMeasuring(void) {
    int i = 0;
    while (I2C_getRegister(dev_addr, BMP280_REGISTER_STATUS) & (1<<3) && i < 50) {
        delay(1);
        i++;
    }
    if (i >= 50 || i == 0) {
        ErrData = true;
    } //	Выходим из цикла если 3й бит регистра 0xF3 (флаг SCO) = 0, или после 50го цикла, или если циклов не было
}
