/* CH32V203 I2C 控制接口：GPIO、功率/电压 ADC、DS18B20、固件版本与 PE4302/PE4312 ATT。 */

#ifndef CH32V_H
#define CH32V_H

#include <Arduino.h>
#include <Wire.h>

class CH32V {
public:
  // GPIO端口、方向、电平和极性。
  enum class Port {
    P0 = 0,
    P1 = 1
  };

  // GPIO 方向配置
  enum class Config {
    IN = 1,   // 输入模式
    OUT = 0   // 输出模式
  };

  // GPIO 电平
  enum class Level {
    L = 0,    // 低电平
    H = 1     // 高电平
  };

  // 极性反转设置
  enum class Polarity {
    ORIGINAL = 0,    // 原始极性
    INVERSE = 1     // 反转极性
  };

  CH32V();
  ~CH32V();

  void attach(TwoWire& wire);
  bool setDeviceAddress(uint8_t address);

  // 单个引脚编号范围为0~15。
  uint8_t config(uint8_t pin, Config cfg);
  uint8_t polarity(uint8_t pin, Polarity pol);
  uint8_t output(uint8_t pin, Level v);
  uint8_t input(uint8_t pin);

  // 写事务收到从机ACK后返回true。
  bool outputConfirmed(uint8_t pin, Level v);

  // 单个引脚配置和控制（端口 + 引脚号）
  void config(Port port, uint8_t pin, Config config);
  void polarity(Port port, uint8_t pin, Polarity polarity);
  void output(Port port, uint8_t pin, Level level);
  Level input(Port port, uint8_t pin);

  // ACK表示I2C写事务成功，不代表外部器件触点状态。
  bool outputConfirmed(Port port, uint8_t pin, Level level);

  void configAll(Port port, uint8_t config);
  void polarityAll(Port port, uint8_t polarity);
  void outputAll(Port port, uint8_t level);
  uint8_t inputAll(Port port);

  void configAll(uint8_t configP0, uint8_t configP1);
  void outputAll(uint8_t levelP0, uint8_t levelP1);
  uint16_t inputAll();

  // 双端口便捷操作。
  void configAll(Config cfg);
  void outputAll(Level v);
  void polarityAll(Polarity pol);

  // 读取正向、反射和电源电压 ADC值。
  uint16_t readFwd();
  uint16_t readRef();

  // Repeated START连续读取，保证寄存器地址和数据处于同一事务。
  uint16_t readFwdFast();   // 原子读取 REG_ADC_FWD_MSB(0x10) + LSB(0x11)
  uint16_t readRefFast();   // 原子读取 REG_ADC_REF_MSB(0x12) + LSB(0x13)
  uint16_t readVccFast();   // 原子读取 REG_ADC_VCC_MSB(0x14) + LSB(0x15)

  // 一次事务读取成对的正向/反射功率；失败时返回false。
  bool readPowerPairFast(uint16_t* fwd, uint16_t* ref);

  // 一次连续读取 DS18B20 状态及带符号 0.1°C 温度值，减少 I2C 事务次数。
  bool readTemperature(int16_t* temperature_deci_c);

  uint16_t readVcc();

  // 读取固件版本；读取失败或固件未实现版本寄存器时返回false。
  bool readFirmwareVersion(uint8_t* major, uint8_t* minor, uint8_t* patch);

  // 衰减编码0~63，每步0.5dB。
  void setAtten(uint8_t code);
  uint8_t readAtten();

private:
  TwoWire* _wire;
  uint8_t _address;
  bool _is_initialized;

  // 记录最近一次收到ACK的输出、极性和方向状态。
  uint8_t _port0_output_shadow;
  uint8_t _port1_output_shadow;
  uint8_t _port0_polarity_shadow;
  uint8_t _port1_polarity_shadow;
  uint8_t _port0_config_shadow;
  uint8_t _port1_config_shadow;

  static const uint8_t REG_PORT0_INPUT    = 0x00;
  static const uint8_t REG_PORT1_INPUT    = 0x01;
  static const uint8_t REG_PORT0_OUTPUT   = 0x02;
  static const uint8_t REG_PORT1_OUTPUT   = 0x03;
  static const uint8_t REG_PORT0_POLARITY = 0x04;
  static const uint8_t REG_PORT1_POLARITY = 0x05;
  static const uint8_t REG_PORT0_CONFIG   = 0x06;
  static const uint8_t REG_PORT1_CONFIG   = 0x07;
  static const uint8_t REG_ADC_FWD_MSB    = 0x10;
  static const uint8_t REG_ADC_FWD_LSB    = 0x11;
  static const uint8_t REG_ADC_REF_MSB    = 0x12;
  static const uint8_t REG_ADC_REF_LSB    = 0x13;
  static const uint8_t REG_ADC_VCC_MSB    = 0x14;  // 电源电压 PA7/ADC_IN7
  static const uint8_t REG_ADC_VCC_LSB    = 0x15;  // 电源电压 PA7/ADC_IN7
  static const uint8_t REG_ATTEN          = 0x16;  // PE4312衰减编码(0-63)
  static const uint8_t REG_FW_VERSION_MAJOR = 0x2F;
  static const uint8_t REG_FW_VERSION_MINOR = 0x30;
  static const uint8_t REG_FW_VERSION_PATCH = 0x31;
  static const uint8_t REG_TEMPERATURE_STATUS = 0x32;
  static const uint8_t REG_TEMPERATURE_LOW = 0x33;
  static const uint8_t REG_TEMPERATURE_HIGH = 0x34;

  bool readRegister(uint8_t reg, uint8_t *value);
  bool writeRegister(uint8_t reg, uint8_t value);
};

CH32V::CH32V() :
  _wire(nullptr),
  _address(0x20),
  _is_initialized(false),
  _port0_output_shadow(0x00u),
  _port1_output_shadow(0x00u),
  _port0_polarity_shadow(0x00u),
  _port1_polarity_shadow(0x00u),
  _port0_config_shadow(0xFFu),
  _port1_config_shadow(0xFFu)
{
}

CH32V::~CH32V()
{
}

void CH32V::attach(TwoWire& wire)
{
  _wire = &wire;
  _is_initialized = true;
}

bool CH32V::setDeviceAddress(uint8_t address)
{
  _address = address;
  if (!_is_initialized) return false;

  _wire->beginTransmission(_address);
  return (_wire->endTransmission() == 0);
}

uint8_t CH32V::config(uint8_t pin, Config cfg)
{
  if (pin > 15) return 0;
  Port port = (pin < 8) ? Port::P0 : Port::P1;
  uint8_t bit = pin % 8;
  config(port, bit, cfg);
  return 0;
}

uint8_t CH32V::polarity(uint8_t pin, Polarity pol)
{
  if (pin > 15) return 0;
  Port port = (pin < 8) ? Port::P0 : Port::P1;
  uint8_t bit = pin % 8;
  polarity(port, bit, pol);
  return 0;
}

uint8_t CH32V::output(uint8_t pin, Level v)
{
  if (pin > 15) return 0;
  Port port = (pin < 8) ? Port::P0 : Port::P1;
  uint8_t bit = pin % 8;
  output(port, bit, v);
  return 0;
}

bool CH32V::outputConfirmed(uint8_t pin, Level v)
{
  if (pin > 15u) return false;
  const Port port = (pin < 8u) ? Port::P0 : Port::P1;
  const uint8_t bit = pin % 8u;
  return outputConfirmed(port, bit, v);
}

uint8_t CH32V::input(uint8_t pin)
{
  if (pin > 15) return 0;
  Port port = (pin < 8) ? Port::P0 : Port::P1;
  uint8_t bit = pin % 8;
  return (input(port, bit) == Level::H) ? 1 : 0;
}

void CH32V::configAll(Config cfg)
{
  configAll((cfg == Config::OUT) ? 0x00 : 0xFF, (cfg == Config::OUT) ? 0x00 : 0xFF);
}

void CH32V::outputAll(Level v)
{
  outputAll((v == Level::L) ? 0x00 : 0xFF, (v == Level::L) ? 0x00 : 0xFF);
}

void CH32V::polarityAll(Polarity pol)
{
  polarityAll(Port::P0, (pol == Polarity::ORIGINAL) ? 0x00 : 0xFF);
  polarityAll(Port::P1, (pol == Polarity::ORIGINAL) ? 0x00 : 0xFF);
}

bool CH32V::readRegister(uint8_t reg, uint8_t *value)
{
  if (!_is_initialized || value == nullptr) return false;

  _wire->beginTransmission(_address);
  if (_wire->write(reg) != 1u || _wire->endTransmission(false) != 0u)
    return false;

  const size_t received = _wire->requestFrom(_address, (size_t)1);
  if (received != 1u || _wire->available() < 1)
    return false;
  const int received_byte = _wire->read();
  if (received_byte < 0)
    return false;
  *value = (uint8_t)received_byte;
  return true;
}

bool CH32V::writeRegister(uint8_t reg, uint8_t value)
{
  if (!_is_initialized) return false;

  _wire->beginTransmission(_address);
  if (_wire->write(reg) != 1u || _wire->write(value) != 1u)
    return false;
  return _wire->endTransmission(true) == 0u;
}

// 单个引脚配置
void CH32V::config(Port port, uint8_t pin, Config config)
{
  if (pin > 7u) return;
  uint8_t reg = (port == Port::P0) ? REG_PORT0_CONFIG : REG_PORT1_CONFIG;
  uint8_t *const shadow = (port == Port::P0) ? &_port0_config_shadow : &_port1_config_shadow;
  uint8_t next = *shadow;

  if (config == Config::IN) {
    next |= (uint8_t)(1u << pin);
  } else {
    next &= (uint8_t)~(1u << pin);
  }

  // 仅在 CH32 确认接收后提交镜像；失败时下次控制会从已确认的安全状态重试。
  if (writeRegister(reg, next)) *shadow = next;
}

// 单个引脚极性设置
void CH32V::polarity(Port port, uint8_t pin, Polarity polarity)
{
  if (pin > 7u) return;
  uint8_t reg = (port == Port::P0) ? REG_PORT0_POLARITY : REG_PORT1_POLARITY;
  uint8_t *const shadow = (port == Port::P0) ? &_port0_polarity_shadow : &_port1_polarity_shadow;
  uint8_t next = *shadow;

  if (polarity == Polarity::INVERSE) {
    next |= (uint8_t)(1u << pin);
  } else {
    next &= (uint8_t)~(1u << pin);
  }

  if (writeRegister(reg, next)) *shadow = next;
}

// 单个引脚输出控制
void CH32V::output(Port port, uint8_t pin, Level level)
{
  // 兼容void接口；需要处理写失败时使用outputConfirmed()。
  (void)outputConfirmed(port, pin, level);
}

// 单个引脚受确认输出控制
bool CH32V::outputConfirmed(Port port, uint8_t pin, Level level)
{
  if (pin > 7u) return false;
  uint8_t reg = (port == Port::P0) ? REG_PORT0_OUTPUT : REG_PORT1_OUTPUT;
  uint8_t *const shadow = (port == Port::P0) ? &_port0_output_shadow : &_port1_output_shadow;
  uint8_t next = *shadow;

  if (level == Level::H) {
    next |= (uint8_t)(1u << pin);
  } else {
    next &= (uint8_t)~(1u << pin);
  }

  // 影子寄存器只能反映已经收到 CH32V ACK 的状态。若本次写失败，保留旧镜像，
  // 使后续安全回退仍能从最后一个已确认的端口状态重新构造完整输出字节。
  if (!writeRegister(reg, next)) return false;
  *shadow = next;
  return true;
}

// 单个引脚输入读取
CH32V::Level CH32V::input(Port port, uint8_t pin)
{
  if (pin > 7u) return Level::L;
  uint8_t reg = (port == Port::P0) ? REG_PORT0_INPUT : REG_PORT1_INPUT;
  uint8_t input_val = 0u;
  // 读取失败按低电平处理，调用方不会把通信异常解释为有效高电平输入。
  if (!readRegister(reg, &input_val)) return Level::L;

  return (input_val & (1 << pin)) ? Level::H : Level::L;
}

// 批量配置整个端口
void CH32V::configAll(Port port, uint8_t config)
{
  uint8_t reg = (port == Port::P0) ? REG_PORT0_CONFIG : REG_PORT1_CONFIG;
  uint8_t *const shadow = (port == Port::P0) ? &_port0_config_shadow : &_port1_config_shadow;
  if (writeRegister(reg, config)) *shadow = config;
}

// 批量设置整个端口极性
void CH32V::polarityAll(Port port, uint8_t polarity)
{
  uint8_t reg = (port == Port::P0) ? REG_PORT0_POLARITY : REG_PORT1_POLARITY;
  uint8_t *const shadow = (port == Port::P0) ? &_port0_polarity_shadow : &_port1_polarity_shadow;
  if (writeRegister(reg, polarity)) *shadow = polarity;
}

// 批量输出控制整个端口
void CH32V::outputAll(Port port, uint8_t level)
{
  uint8_t reg = (port == Port::P0) ? REG_PORT0_OUTPUT : REG_PORT1_OUTPUT;
  uint8_t *const shadow = (port == Port::P0) ? &_port0_output_shadow : &_port1_output_shadow;
  if (writeRegister(reg, level)) *shadow = level;
}

// 批量读取整个端口
uint8_t CH32V::inputAll(Port port)
{
  uint8_t reg = (port == Port::P0) ? REG_PORT0_INPUT : REG_PORT1_INPUT;
  uint8_t input = 0u;
  return readRegister(reg, &input) ? input : 0u;
}

// 批量配置两个端口
void CH32V::configAll(uint8_t configP0, uint8_t configP1)
{
  configAll(Port::P0, configP0);
  configAll(Port::P1, configP1);
}

// 批量输出控制两个端口
void CH32V::outputAll(uint8_t levelP0, uint8_t levelP1)
{
  outputAll(Port::P0, levelP0);
  outputAll(Port::P1, levelP1);
}

// 批量读取两个端口
uint16_t CH32V::inputAll()
{
  return (uint16_t)inputAll(Port::P0) | ((uint16_t)inputAll(Port::P1) << 8);
}

// 读取正向功率12位ADC。
uint16_t CH32V::readFwd()
{
  // 格式：MSB 是高 8 位[11:4]，LSB 是低 4 位[3:0]
  // 组合方式：(MSB << 4) | LSB，还原为12位值(0~4095)
  uint8_t msb;
  uint8_t lsb;
  if (!readRegister(REG_ADC_FWD_MSB, &msb) || !readRegister(REG_ADC_FWD_LSB, &lsb))
    return 0xffffu;
  return ((uint16_t)msb << 4) | ((uint16_t)lsb & 0x0F);
}

// 读取反射功率12位ADC。
uint16_t CH32V::readRef()
{
  uint8_t msb;
  uint8_t lsb;
  if (!readRegister(REG_ADC_REF_MSB, &msb) || !readRegister(REG_ADC_REF_LSB, &lsb))
    return 0xffffu;
  return ((uint16_t)msb << 4) | ((uint16_t)lsb & 0x0F);
}

// Repeated START单事务快速读取正向功率。
uint16_t CH32V::readFwdFast()
{
  if (!_is_initialized) return 0xffffu;

  _wire->beginTransmission(_address);    // START + 写从机地址
  if (_wire->write(REG_ADC_FWD_MSB) != 1u) return 0xffffu;
  if (_wire->endTransmission(false) != 0u) return 0xffffu;
  const size_t received = _wire->requestFrom(_address, (size_t)2);
  if (received != 2u || _wire->available() < 2) return 0xffffu;

  uint8_t msb = _wire->read();
  uint8_t lsb = _wire->read();

  return ((uint16_t)msb << 4) | ((uint16_t)lsb & 0x0F);
}

// Repeated START单事务快速读取反射功率。
uint16_t CH32V::readRefFast()
{
  if (!_is_initialized) return 0xffffu;

  _wire->beginTransmission(_address);
  if (_wire->write(REG_ADC_REF_MSB) != 1u) return 0xffffu;
  if (_wire->endTransmission(false) != 0u) return 0xffffu;
  const size_t received = _wire->requestFrom(_address, (size_t)2);
  if (received != 2u || _wire->available() < 2) return 0xffffu;

  uint8_t msb = _wire->read();
  uint8_t lsb = _wire->read();

  return ((uint16_t)msb << 4) | ((uint16_t)lsb & 0x0F);
}

// 成对快速读取正向/反射功率。
// CH32V 从机自动递增寄存器地址，因此从 0x10 连续读取四个
// 字节会依次得到 FWD_MSB、FWD_LSB、REF_MSB、REF_LSB。整个读阶段由从机 I2C
// 中断连续服务，主循环不会插入 ADC 缓存更新，两个通道由同一缓存快照组成。
bool CH32V::readPowerPairFast(uint16_t* fwd, uint16_t* ref)
{
  if (!_is_initialized || fwd == nullptr || ref == nullptr) return false;

  _wire->beginTransmission(_address);
  if (_wire->write(REG_ADC_FWD_MSB) != 1u) return false;
  if (_wire->endTransmission(false) != 0) return false;

  const size_t received = _wire->requestFrom(_address, (size_t)4);
  if (received != 4u || _wire->available() < 4) return false;

  const uint8_t fwd_msb = _wire->read();
  const uint8_t fwd_lsb = _wire->read();
  const uint8_t ref_msb = _wire->read();
  const uint8_t ref_lsb = _wire->read();
  *fwd = ((uint16_t)fwd_msb << 4) | ((uint16_t)fwd_lsb & 0x0Fu);
  *ref = ((uint16_t)ref_msb << 4) | ((uint16_t)ref_lsb & 0x0Fu);
  return true;
}

bool CH32V::readTemperature(int16_t* temperature_deci_c)
{
  if (!_is_initialized || temperature_deci_c == nullptr) return false;

  // CH32V 自动递增地址；一次事务读取状态和两个温度字节，
  // 避免分别读取造成额外 START/STOP 以及状态与数值不一致。
  _wire->beginTransmission(_address);
  if (_wire->write(REG_TEMPERATURE_STATUS) != 1u) return false;
  if (_wire->endTransmission(false) != 0u) return false;
  const size_t received = _wire->requestFrom(_address, (size_t)3);
  if (received != 3u || _wire->available() < 3) return false;

  const uint8_t status = (uint8_t)_wire->read();
  const uint8_t low = (uint8_t)_wire->read();
  const uint8_t high = (uint8_t)_wire->read();
  if ((status & 0x01u) == 0u) return false;
  *temperature_deci_c = (int16_t)(((uint16_t)high << 8) | low);
  return true;
}

// Repeated START单事务快速读取电源电压。
uint16_t CH32V::readVccFast()
{
  if (!_is_initialized) return 0xffffu;

  _wire->beginTransmission(_address);    // START + 写从机地址
  if (_wire->write(REG_ADC_VCC_MSB) != 1u) return 0xffffu;
  if (_wire->endTransmission(false) != 0u) return 0xffffu;
  const size_t received = _wire->requestFrom(_address, (size_t)2);
  if (received != 2u || _wire->available() < 2) return 0xffffu;

  uint8_t msb = _wire->read();
  uint8_t lsb = _wire->read();

  return ((uint16_t)msb << 4) | ((uint16_t)lsb & 0x0F);
}

// 读取电源电压12位ADC。
uint16_t CH32V::readVcc()
{
  uint8_t msb;
  uint8_t lsb;
  if (!readRegister(REG_ADC_VCC_MSB, &msb) || !readRegister(REG_ADC_VCC_LSB, &lsb))
    return 0xffffu;
  return ((uint16_t)msb << 4) | ((uint16_t)lsb & 0x0F);
}

bool CH32V::readFirmwareVersion(uint8_t* major, uint8_t* minor, uint8_t* patch)
{
  if (!_is_initialized || major == nullptr || minor == nullptr || patch == nullptr) {
    return false;
  }

  // 先读取到局部变量；任一事务失败时不向调用方发布部分版本，避免上一轮缓存与
  // 本轮半截数据混合。主版本 0 保留给未实现新寄存器的旧协处理器固件。
  uint8_t fetchedMajor = 0u;
  uint8_t fetchedMinor = 0u;
  uint8_t fetchedPatch = 0u;
  if (!readRegister(REG_FW_VERSION_MAJOR, &fetchedMajor) ||
      !readRegister(REG_FW_VERSION_MINOR, &fetchedMinor) ||
      !readRegister(REG_FW_VERSION_PATCH, &fetchedPatch) || fetchedMajor == 0u) {
    return false;
  }

  *major = fetchedMajor;
  *minor = fetchedMinor;
  *patch = fetchedPatch;
  return true;
}

// 设置衰减值，编码范围0~63。
void CH32V::setAtten(uint8_t code)
{
  (void)writeRegister(REG_ATTEN, code & 0x3F);
}

uint8_t CH32V::readAtten()
{
  uint8_t atten = 0u;
  return readRegister(REG_ATTEN, &atten) ? atten : 0u;
}

#endif  // CH32V_H
