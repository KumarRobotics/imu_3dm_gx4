/*
 * imu.hpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 2/6/2014
 *		  Author: gareth
 */

#include "imu.hpp"
#include <chrono>
#include <locale>
#include <tuple>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <boost/assert.hpp>


extern "C" {
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h> //  close
#include <string.h> //  strerror
}

#define kDefaultTimeout    (300)
#define kBufferSize        (10) //  keep this small, or 1000Hz is not attainable

#define u8(x) static_cast<uint8_t>((x))

#define COMMAND_CLASS_BASE    u8(0x01)
#define COMMAND_CLASS_3DM     u8(0x0C)
#define COMMAND_CLASS_FILTER  u8(0x0D)

#define DATA_CLASS_IMU        u8(0x80)
#define DATA_CLASS_FILTER     u8(0x82)

#define FUNCTION_APPLY        u8(0x01)

#define SELECTOR_IMU          u8(0x01)
#define SELECTOR_FILTER       u8(0x03)

//  base commands
#define DEVICE_PING           u8(0x01)
#define DEVICE_IDLE           u8(0x02)
#define DEVICE_RESUME         u8(0x06) 

//  3DM and FILTER commands
#define COMMAND_GET_DEVICE_INFO       u8(0x03)
#define COMMAND_GET_IMU_BASE_RATE     u8(0x06)
#define COMMAND_GET_FILTER_BASE_RATE  u8(0x0B)
#define COMMAND_IMU_MESSAGE_FORMAT    u8(0x08)
#define COMAMND_FILTER_MESSAGE_FORMAT u8(0x0A)
#define COMMAND_ENABLE_DATA_STREAM    u8(0x11)
#define COMMAND_FILTER_CONTROL_FLAGS  u8(0x14)
#define COMMAND_UART_BAUD_RATE        u8(0x40)
#define COMMAND_SET_HARD_IRON         u8(0x3A)
#define COMMAND_SET_SOFT_IRON         u8(0x3B)
#define COMMAND_ENABLE_MEASUREMENTS   u8(0x41)
#define COMMAND_DEVICE_STATUS         u8(0x64)

//  supported fields
#define FIELD_QUATERNION        u8(0x03)
#define FIELD_ACCELEROMETER     u8(0x04)
#define FIELD_GYROSCOPE         u8(0x05)
#define FIELD_GYRO_BIAS         u8(0x06)
#define FIELD_MAGNETOMETER      u8(0x06)
#define FIELD_ANGLE_UNCERTAINTY u8(0x0A)
#define FIELD_BIAS_UNCERTAINTY  u8(0x0B)
#define FIELD_BAROMETER         u8(0x17)
#define FIELD_DEVICE_INFO       u8(0x81)
#define FIELD_IMU_BASERATE      u8(0x83)
#define FIELD_FILTER_BASERATE   u8(0x8A)
#define FIELD_STATUS_REPORT     u8(0x90)
#define FIELD_ACK_OR_NACK       u8(0xF1)

using namespace imu_3dm_gx4;

// trim from start
static inline std::string ltrim(std::string s) {
  s.erase(s.begin(),
          std::find_if(s.begin(), s.end(),
                       std::not1(std::ptr_fun<int, int>(std::isspace))));
  return s;
}

//  swap order of bytes if on a little endian platform
template <size_t sz> void to_device_order(uint8_t buffer[sz]) {
#ifdef HOST_LITTLE_ENDIAN
  for (size_t i = 0; i < sz / 2; i++) {
    std::swap(buffer[i], buffer[sz - i - 1]);
  }
#endif
}

template <> void to_device_order<1>(uint8_t buffer[1]) {}

template <typename T> size_t encode(uint8_t *buffer, const T &t) {
  static_assert(std::is_fundamental<T>::value,
                "Only fundamental types may be encoded");

  const size_t szT = sizeof(T);
  union {
    T tc;
    uint8_t bytes[szT];
  };
  tc = t;

  to_device_order<szT>(bytes);

  //  append
  for (size_t i = 0; i < szT; i++) {
    buffer[i] = bytes[i];
  }
  
  return szT;
}

template <typename T, typename... Ts>
size_t encode(uint8_t *buffer, const T &t, const Ts &... ts) {
  const size_t sz = encode(buffer, t);
  //  recurse
  return sz + encode(buffer + sizeof(T), ts...);
}

template <typename T> void decode(const uint8_t *buffer, size_t count, 
                                  T *output) {
  const size_t szT = sizeof(T);
  static_assert(std::is_fundamental<T>::value,
                "Only fundamental types may be decoded");
  union {
    T tc;
    uint8_t bytes[szT];
  };

  for (size_t i = 0; i < count; i++) {
    for (size_t j = 0; j < szT; j++) {
      bytes[j] = buffer[j];
    }
    to_device_order<szT>(&bytes[0]);
    output[i] = tc;

    buffer += szT;
  }
}

/**
 * @brief Tool for encoding packets more easily by simply appending fields.
 */
class PacketEncoder {
public:
  PacketEncoder(Imu::Packet& p) : p_(p), fs_(0), enc_(false) {
    p.length = 0;
  }
  virtual ~PacketEncoder() {
    //  no destruction without completion
    assert(!enc_);
  }
  
  void beginField(uint8_t desc) {
    assert(!enc_);
    assert(p_.length < sizeof(p_.payload) - 2); //  2 bytes per field minimum
    fs_ = p_.length;
    p_.payload[fs_+1] = desc;
    p_.length += 2;
    enc_ = true;
  }
  
  template <typename ...Ts>
  void append(const Ts& ...args) {
    assert(enc_); //  todo: check argument length here
    p_.length += encode(p_.payload+p_.length, args...);
  }
  
  void endField() {
    assert(enc_);
    p_.payload[fs_] = p_.length - fs_;
    enc_ = false;
  }
  
private:
  Imu::Packet& p_;
  uint8_t fs_;
  bool enc_;
};

/**
 * @brief Tool for decoding packets by iterating through fields.
 */
class PacketDecoder {
public:
  PacketDecoder(const Imu::Packet& p) : p_(p), fs_(0), pos_(2) {
    assert(p.length > 0);
  }
  
  int fieldDescriptor() const {
    if (fs_ > sizeof(p_.payload)-2) { 
      return -1; 
    }
    if (p_.payload[fs_] == 0) { 
      return -1;  //  no field 
    }
    return p_.payload[fs_ + 1]; //  descriptor after length
  }
  
  int fieldLength() const {
    assert(fs_ < sizeof(p_.payload));
    return p_.payload[fs_];
  }
  
  bool fieldIsAckOrNack() const {
    const int desc = fieldDescriptor();
    if (desc == static_cast<int>(FIELD_ACK_OR_NACK)) {
      return true;
    }
    return false;
  }
  
  bool advanceTo(uint8_t field) {
    for (int d; (d = fieldDescriptor()) > 0; advance()) {
      if (d == static_cast<int>(field)) {
        return true;
      }
    }
    return false;
  }
  
  void advance() {
    fs_ += p_.payload[fs_];
    pos_ = 2;  //  skip length and descriptor
  }

  template <typename T>
  void extract(size_t count, T* output) {
    BOOST_VERIFY(fs_ + pos_ + sizeof(T) * count <= sizeof(p_.payload));
    decode(&p_.payload[fs_ + pos_], count, output);
    pos_ += sizeof(T) * count;
  }

private:
  const Imu::Packet& p_;
  uint8_t fs_;
  uint8_t pos_;
};

bool Imu::Packet::isIMUData() const {
  return descriptor == DATA_CLASS_IMU;
}

bool Imu::Packet::isFilterData() const {
  return descriptor == DATA_CLASS_FILTER;
}

int Imu::Packet::ackErrorCodeFor(const Packet &command) const {
  PacketDecoder decoder(*this);
  const uint8_t sentDesc = command.descriptor;
  const uint8_t sentField = command.payload[1];

  if (sentDesc != this->descriptor) {
    //  not for this packet
    return -1;
  }

  //  look for a matching ACK
  for (int d; (d = decoder.fieldDescriptor()) > 0; decoder.advance()) {
    if (decoder.fieldIsAckOrNack()) {
      uint8_t cmd, code;
      decoder.extract(1, &cmd);
      decoder.extract(1, &code);
      if (cmd == sentField) {
        //  match
        return code;
      }
    }
  }
  //  not an ACK/NACK
  return -1;
}

void Imu::Packet::calcChecksum() {
  uint8_t byte1 = 0, byte2 = 0;

#define add_byte(x)  \
  byte1 += (x);      \
  byte2 += byte1;

  add_byte(syncMSB);
  add_byte(syncLSB);
  add_byte(descriptor);
  add_byte(length);

  for (size_t i = 0; i < length; i++) {
    add_byte(payload[i]);
  }
#undef add_byte

  checksum = (static_cast<uint16_t>(byte1) << 8) + static_cast<uint16_t>(byte2);
#ifdef HOST_LITTLE_ENDIAN
  uint8_t temp = checkLSB;
  checkLSB = checkMSB;
  checkMSB = temp;
#endif
}

Imu::Packet::Packet(uint8_t desc)
    : syncMSB(kSyncMSB), syncLSB(kSyncLSB), descriptor(desc), length(0),
      checksum(0) {
  memset(&payload, 0, sizeof(payload));
}

std::string Imu::Packet::toString() const {
  std::stringstream ss;
  ss << std::hex;
  ss << "SyncMSB: " << static_cast<int>(syncMSB) << "\n";
  ss << "SyncLSB: " << static_cast<int>(syncLSB) << "\n";
  ss << "Descriptor: " << static_cast<int>(descriptor) << "\n";
  ss << "Length: " << static_cast<int>(length) << "\n";
  ss << "Payload: ";
  for (size_t s=0; s < length; s++) {
    ss << static_cast<int>(payload[s]) << " ";
  }
  ss << "\nCheck MSB: " << static_cast<int>(checkMSB) << "\n";
  ss << "Check LSB: " << static_cast<int>(checkLSB);
  return ss.str();
}

std::map <std::string, std::string> Imu::Info::toMap() const {
  std::map<std::string, std::string> map;
  map["Firmware version"] = std::to_string(firmwareVersion);
  map["Model name"] = modelName;
  map["Model number"] = modelNumber;
  map["Serial number"] = serialNumber;
  map["Device options"] = deviceOptions;
  //  omit lot number since it is empty
  return map;
}

uint16_t Imu::Info::getModelNumber() const {
  // Simply convert the first contiguous group of numbers into an int
  int modelInt;
  try {
    modelInt = std::stoi(modelNumber);
  }
  catch (const std::invalid_argument &ex) {
    // If cannot convert, default to the -25 (6324)
    modelInt = 6324;
  }
  return static_cast<uint16_t>(modelInt);
}

std::map <std::string, unsigned int> Imu::DiagnosticFields::toMap() const {
  std::map<std::string, unsigned int> map;
  map["Model number"] = modelNumber;
  map["Selector"] = selector;
  map["Status flags"] = statusFlags;
  map["System timer"] = systemTimer;
  map["Num 1PPS Pulses"] = num1PPSPulses;
  map["Last 1PPS Pulse"] = last1PPSPulse;
  map["Imu stream enabled"] = imuStreamEnabled;
  map["Filter stream enabled"] = filterStreamEnabled;
  map["Imu packets dropped"] = imuPacketsDropped;
  map["Filter packets dropped"] = filterPacketsDropped;
  map["Com bytes written"] = comBytesWritten;
  map["Com bytes read"] = comBytesRead;
  map["Com num write overruns"] = comNumReadOverruns;
  map["Com num read overruns"] = comNumReadOverruns;
  map["Usb bytes written"] = usbBytesWritten;
  map["Usb bytes read"] = usbBytesRead;
  map["Usb num write overruns"] = usbNumWriteOverruns;
  map["Usb num read overruns"] = usbNumReadOverruns;
  map["Num imu parse errors"] = numIMUParseErrors;
  map["Total imu messages"] = totalIMUMessages;
  map["Last imu message"] = lastIMUMessage;
  return map;
}

Imu::command_error::command_error(const Packet& p, uint8_t code) : 
  std::runtime_error(generateString(p, code)) {}

std::string Imu::command_error::generateString(const Packet& p, uint8_t code) {
  std::stringstream ss;
  ss << "Received NACK with error code " << std::hex << static_cast<int>(code);
  ss << ". Command Packet:\n" << p.toString();
  return ss.str();
}

Imu::timeout_error::timeout_error(bool write, unsigned int to)
    : std::runtime_error(generateString(write,to)) {}

std::string Imu::timeout_error::generateString(bool write, 
                                               unsigned int to) {
  std::stringstream ss;
  ss << "Timed-out while " << ((write) ? "writing" : "reading") << ". ";
  ss << "Time-out limit is " << to << "ms.";
  return ss.str();
}

Imu::Imu(const std::string &device, bool verbose) : device_(device), verbose_(verbose),
  fd_(0), 
  rwTimeout_(kDefaultTimeout),
  srcIndex_(0), dstIndex_(0),
  state_(Idle) {
  //  buffer for storing reads
  buffer_.resize(kBufferSize);
}

Imu::~Imu() { disconnect(); }

void Imu::connect() {
  if (fd_ > 0) {
    throw std::runtime_error("Socket is already open");
  }

  const char *path = device_.c_str();
  fd_ = ::open(path, O_RDWR | O_NOCTTY | O_NDELAY); //  read/write, no
                                                    // controlling terminal, non
                                                    // blocking access
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open device : " + device_);
  }

  //  make sure it is non-blocking
  if (fcntl(fd_, F_SETFL, FNONBLOCK) < 0) {
    disconnect();
    throw io_error(strerror(errno));
  }

  struct termios toptions;
  if (tcgetattr(fd_, &toptions) < 0) {
    disconnect();
    throw io_error(strerror(errno));
  }

  //  set default baud rate
  cfsetispeed(&toptions, B115200);
  cfsetospeed(&toptions, B115200);

  toptions.c_cflag &= ~PARENB; //  no parity bit
  toptions.c_cflag &= ~CSTOPB; //  disable 2 stop bits (only one used instead)
  toptions.c_cflag &= ~CSIZE;  //  disable any previous size settings
  toptions.c_cflag |= HUPCL;   //  enable modem disconnect
  toptions.c_cflag |= CS8;     //  bytes are 8 bits long

  toptions.c_cflag &= ~CRTSCTS; //  no hardware RTS/CTS switch

  toptions.c_cflag |=
      CREAD |
      CLOCAL; //  enable receiving of data, forbid control lines (CLOCAL)
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY); //  no software flow control
  toptions.c_iflag &= ~(INLCR|ICRNL); //  disable NL->CR and CR->NL

  //  disable the following
  //  ICANON = input is read on a per-line basis
  //  ECHO/ECHOE = echo enabled
  //  ISIG = interrupt signals
  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  toptions.c_oflag &= ~OPOST; //  disable pre-processing of input data
  toptions.c_oflag &= ~(ONLCR|OCRNL); //  disable NL->CR and CR->NL

  toptions.c_cc[VMIN] = 0;  //  no minimum number of bytes when reading
  toptions.c_cc[VTIME] = 0; //  no time blocking

  //  TCSAFLUSH = make change after flushing i/o buffers
  if (tcsetattr(fd_, TCSAFLUSH, &toptions) < 0) {
    disconnect();
    throw io_error(strerror(errno));
  }
}

void Imu::disconnect() {
  if (fd_ > 0) {
    //  send the idle command first
    idle(false);  //  we don't care about reply here
    close(fd_);
  }
  fd_ = 0;
}

bool Imu::termiosBaudRate(unsigned int baud) {
  struct termios toptions;
  if (tcgetattr(fd_, &toptions) < 0) {
    return false;
  }

  speed_t speed;
  switch (baud) {
  case 9600:
    speed = B9600;
    break;
  case 19200:
    speed = B19200;
    break;
  case 115200:
    speed = B115200;
    break;
  case 230400:
    speed = B230400;
    break;
  case 460800:
    speed = B460800;
    break;
  case 921600:
    speed = B921600;
    break;
  default:
    throw std::invalid_argument("Invalid Baud Rate" );
  }

  //  modify only the baud rate
  cfsetispeed(&toptions, speed);
  cfsetospeed(&toptions, speed);

  if (tcsetattr(fd_, TCSAFLUSH, &toptions) < 0) {
    return false;
  }

  usleep(200000); //  wait for connection to be negotiated
                  //  found this length through experimentation
  return true;
}

void Imu::runOnce() {
  int sig = pollInput(5);
  if (sig < 0) {
    //  failure in poll/read, device disconnected
    throw io_error(strerror(errno));
  }
}

void Imu::selectBaudRate(unsigned int baud) {
  //  baud rates supported by the 3DM-GX4-25
  const size_t num_rates = 6;
  unsigned int rates[num_rates] = {
    9600, 19200, 115200, 230400, 460800, 921600
  };

  if (!std::binary_search(rates, rates + num_rates, baud)) {
    //  invalid baud rate
    std::stringstream ss;
    ss << "Baud rate unsupported: " << baud;
    throw std::invalid_argument(ss.str());
  }

  Imu::Packet pp(COMMAND_CLASS_BASE); //  was 0x02
  {
    PacketEncoder encoder(pp);
    encoder.beginField(DEVICE_PING);
    encoder.endField();
  }
  pp.calcChecksum();

  size_t i;
  bool foundRate = false;
  for (i = 0; i < num_rates; i++) {
    if (verbose_){
      std::cout << "Switching to baud rate " << rates[i] << std::endl;
    }
    if (!termiosBaudRate(rates[i])) {
      throw io_error(strerror(errno));
    }
    
    if (verbose_) {
      std::cout << "Switched baud rate to " << rates[i] << std::endl;
      std::cout << "Sending a ping packet.\n" << std::flush;
    }

    //  send ping and wait for first response
    sendPacket(pp, 100);
    try {
      receiveResponse(pp, 500);
    } catch (timeout_error&) {
      if (verbose_) {
        std::cout << "Timed out waiting for ping response.\n" << std::flush;
      }
      continue;
    } catch (command_error&) {
      if (verbose_) {
        std::cout << "IMU returned error code for ping.\n" << std::flush;
      }
      continue;
    } //  do not catch io_error

    if (verbose_) {
      std::cout << "Found correct baudrate.\n" << std::flush;
    }
    
    //  no error in receiveResponse, this is correct baud rate
    foundRate = true;
    break;
  }

  if (!foundRate) {
    throw std::runtime_error("Failed to reach device " + device_);
  }

  //  we are on the correct baud rate, now change to the new rate
  Packet comm(COMMAND_CLASS_3DM);  //  was 0x07
  {
    PacketEncoder encoder(comm);
    encoder.beginField(COMMAND_UART_BAUD_RATE);
    encoder.append(FUNCTION_APPLY);
    encoder.append(static_cast<uint32_t>(baud));
    encoder.endField();
    assert(comm.length == 0x07);
  }
  comm.calcChecksum();

  try {
    if (verbose_) {
      std::cout << "Instructing device to change to " << baud << std::endl
                << std::flush;
    }
    sendCommand(comm);
  } catch (std::exception& e) {
    std::stringstream ss;
    ss << "Device rejected baud rate " << baud << ".\n";
    ss << e.what();
    throw std::runtime_error(ss.str());
  }
  
  //  device has switched baud rate, now we should also
  if (!termiosBaudRate(baud)) {
    throw io_error(strerror(errno));
  }

  //  ping
  try {
    ping();
  } catch (std::exception& e) {
    std::string err("Device did not respond to ping.\n");
    err += e.what();
    throw std::runtime_error(err);
  }
}

void Imu::ping() {
  Imu::Packet p(COMMAND_CLASS_BASE);  //  was 0x02
  PacketEncoder encoder(p);
  encoder.beginField(DEVICE_PING);
  encoder.endField();
  p.calcChecksum();
  assert(p.checkMSB == 0xE0 && p.checkLSB == 0xC6);
  sendCommand(p);
}

void Imu::idle(bool needReply) {
  Imu::Packet p(COMMAND_CLASS_BASE);  //  was 0x02
  PacketEncoder encoder(p);
  encoder.beginField(DEVICE_IDLE);
  encoder.endField();
  p.calcChecksum();
  assert(p.checkMSB == 0xE1 && p.checkLSB == 0xC7);
  sendCommand(p, needReply);
}

void Imu::resume() {
  Imu::Packet p(COMMAND_CLASS_BASE);  //  was 0x02
  PacketEncoder encoder(p);
  encoder.beginField(DEVICE_RESUME);
  encoder.endField();
  p.calcChecksum();
  assert(p.checkMSB == 0xE5 && p.checkLSB == 0xCB);
  sendCommand(p);
}

void Imu::getDeviceInfo(Imu::Info &info) {
  Imu::Packet p(COMMAND_CLASS_BASE);  //  was 0x02
  PacketEncoder encoder(p);
  encoder.beginField(COMMAND_GET_DEVICE_INFO);
  encoder.endField();
  p.calcChecksum();
  assert(p.checkMSB == 0xE2 && p.checkLSB == 0xC8);

  sendCommand(p);
  {
    PacketDecoder decoder(packet_);
    BOOST_VERIFY(decoder.advanceTo(FIELD_DEVICE_INFO));
    char buffer[16];
    
    decoder.extract(1, &info.firmwareVersion);
    //  decode all strings and trim left whitespace
    decoder.extract(sizeof(buffer), &buffer[0]);
    info.modelName = ltrim(std::string(buffer,16));
    decoder.extract(sizeof(buffer), &buffer[0]);
    info.modelNumber = ltrim(std::string(buffer,16));
    decoder.extract(sizeof(buffer), &buffer[0]);
    info.serialNumber = ltrim(std::string(buffer,16));
    decoder.extract(sizeof(buffer), &buffer[0]);
    info.lotNumber = ltrim(std::string(buffer,16));
    decoder.extract(sizeof(buffer), &buffer[0]);
    info.deviceOptions = ltrim(std::string(buffer,16));
  }
}

void Imu::getIMUDataBaseRate(uint16_t &baseRate) {
  Packet p(COMMAND_CLASS_3DM);  //  was 0x02
  PacketEncoder encoder(p);
  encoder.beginField(COMMAND_GET_IMU_BASE_RATE);
  encoder.endField();
  p.calcChecksum();

  sendCommand(p);
  {
    PacketDecoder decoder(packet_);
    BOOST_VERIFY(decoder.advanceTo(FIELD_IMU_BASERATE));
    decoder.extract(1, &baseRate);
  }
}

void Imu::getFilterDataBaseRate(uint16_t &baseRate) {
  Packet p(COMMAND_CLASS_3DM); //  was 0x02
  PacketEncoder encoder(p);
  encoder.beginField(COMMAND_GET_FILTER_BASE_RATE);
  encoder.endField();
  p.calcChecksum();

  sendCommand(p);
  decode(&packet_.payload[6], 1, &baseRate);
}

void Imu::getDiagnosticInfo(Imu::DiagnosticFields &fields,
                            const Imu::Info &info) {
  Packet p(COMMAND_CLASS_3DM);
  PacketEncoder encoder(p);
  encoder.beginField(COMMAND_DEVICE_STATUS);
  encoder.append(info.getModelNumber());  //  device model number
  encoder.append(u8(0x02)); //  diagnostic mode
  encoder.endField();
  p.calcChecksum();

  sendCommand(p);
  {
    PacketDecoder decoder(packet_);
    BOOST_VERIFY(decoder.advanceTo(FIELD_STATUS_REPORT));

    decoder.extract(1, &fields.modelNumber);
    decoder.extract(1, &fields.selector);
    decoder.extract(4, &fields.statusFlags);
    decoder.extract(2, &fields.imuStreamEnabled);
    decoder.extract(13, &fields.imuPacketsDropped);
  }
}

void Imu::setIMUDataRate(uint16_t decimation, 
                        const std::bitset<4>& sources) {
  Imu::Packet p(COMMAND_CLASS_3DM);  //  was 0x04
  PacketEncoder encoder(p);
  
  //  valid field descriptors: accel, gyro, mag, pressure
  static const uint8_t fieldDescs[] = { FIELD_ACCELEROMETER, 
                                        FIELD_GYROSCOPE, 
                                        FIELD_MAGNETOMETER, 
                                        FIELD_BAROMETER };
  assert(sizeof(fieldDescs) == sources.size());
  std::vector<uint8_t> fields;
  
  for (size_t i=0; i < sources.size(); i++) {
    if (sources[i]) {
      fields.push_back(fieldDescs[i]);
    }
  }
  encoder.beginField(COMMAND_IMU_MESSAGE_FORMAT);
  encoder.append(FUNCTION_APPLY, u8(fields.size()));
  
  for (const uint8_t& field : fields) {
    encoder.append(field, decimation);
  }
  
  encoder.endField();
  p.calcChecksum();
  sendCommand(p);
}

void Imu::setFilterDataRate(uint16_t decimation, const std::bitset<4>& sources) {
  Imu::Packet p(COMMAND_CLASS_3DM);  //  was 0x04
  PacketEncoder encoder(p);
 
  static const uint8_t fieldDescs[] = { FIELD_QUATERNION,
                                        FIELD_GYRO_BIAS,
                                        FIELD_ANGLE_UNCERTAINTY,
                                        FIELD_BIAS_UNCERTAINTY };
  assert(sizeof(fieldDescs) == sources.size());
  std::vector<uint8_t> fields;
  
  for (size_t i=0; i < sources.size(); i++) {
    if (sources[i]) {
      fields.push_back(fieldDescs[i]);
    }
  }
  
  encoder.beginField(COMAMND_FILTER_MESSAGE_FORMAT);
  encoder.append(FUNCTION_APPLY, u8(fields.size()));
  
  for (const uint8_t& field : fields) {
    encoder.append(field, decimation);
  }
  encoder.endField();
  p.calcChecksum();
  sendCommand(p);
}

void Imu::enableMeasurements(bool accel, bool magnetometer) {
  Imu::Packet p(COMMAND_CLASS_FILTER);
  PacketEncoder encoder(p);
  encoder.beginField(COMMAND_ENABLE_MEASUREMENTS);
  uint16_t flag=0;
  if (accel) {
    flag |= 0x01;
  }
  if (magnetometer) {
    flag |= 0x02;
  }
  encoder.append(FUNCTION_APPLY, flag);
  encoder.endField();
  p.calcChecksum();
  sendCommand(p);
}

void Imu::enableBiasEstimation(bool enabled) {
  Imu::Packet p(COMMAND_CLASS_FILTER);
  PacketEncoder encoder(p);
  encoder.beginField(COMMAND_FILTER_CONTROL_FLAGS);
  uint16_t flag = 0xFFFE;
  if (enabled) {
    flag = 0xFFFF;
  }  
  encoder.append(FUNCTION_APPLY, flag);
  encoder.endField();
  p.calcChecksum();
  sendCommand(p);
}

void Imu::setHardIronOffset(float offset[3]) {
  Imu::Packet p(COMMAND_CLASS_3DM);
  PacketEncoder encoder(p);
  encoder.beginField(COMMAND_SET_HARD_IRON);
  encoder.append(FUNCTION_APPLY, offset[0], offset[1], offset[2]);
  encoder.endField();
  assert(p.length == 0x0F);
  p.calcChecksum();
  sendCommand(p);
}

void Imu::setSoftIronMatrix(float matrix[9]) {
  Imu::Packet p(COMMAND_CLASS_3DM);
  PacketEncoder encoder(p);
  encoder.beginField(COMMAND_SET_SOFT_IRON);
  encoder.append(FUNCTION_APPLY);
  for (int i=0; i < 9; i++) {
    encoder.append(matrix[i]);
  }
  encoder.endField();
  assert(p.length == 0x27);
  p.calcChecksum();
  sendCommand(p);
}

void Imu::enableIMUStream(bool enabled) {
  Packet p(COMMAND_CLASS_3DM);
  PacketEncoder encoder(p);  
  encoder.beginField(COMMAND_ENABLE_DATA_STREAM);
  encoder.append(FUNCTION_APPLY, SELECTOR_IMU);
  encoder.append(u8(enabled));
  encoder.endField();
  p.calcChecksum();
  if (enabled) {
    assert(p.checkMSB == 0x04 && p.checkLSB == 0x1A);
  }
  sendCommand(p);
}

void Imu::enableFilterStream(bool enabled) {
  Packet p(COMMAND_CLASS_3DM);
  PacketEncoder encoder(p);
  encoder.beginField(COMMAND_ENABLE_DATA_STREAM);
  encoder.append(FUNCTION_APPLY, SELECTOR_FILTER);
  encoder.append(u8(enabled));
  encoder.endField();
  p.calcChecksum();
  if (enabled) {
    assert(p.checkMSB == 0x06 && p.checkLSB == 0x1E);
  }
  sendCommand(p);
}

void
Imu::setIMUDataCallback(const std::function<void(const Imu::IMUData &)> &cb) {
  imuDataCallback_ = cb;
}

void Imu::setFilterDataCallback(
    const std::function<void(const Imu::FilterData &)> &cb) {
  filterDataCallback_ = cb;
}

int Imu::pollInput(unsigned int to) {
  //  poll socket for inputs
  struct pollfd p;
  p.fd = fd_;
  p.events = POLLIN;

  int rPoll = poll(&p, 1, to); // timeout is in millis
  if (rPoll > 0) {
    const ssize_t amt = ::read(fd_, &buffer_[0], buffer_.size());
    if (amt > 0) {
      return handleRead(amt);
    } else if (amt == 0) {
      //  end-of-file, device disconnected
      return -1;
    }
  } else if (rPoll == 0) {
    return 0; //  no socket can be read
  }

  if (errno == EAGAIN || errno == EINTR) {
    //  treat these like timeout errors
    return 0;
  }

  //  poll() or read() failed
  return -1;
}

/**
 * @note handleByte will interpret a single byte. It will return the number
 *  of bytes which can be cleared from the queue. On early failure, this will
 *  always be 1 - ie. kSyncMSB should be cleared from the queue. On success, the
 *  total length of the valid packet should be cleared.
 */
std::size_t Imu::handleByte(const uint8_t& byte, bool& found) {
  found = false;
  if (state_ == Idle) {
    //  reset dstIndex_ to start of packet
    dstIndex_ = 0;
    if (byte == Imu::Packet::kSyncMSB) {
      packet_.syncMSB = byte;
      state_ = Reading;
      //  clear previous packet out
      memset(&packet_.payload[0], 0, sizeof(packet_.payload));
    } else {
      //  byte is no good, stay in idle
      return 1;
    }
  }
  else if (state_ == Reading) {
    const size_t end = Packet::kHeaderLength + packet_.length;
    //  fill out fields of packet structure
    if (dstIndex_ == 1) {
      if (byte != Imu::Packet::kSyncLSB) {
        //  not a true header, throw away and go back to idle
        state_ = Idle;
        return 1;
      }
      packet_.syncLSB = byte;
    }
    else if (dstIndex_ == 2) {
      packet_.descriptor = byte;
    } 
    else if (dstIndex_ == 3) {
      packet_.length = byte;
    }
    else if (dstIndex_ < end) {
      packet_.payload[dstIndex_ - Packet::kHeaderLength] = byte;
    }
    else if (dstIndex_ == end) {
      packet_.checkMSB = byte;
    }
    else if (dstIndex_ == end + 1) {
      state_ = Idle; //  finished packet
      packet_.checkLSB = byte;

      //  check checksum
      const uint16_t sum = packet_.checksum;
      packet_.calcChecksum();

      if (sum != packet_.checksum) {
        //  invalid, go back to waiting for a marker in the stream
        std::cout << "Warning: Dropped packet with mismatched checksum\n"
                  << std::flush;
        if (verbose_) {
          std::cout << "Expected " << std::hex << 
                       static_cast<int>(packet_.checksum) << " but received " <<
                       static_cast<int>(sum) << std::endl;
          std::cout << "Packet content:\n" << packet_.toString() << std::endl;
          std::cout << "Queue content: \n";
          for (const uint8_t& q : queue_) {
            std::cout << static_cast<int>(q) << " ";
          }
          std::cout << "\n" << std::flush;
        }
        return 1;
      } else {
        //  successfully read a packet
        processPacket();
        found = true;
        return end+2;
      }
    }
  }
  
  //  advance to next byte in packet
  dstIndex_++;
  return 0;
}

//  parses packets out of the input buffer
int Imu::handleRead(size_t bytes_transferred) {
  //  read data into queue
  std::stringstream ss;
  ss << "Handling read : " << std::hex;
  for (size_t i = 0; i < bytes_transferred; i++) {
    queue_.push_back(buffer_[i]);
    ss << static_cast<int>(buffer_[i]) << " ";
  }
  ss << std::endl;
  if (verbose_) {
    std::cout << ss.str() << std::flush;
  }
  
  bool found = false;
  while (srcIndex_ < queue_.size() && !found) {
    const uint8_t head = queue_[srcIndex_];
    const size_t clear = handleByte(head, found);
    //  pop 'clear' bytes from the queue
    for (size_t i=0; i < clear; i++) {
      queue_.pop_front();
    }
    if (clear) {
      //  queue was shortened, return to head
      srcIndex_=0;
    } else {
      //  continue up the queue
      srcIndex_++;
    }
  }

  //  no packet
  return found;
}

void Imu::processPacket() {
  IMUData data;
  FilterData filterData;
  PacketDecoder decoder(packet_);
  
  if (packet_.isIMUData()) {
    //  process all fields in the packet
    for (int d; (d = decoder.fieldDescriptor()) > 0; decoder.advance()) {
      switch (u8(d)) {
      case FIELD_ACCELEROMETER:
        decoder.extract(3, &data.accel[0]);
        data.fields |= IMUData::Accelerometer;
        break;
      case FIELD_GYROSCOPE:
        decoder.extract(3, &data.gyro[0]);
        data.fields |= IMUData::Gyroscope;
        break;
      case FIELD_MAGNETOMETER:
        decoder.extract(3, &data.mag[0]);
        data.fields |= IMUData::Magnetometer;
        break;
      case FIELD_BAROMETER:
        decoder.extract(1, &data.pressure);
        data.fields |= IMUData::Barometer;
        break;
      default:
        std::stringstream ss;
        ss << "Unsupported field in IMU packet: " << std::hex << d;
        throw std::runtime_error(ss.str());
        break;
      }
    }

    if (imuDataCallback_) {
      imuDataCallback_(data);
    }
  } else if (packet_.isFilterData()) {
    for (int d; (d = decoder.fieldDescriptor()) > 0; decoder.advance()) {
      switch (u8(d)) {
      case FIELD_QUATERNION:
        decoder.extract(4, &filterData.quaternion[0]);
        decoder.extract(1, &filterData.quaternionStatus);
        filterData.fields |= FilterData::Quaternion;
        break;
      case FIELD_GYRO_BIAS:
        decoder.extract(3, &filterData.bias[0]);
        decoder.extract(1, &filterData.biasStatus);
        filterData.fields |= FilterData::Bias;
        break;
      case FIELD_ANGLE_UNCERTAINTY:
        decoder.extract(3, &filterData.angleUncertainty[0]);
        decoder.extract(1, &filterData.angleUncertaintyStatus);
        filterData.fields |= FilterData::AngleUnertainty;
        break;
      case FIELD_BIAS_UNCERTAINTY:
        decoder.extract(3, &filterData.biasUncertainty[0]);
        decoder.extract(1, &filterData.biasUncertaintyStatus);
        filterData.fields |= FilterData::BiasUncertainty;
        break;
      default:
        std::stringstream ss;
        ss << "Unsupported field in filter packet: " << std::hex << d;
        throw std::runtime_error(ss.str());
        break;
      }
    }

    if (filterDataCallback_) {
      filterDataCallback_(filterData);
    }
  } else {
    //  find any NACK fields and log them
    for (int d; (d = decoder.fieldDescriptor()) > 0; decoder.advance()) {
      if (decoder.fieldIsAckOrNack()) {
        uint8_t cmd_code[2];  //  0 = command echo, 1 = command code
        decoder.extract(2, &cmd_code[0]);
        if (cmd_code[1] != 0) {
          //  error occurred
          std::cout << "Received NACK packet (class, command, code): ";
          std::cout << std::hex << static_cast<int>(packet_.descriptor) << ", ";
          std::cout << static_cast<int>(cmd_code[0]) << ", "; 
          std::cout << static_cast<int>(cmd_code[1]) << "\n" << std::flush;
        }
      }
    }
  }
}

int Imu::writePacket(const Packet &p, unsigned int to) {
  using namespace std::chrono;

  //  place into buffer
  std::vector<uint8_t> v;
  v.reserve(Packet::kHeaderLength + sizeof(Packet::payload) + 2);

  v.push_back(p.syncMSB);
  v.push_back(p.syncLSB);
  v.push_back(p.descriptor);
  v.push_back(p.length);
  for (size_t i = 0; i < p.length; i++) {
    v.push_back(p.payload[i]);
  }
  v.push_back(p.checkMSB);
  v.push_back(p.checkLSB);

  auto tstart = high_resolution_clock::now();
  auto tstop = tstart + milliseconds(to);

  size_t written = 0;
  while (written < v.size()) {
    const ssize_t amt = ::write(fd_, &v[written], v.size() - written);
    if (amt > 0) {
      written += amt;
    } else if (amt < 0) {
      if (errno == EAGAIN || errno == EINTR) {
        //  blocked or interrupted - try again until timeout
      } else {
        return -1; //  error while writing
      }
    }

    if (tstop < high_resolution_clock::now()) {
      return 0; //  timed out
    }
  }

  return static_cast<int>(written); //  wrote w/o issue
}

void Imu::sendPacket(const Packet &p, unsigned int to) {
  const int wrote = writePacket(p, to);
  if (wrote < 0) {
    throw io_error(strerror(errno));
  } else if (wrote == 0) {
    throw timeout_error(true,to);
  }
}

void Imu::receiveResponse(const Packet &command, unsigned int to) {
  //  read back response
  const auto tstart = std::chrono::high_resolution_clock::now();
  const auto tend = tstart + std::chrono::milliseconds(to);

  while (std::chrono::high_resolution_clock::now() <= tend) {
    const int resp = pollInput(1);
    if (resp > 0) {
      //  check if this is an ack
      const int ack = packet_.ackErrorCodeFor(command);
      
      if (ack == 0) {
        return; //  success, exit
      } else if (ack > 0) {
        throw command_error(command, ack);
      } else {
        if (verbose_) {
          std::cout << "Not interested in this [N]ACK!\n";
          std::cout << packet_.toString() << "\n";
        }
        //  this ack was not for us, keep spinning until timeout
      }
    } else if (resp < 0) {
      throw io_error(strerror(errno));
    } else {
      //  resp == 0 keep reading until timeout
    }
  }
  if (verbose_) {
    std::cout << "Timed out reading response to:\n";
    std::cout << command.toString() << std::endl << std::flush;
  }
  //  timed out
  throw timeout_error(false, to);  
}

void Imu::sendCommand(const Packet &p, bool readReply) {
  if (verbose_) {
    std::cout << "Sending command:\n";
    std::cout << p.toString() << std::endl;
  }
  sendPacket(p, rwTimeout_);
  if (readReply) {
    receiveResponse(p, rwTimeout_);
  }
}
