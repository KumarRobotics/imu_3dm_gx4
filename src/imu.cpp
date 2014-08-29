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

#include <ros/ros.h>

extern "C" {
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
}

#define kTimeout    (100)
#define kBufferSize (10)  //  for full performance this must be kept SMALL!

#define u8(x) static_cast<uint8_t>((x))

using namespace imu_3dm_gx4;
using namespace std;

// trim from start
static inline std::string ltrim(std::string s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

//  swap order of bytes if on a little endian platform
template <size_t sz>
void to_device_order(uint8_t buffer[sz]) {
#ifdef HOST_LITTLE_ENDIAN
    for (size_t i=0; i < sz/2; i++) {
        std::swap(buffer[i], buffer[sz-i-1]);
    }
#endif
}

template <>
void to_device_order<1>(uint8_t buffer[1]) {}

template <typename T>
size_t encode(uint8_t * buffer, const T& t)
{
    static_assert(std::is_fundamental<T>::value, "Only fundamental types may be encoded");

    const size_t szT = sizeof(T);
    union {
        T tc;
        uint8_t bytes[szT];
    };
    tc = t;

    to_device_order<szT>(bytes);

    //  append
    for (size_t i=0; i < szT; i++) {
        buffer[i] = bytes[i];
    }

    return szT;
}

template <typename T, typename ...Ts>
size_t encode(uint8_t * buffer, const T& t, const Ts&... ts) {

    const size_t sz = encode(buffer, t);

    //  recurse
    return sz + encode(buffer+sizeof(T), ts...);
}

//  note: changed decoder from tuple to array to deal with gcc's bullshit

template <typename T>
void decode(uint8_t * buffer, size_t count, T* output)
{
  const size_t szT = sizeof(T);
  static_assert(std::is_fundamental<T>::value, "Only fundamental types may be decoded");

  union {
      T tc;
      uint8_t bytes[szT];
  };

  for (size_t i=0; i < count; i++)
  {
    for (size_t j=0; j < szT; j++) {
        bytes[j] = buffer[j];
    }
    to_device_order<szT>(&bytes[0]);
    output[i] = tc;

    buffer += szT;
  }
}

bool Imu::Packet::is_ack() const {
    //  length must be 4 for the first field in an ACK
    return (payload[0]==0x04 && payload[1]==0xF1);
}

bool Imu::Packet::is_imu_data() const {
  return descriptor==0x80;
}

bool Imu::Packet::is_filter_data() const {
  return descriptor==0x82;
}

int Imu::Packet::ack_code(const Packet& command) const {

    if ( !is_ack() ) {
        return -1;
    }
    else if (descriptor != command.descriptor) {
        return -1;   //  does not correspond to this command
    }
    else if (payload[2] != command.payload[1]) {
        return -1;   //  first entry in field must match the sent field desc
    }

    //  ack, return error code
    return payload[3];
}

void Imu::Packet::calcChecksum()
{
    uint8_t byte1=0,byte2=0;

#define add_byte(x) byte1 += (x); \
                    byte2 += byte1;

    add_byte(syncMSB);
    add_byte(syncLSB);
    add_byte(descriptor);
    add_byte(length);

    for (size_t i=0; i < length; i++) {
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

Imu::Packet::Packet(uint8_t desc, uint8_t len) : syncMSB(kSyncMSB), syncLSB(kSyncLSB),
    descriptor(desc), length(len), checksum(0)
{
}

Imu::Imu(const std::string& device) : device_(device), fd_(0), state_(Idle)
{
    //  buffer for storing reads
    buffer_.resize(kBufferSize);
}

Imu::~Imu() {
    disconnect();
}

void Imu::connect()
{
    if (fd_ > 0) {
        throw std::runtime_error("Socket is already open");
    }

    const char * path = device_.c_str();
    fd_ = ::open(path, O_RDWR | O_NOCTTY | O_NDELAY); //  read/write, no controlling terminal, non blocking access
    if (fd_ < 0) {
        throw std::runtime_error("Failed to open device : " + device_);
    }
    
    //  make sure it is non-blocking
    if(fcntl(fd_, F_SETFL, FNONBLOCK) < 0) {
      disconnect();
      throw io_error( strerror(errno) );
    }

    struct termios toptions;
    if (tcgetattr(fd_, &toptions) < 0) {
        disconnect();
        throw io_error( strerror(errno) );
    }

    //  set default baud rate
    cfsetispeed(&toptions, B115200);
    cfsetospeed(&toptions, B115200);

    toptions.c_cflag &= ~PARENB;    //  no parity bit
    toptions.c_cflag &= ~CSTOPB;    //  disable 2 stop bits (only one used instead)
    toptions.c_cflag &= ~CSIZE;     //  disable any previous size settings
    toptions.c_cflag |= HUPCL;      //  enable modem disconnect
    toptions.c_cflag |= CS8;        //  bytes are 8 bits long

    toptions.c_cflag &= ~CRTSCTS;  //  no hardware RTS/CTS switch

    toptions.c_cflag |= CREAD | CLOCAL;             //  enable receiving of data, forbid control lines (CLOCAL)
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY);    //  no software flow control

    //  disable the following
    //  ICANON = input is read on a per-line basis
    //  ECHO/ECHOE = echo enabled
    //  ISIG = interrupt signals
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    toptions.c_oflag &= ~OPOST;     //  disable pre-processing of input data

    toptions.c_cc[VMIN] = 0;        //  no minimum number of bytes when reading
    toptions.c_cc[VTIME] = 0;       //  no time blocking

    //  TCSAFLUSH = make change after flushing i/o buffers
    if (tcsetattr(fd_, TCSAFLUSH, &toptions) < 0) {
        disconnect();
        throw io_error( strerror(errno) );
    }
}

void Imu::disconnect()
{
    if (fd_ > 0) {
        //  send the idle command first
        idle(kTimeout);

        close(fd_);
    }
    fd_ = 0;
}

bool Imu::termiosBaudRate(unsigned int baud)
{
    struct termios toptions;
    if (tcgetattr(fd_, &toptions) < 0) {
        return false;
    }

    speed_t speed;
    switch (baud) {
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 115200: speed = B115200; break;
      case 230400: speed = B230400; break;
      case 460800: speed = B460800; break;
      case 921600: speed = B921600; break;
    default:
      assert(false);
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

void Imu::runOnce()
{
    int sig = pollInput(5);
    if (sig < 0) {
        //  failure in poll/read, device disconnected
        throw io_error( strerror(errno) );
    }
}

void Imu::selectBaudRate(unsigned int baud)
{
    //  baud rates supported by the 3DM-GX4-25
    const size_t num_rates = 6;
    unsigned int rates[num_rates] = {9600,19200,115200,230400,460800,921600};

    if ( !std::binary_search(rates, rates + num_rates, baud) ) {
        //  invalid baud rate
        throw std::invalid_argument(string("Baud rate unsupported ") + to_string(baud));
    }

    Imu::Packet pp(0x01, 0x02);
    pp.payload[0] = 0x02;
    pp.payload[1] = 0x01;
    pp.calcChecksum();

    size_t i;
    bool foundRate = false;

    for (i=0; i < num_rates; i++)
    {
        if ( !termiosBaudRate(rates[i]) ) {
            throw io_error(strerror(errno));
        }

        //  send ping and wait for first response
        sendPacket(pp, 100);

        if (receiveResponse(pp, 500) > 0) {
            foundRate = true;
            break;
        }
    }

    if (!foundRate) {
        throw runtime_error("Failed to reach device " + device_);
    }

    //  we are on the correct baud rate, now change to the new rate
    Packet comm(0x0C, 0x07);
    assert( encode(comm.payload, u8(0x07), u8(0x40), u8(0x01), static_cast<uint32_t>(baud)) == 0x07 );
    comm.calcChecksum();

    int code = sendCommand(comm, 300);
    if ( code <= 0 ) {
        throw runtime_error("Device rejected baud rate " + to_string(baud) + " (Code: " + to_string(code) + ")");
    }

    //  device has switched baud rate, now we should also
    if ( !termiosBaudRate(baud) ) {
        throw io_error( strerror(errno) );
    }

    //  ping
    if (ping(300) <= 0) {
        throw runtime_error("Device did not respond to ping");
    }
}

int Imu::ping(unsigned int to)
{
    Imu::Packet p(0x01, 0x02);
    p.payload[0] = 0x02;
    p.payload[1] = 0x01;
    p.calcChecksum();
    assert(p.checkMSB == 0xE0 && p.checkLSB == 0xC6);

    return sendCommand(p, to);
}

int Imu::idle(unsigned int to)
{
    Imu::Packet p(0x01, 0x02);
    p.payload[0] = 0x02;
    p.payload[1] = 0x02;
    p.calcChecksum();
    assert(p.checkMSB == 0xE1 && p.checkLSB == 0xC7);

    return sendCommand(p, to);
}

int Imu::resume(unsigned int to)
{
    Imu::Packet p(0x01, 0x02);
    p.payload[0] = 0x02;
    p.payload[1] = 0x06;
    p.calcChecksum();
    assert(p.checkMSB == 0xE5 && p.checkLSB == 0xCB);

    return sendCommand(p, to);
}

/*int Imu::reset(unsigned int to)
{
    Imu::Packet p(0x01,0x02);
    p.payload[0] = 0x02;
    p.payload[1] = 0x7E;
    p.calcChecksum();
    assert(p.checkMSB == 0x5D && p.checkLSB == 0x43);

    return sendCommand(p, to);
}*/

int Imu::getDeviceInfo(Imu::Info& info)
{
    Imu::Packet p(0x01, 0x02);
    p.payload[0] = 0x02;
    p.payload[1] = 0x03;
    p.calcChecksum();
    assert(p.checkMSB == 0xE2 && p.checkLSB == 0xC8);

    int comm = sendCommand(p, kTimeout);
    if ( comm > 0 ) {

        to_device_order<2>(&packet_.payload[6]);    //  swap firmware ver.
        memcpy(&info.firmwareVersion, &packet_.payload[6], 2);

#define devinfo2str(buf,ofs)  std::string((char*)(buf) + (ofs), 16)

        info.modelName = ltrim( devinfo2str(packet_.payload, 8) );
        info.modelNumber = ltrim( devinfo2str(packet_.payload, 24) );
        info.serialNumber = ltrim( devinfo2str(packet_.payload, 40) );
        info.lotNumber = ltrim( devinfo2str(packet_.payload, 56) );
        info.deviceOptions = ltrim( devinfo2str(packet_.payload, 72) );

#undef devinfo2str

    }

    return comm;
}

int Imu::getIMUDataBaseRate(uint16_t& baseRate)
{
  Packet p(0x0C, 0x02);
  p.payload[0] = 0x02;
  p.payload[1] = 0x06;
  p.calcChecksum();

  int comm = sendCommand(p, kTimeout);
  if (comm > 0)
  {
    decode(&packet_.payload[6],1,&baseRate);
  }
  return comm;
}

int Imu::getFilterDataBaseRate(uint16_t& baseRate)
{
  Packet p(0x0C, 0x02);
  p.payload[0] = 0x02;
  p.payload[1] = 0x0B;
  p.calcChecksum();

  int comm = sendCommand(p, kTimeout);
  if (comm > 0)
  {
    decode(&packet_.payload[6],1,&baseRate);
  }
  return comm;
}

int Imu::getDiagnosticInfo(Imu::DiagnosticFields& fields)
{
  Packet p(0x0C, 0x05);
  p.payload[0] = 0x05;
  p.payload[1] = 0x64;
  encode(&p.payload[2],static_cast<uint16_t>(6234));  //  device model number (0x185A)
  p.payload[4] = 0x02;  //  diagnostic mode
  p.calcChecksum();

  int comm = sendCommand(p, kTimeout);
  if (comm > 0)
  {
    if (packet_.payload[4] != 0x4B) { //  should be full diagnostic report
      ROS_WARN("Wrong diagnostic field length returned");
    }

    decode(&packet_.payload[6],1,&fields.modelNumber);
    decode(&packet_.payload[8],1,&fields.selector);
    decode(&packet_.payload[9],4,&fields.statusFlags);
    decode(&packet_.payload[25],2,&fields.imuStreamEnabled);
    decode(&packet_.payload[27],13,&fields.imuPacketsDropped);
  }
  return comm;
}

template <typename T>
int getNextBit(T& val)
{
    const static int nBits = sizeof(T)*8;
    static_assert(!std::numeric_limits<T>::is_signed &&
                   std::numeric_limits<T>::is_integer, "Type must be a unsigned integer");

    int count = 0;
    T n = val;

    while (!(n & 0x01) && count != nBits) {
        n = n >> 1;
        count++;
    }

    if (count < nBits) {
        val = val & ~(1 << count);
        return count;
    }

    return -1;
}

int Imu::setIMUDataRate(uint16_t decimation, unsigned int sources)
{
    Imu::Packet p(0x0C,0x04);
    p.payload[0] = 0x00;    //  field length
    p.payload[1] = 0x08;
    p.payload[2] = 0x01;    //  function
    p.payload[3] = 0x00;    //  descriptor count

    //  order of fields:
    //  Accelerometer   bit 0
    //  Gyroscope       bit 1
    //  Magnetometer    bit 2
    //  Barometer       bit 3

    //  these correspond to numbers in the device manual
    static const uint8_t fieldDescs[] = {0x04, 0x05, 0x06, 0x17};
    uint8_t descCount = 0;

    unsigned int bPos;
    while ( (bPos = getNextBit(sources)) >= 0 )
    {
      if (bPos >= sizeof(fieldDescs)) {
        throw invalid_argument("Invalid data source requested!");
      }

      descCount++;
      p.length += encode(p.payload + p.length, fieldDescs[bPos], decimation);
    }

    p.payload[3] = descCount;
    p.payload[0] = 4 + 3*descCount;

    p.calcChecksum();
    return sendCommand(p, kTimeout);
}

int Imu::setFilterDataRate(uint16_t decimation, unsigned int sources)
{
    Imu::Packet p(0x0C,0x04);
    p.payload[0] = 0x00;    //  field length
    p.payload[1] = 0x0A;
    p.payload[2] = 0x01;    //  function
    p.payload[3] = 0x00;    //  descriptor count

    static const uint8_t fieldDescs[] = {0x03, 0x06};
    uint8_t descCount = 0;

    unsigned int bPos;
    while ( (bPos = getNextBit(sources)) >= 0 )
    {
      if (bPos >= sizeof(fieldDescs)) {
        throw invalid_argument("Invalid data source requested!");
      }

      descCount++;
      p.length += encode(p.payload + p.length, fieldDescs[bPos], decimation);
    }

    if (sources != 0) {
      throw std::invalid_argument("Invalid data source requested");
    }

    p.payload[3] = descCount;
    p.payload[0] = 4 + 3*descCount;

    p.calcChecksum();
    return sendCommand(p, kTimeout);
}

int Imu::enableMeasurements(bool accel, bool magnetometer)
{
  Imu::Packet p(0x0D, 0x05);
  p.payload[0] = 0x05;
  p.payload[1] = 0x41;
  p.payload[2] = 0x01;  //  apply new settings

  p.payload[3] = 0;
  if (accel) {
    p.payload[3] |= 0x01;
  }
  if (magnetometer) {
    p.payload[3] |= 0x02;
  }

  p.calcChecksum();
  return sendCommand(p, kTimeout);
}

int Imu::enableBiasEstimation(bool enabled)
{
  Imu::Packet p(0x0D, 0x05);
  p.payload[0] = 0x05;
  p.payload[1] = 0x14;
  p.payload[2] = 0x01;
  
  uint16_t flag = 0xFFFE;
  if (enabled) {
    flag = 0xFFFF;
  }
  
  encode(&p.payload[3],flag);
  p.calcChecksum();
  
  return sendCommand(p,kTimeout);
}

int Imu::setHardIronOffset(float offset[3])
{
  Imu::Packet p(0x0C, 0x0F);
  p.payload[0] = 0x0F;
  p.payload[1] = 0x3A;
  p.payload[2] = 0x01;  //  apply settings
  
  encode(&p.payload[3], offset[0], offset[1], offset[2]);
  p.calcChecksum();
  return sendCommand(p,kTimeout);
}

int Imu::setSoftIronMatrix(float matrix[9])
{
  Imu::Packet p(0x0C, 0x27);
  p.payload[0] = 0x27;
  p.payload[1] = 0x3B;
  p.payload[2] = 0x01;  //  apply settings
  
  encode(&p.payload[3], matrix[0], matrix[1], matrix[2], matrix[3],
      matrix[4], matrix[5], matrix[6], matrix[7], matrix[8]);
  
  p.calcChecksum();
  return sendCommand(p,kTimeout);
}

int Imu::enableIMUStream(bool enabled)
{
    Packet p(0x0C, 0x05);
    p.payload[0] = 0x05;
    p.payload[1] = 0x11;
    p.payload[2] = 0x01;    //  apply new settings
    p.payload[3] = 0x01;    //  device: IMU
    p.payload[4] = (enabled == true);

    p.calcChecksum();
    return sendCommand(p, kTimeout);
}

int Imu::enableFilterStream(bool enabled)
{
    Packet p(0x0C, 0x05);
    p.payload[0] = 0x05;
    p.payload[1] = 0x11;
    p.payload[2] = 0x01;
    p.payload[3] = 0x03;    //  device: estimation filter
    p.payload[4] = (enabled == true);

    p.calcChecksum();
    return sendCommand(p, kTimeout);
}

int Imu::pollInput(unsigned int to)
{  
    //  poll socket for inputs
    struct pollfd p;
    p.fd = fd_;
    p.events = POLLIN;
    
    int rPoll = poll(&p, 1, to); // timeout is in millis
    if (rPoll > 0)
    {
        const ssize_t amt = ::read(fd_, &buffer_[0], buffer_.size());
        if (amt > 0) {
            return handleRead(amt);
        } else if (amt == 0) {
            //  end-of-file, device disconnected
            return -1;
        }
    } else if (rPoll == 0) {
        return 0;   //  no socket can be read
    }

    if (errno == EAGAIN || errno == EINTR) {
        //  treat these like timeout errors
        return 0;
    }
    
    //  poll() or read() failed
    return -1;
}

int Imu::handleRead(size_t bytes_transferred)  //  parses packets out of the input buffer
{  
    //  read data into queue
    for (size_t i=0; i < bytes_transferred; i++) {
        queue_.push_back(buffer_[i]);
    }

    if (state_ == Idle) {
        dstIndex_ = 1;  //  reset counts
        srcIndex_ = 0;
    }

    while (srcIndex_ < queue_.size())
    {
        if (state_ == Idle)
        {
            //   waiting for packet
            if (queue_.size() > 1)
            {
                if (queue_[0] == Imu::Packet::kSyncMSB && queue_[1] == Imu::Packet::kSyncLSB)
                {
                    //   found the magic ID
                    packet_.syncMSB = queue_[0];
                    state_ = Reading;

                    //  clear out previous packet content
                    memset(packet_.payload, 0, sizeof(packet_.payload));
                }
            }

            queue_.pop_front();

            dstIndex_ = 1;
            srcIndex_ = 0;
        }
        else if (state_ == Reading)
        {
            uint8_t byte = queue_[srcIndex_];
            const size_t end = Packet::kHeaderLength + packet_.length;

            //  fill out fields of packet structure
            if (dstIndex_ == 1)         packet_.syncLSB = byte;
            else if (dstIndex_ == 2) {
                packet_.descriptor = byte;
            }
            else if (dstIndex_ == 3)    packet_.length = byte;
            else if (dstIndex_ < end)   packet_.payload[dstIndex_ - Packet::kHeaderLength] = byte;
            else if (dstIndex_ == end)  packet_.checkMSB = byte;
            else if (dstIndex_ == end+1)
            {
                state_ = Idle;  //  finished packet

                packet_.checkLSB = byte;

                //  check checksum
                uint16_t sum = packet_.checksum;

                packet_.calcChecksum();

                if (sum != packet_.checksum)
                {
                    //  invalid, go back to waiting for a marker in the stream
                    ROS_WARN("Warning: Dropped packet with mismatched checksum\n");
                }
                else {
                    //  packet is valid, remove relevant data from queue
                    for (size_t k=0; k <= srcIndex_; k++) {
                        queue_.pop_front();
                    }

                    //  read a packet
                    processPacket();
                    return 1;
                }
            }

            dstIndex_++;
            srcIndex_++;
        }
    }

    //  no packet
    return 0;
}

void Imu::processPacket()
{
    IMUData data;
    FilterData filterData;

    if ( packet_.is_imu_data() )
    {
        //  process all fields in the packet
        size_t idx=0;

        while (idx+1 < sizeof(packet_.payload))
        {
            size_t len = packet_.payload[idx];
            size_t ddesc = packet_.payload[idx+1];

            if (!len) {
                break;
            }

            if (ddesc >= 0x04 && ddesc <= 0x06) //  accel/gyro/mag
            {
                float * buf=0;
                switch (ddesc) {
                    case 0x04: buf = data.accel; break;
                    case 0x05: buf = data.gyro;  break;
                    case 0x06: buf = data.mag;   break;
                };
                assert(buf != 0);

                decode(&packet_.payload[idx + 2], 3, buf);
            }
            else if (ddesc == 0x17) //  pressure sensor
            {
                decode(&packet_.payload[idx + 2], 1, &data.pressure);
            }
            else {
                ROS_WARN("Warning: Unsupported data field present in IMU packet");
            }

            idx += len;
        }

        if (this->imuDataCallback) {
            imuDataCallback(data);
        }
    }
    else if ( packet_.is_filter_data() ) {

      size_t idx=0;

      while (idx+1 < sizeof(packet_.payload))
      {
        size_t len = packet_.payload[idx];
        size_t ddesc = packet_.payload[idx+1];

        if (!len) {
            break;
        }

        if (ddesc == 0x03)  //  quaternion
        {
          decode(&packet_.payload[idx+2], 4, filterData.quaternion);
          decode(&packet_.payload[idx+2+sizeof(float)*4], 1, &filterData.quatStatus);
        }
        else if (ddesc == 0x06)
        {
          decode(&packet_.payload[idx+2], 3, filterData.bias);
          decode(&packet_.payload[idx+2+sizeof(float)*3], 1, &filterData.biasStatus);
        }
        else {
          ROS_WARN("Warning: Unsupported data field present in estimator packet");
        }

        idx += len;
      }

      if (this->filterDataCallback) {
        filterDataCallback(filterData);
      }
    }
    else if ( packet_.is_ack() ) {
        if (packet_.payload[3] != 0) {
            ROS_ERROR("Received NACK packet: 0x%02x, 0x%02x, 0x%02x\n", packet_.descriptor, packet_.payload[2], packet_.payload[3]);
        }
    }
}

int Imu::writePacket(const Packet& p, unsigned int to)
{
    using namespace std::chrono;

    //  place into buffer
    std::vector<uint8_t> v;
    v.reserve(Packet::kHeaderLength+sizeof(Packet::payload)+2);

    v.push_back(p.syncMSB);
    v.push_back(p.syncLSB);
    v.push_back(p.descriptor);
    v.push_back(p.length);
    for (size_t i=0; i < p.length; i++) {
        v.push_back(p.payload[i]);
    }
    v.push_back(p.checkMSB);
    v.push_back(p.checkLSB);

    auto tstart = high_resolution_clock::now();
    auto tstop = tstart + milliseconds(to);

    size_t written = 0;
    while ( written < v.size() )
    {
        const ssize_t amt = ::write(fd_, &v[written], v.size() - written);
        if (amt > 0) {
            written += amt;
        } else if (amt < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                //  blocked or interrupted - try again until timeout
            } else {
                return -1;  //  error while writing
            }
        }

        if (tstop < high_resolution_clock::now()) {
            return 0;   //  timed out
        }
    }

    return 1;   //  wrote w/o issue
}

void Imu::sendPacket(const Packet& p, unsigned int to)
{
    const int wrote = writePacket(p, to);
    if (wrote < 0) {
        throw io_error( strerror(errno) );
    }
    else if (wrote == 0) {
        throw timeout_error(p.descriptor, p.payload[1]);
    }
}

int Imu::receiveResponse(const Packet& command, unsigned int to)
{
    //  read back response
    auto tstart = chrono::high_resolution_clock::now();
    auto tend = tstart + chrono::milliseconds(to);

    while (chrono::high_resolution_clock::now() <= tend)
    {
        auto resp = pollInput(1);
        if (resp > 0)
        {
            //  check if this is an ack
            int ack = packet_.ack_code(command);

            if (ack == 0) {
                return 1;
            }
            else if (ack > 0) {
                ROS_WARN("Warning: Received NACK code %x for command {%x, %x}",
                      ack, command.descriptor, command.payload[1]);
                return -ack;
            }
        }
        else if (resp < 0) {
            throw io_error( strerror(errno) );
        }
    }

    //  unreachable
    return 0;
}

int Imu::sendCommand(const Packet &p, unsigned int to)
{
    sendPacket(p, to);
    return receiveResponse(p, to);
}
