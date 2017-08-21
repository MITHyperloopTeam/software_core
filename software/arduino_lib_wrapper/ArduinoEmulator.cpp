
#ifdef __cplusplus
extern "C"{
#endif // __cplusplus
	
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>

#include <lcm/lcm.h>
#include "lcmgen_c/mithl_pin_sim_t.h"

}

// This one has to be in the c++ scope... whyy???
void watchdogSetup(void);
void watchdogSetup(void){
  printf("Watchdog setup\n");
}


#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

// The great prototype wall
static double getUnixTime(void);
void *lcmMonitor(void *plcm);
static void pin_sim_handler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_pin_sim_t * msg, void * user);
void *pinPublisher(void *plcm);

void init(void);
void delay( uint32_t dwMs );
uint32_t digitalRead(int pin);
void attachInterrupt(uint32_t pin, void (*callback)(void), uint8_t mode);
void pinMode(uint32_t pin, uint32_t mode);
void digitalWrite(uint32_t dwPin, uint32_t dwVal);
void analogWrite(uint32_t dwPin, uint32_t dwVal);
uint32_t millis(void);
uint32_t micros(void);
uint32_t analogRead(int pin);
// End the great protoype wall

// global simulation vars, per-module
extern char MODULE_NAME[]; // must be defined in the arduino module being emulated
int16_t analogPinsRead[12];
int16_t analogPinsWrite[12];
int8_t digitalPinsRead[54];
int8_t digitalPinsWrite[54];
bool pinsNeedPublish = false;
lcm_t * lcm;
pthread_t lcmThread;
pthread_t pinPublishThread;
double start_time;

// utilities

static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

void *lcmMonitor(void *plcm) {
  while (1)
    lcm_handle((lcm_t *) plcm);
  return 0;
}

static void pin_sim_handler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_pin_sim_t * msg, void * user){
  for (unsigned int i=0; i<sizeof(msg->analog)/sizeof(msg->analog[0]); i++)
    if (msg->analog[i] >= 0)
      analogPinsRead[i] = msg->analog[i];
  for (unsigned int i=0; i<sizeof(msg->digital)/sizeof(msg->digital[0]); i++)
    if (msg->digital[i] >= 0)
      digitalPinsRead[i] = msg->digital[i];
}

void *pinPublisher(void *plcm){
  mithl_pin_sim_t msg;
  char send_chan[100];
  sprintf(send_chan, "SIM_PINS_WRITE_%s", MODULE_NAME);
  while (1){
    pinsNeedPublish = false;
    for (unsigned int i=0; i<sizeof(msg.analog)/sizeof(msg.analog[0]); i++)
      msg.analog[i] = analogPinsWrite[i];
    for (unsigned int i=0; i<sizeof(msg.digital)/sizeof(msg.digital[0]); i++)
      msg.digital[i] = digitalPinsWrite[i];
    mithl_pin_sim_t_publish((lcm_t *) lcm, send_chan, &msg);
    while (!pinsNeedPublish)
      usleep(333); //TODO(gizatt): what's the right rate here? isn't it sim-speed dependent?
  }
  return 0;
}

// Begin the emulation functions that must match Arduino library function declarations
void init(void){
  printf("Init\n");
  start_time = getUnixTime();
  lcm = lcm_create("udpm://239.255.76.67:62237?ttl=0");
  pthread_create(&lcmThread, NULL, lcmMonitor, lcm);
  pthread_create(&pinPublishThread, NULL, pinPublisher, lcm);
  char name_buf[100];
  sprintf(name_buf, "SIM_PINS_READ_%s", MODULE_NAME);
  mithl_pin_sim_t_subscribe(lcm, name_buf, &pin_sim_handler, NULL);
}

void delay( uint32_t dwMs ){
  usleep(dwMs*1000);
}

void pinMode(uint32_t pin, uint32_t mode){
  printf("Pinmode %d -> %d. TODO: Make this do something.\n", pin, mode);
}

void digitalWrite(uint32_t dwPin, uint32_t dwVal) {
  //printf("Digital Write %d, %d\n", dwPin, dwVal);
  digitalPinsWrite[dwPin] = dwVal;
  pinsNeedPublish = true;
}

// TODO(gizatt): This is a little dangerous,
// as some macros like "A1" resolve to a physical
// pin # (i.e. 55) instead of the analog pin index.
// Those will go out of bounds and break things
void analogWrite(uint32_t dwPin, uint32_t dwVal) {
  //printf("Analog Write %d, %d\n", dwPin, dwVal);
  if (dwPin >= 54)
    dwPin -= 54;
  analogPinsWrite[dwPin] = dwVal;
  pinsNeedPublish = true;
}

uint32_t millis(void ) {
	return (uint32_t)(1000 * (getUnixTime() - start_time));
}

uint32_t micros(void ) {
  return (uint32_t)(1000 * 1000 * (getUnixTime() - start_time));
}

uint32_t analogRead(int pin) {
  if (pin >= 54)
    pin -= 54;
  return analogPinsRead[pin];
}

uint32_t digitalRead(int pin) {
  return digitalPinsRead[pin];
}

class Print
{
  public:
    virtual size_t write(uint8_t);
    size_t write(const char *str) {
      if (str == NULL) return 0;
      return write((const uint8_t *)str, strlen(str));
    }
    virtual size_t write(const uint8_t *buffer, size_t size);
    size_t write(const char *buffer, size_t size) {
      return write((const uint8_t *)buffer, size);
    }
};
size_t Print::write(const uint8_t c){
  printf("%c", c);
  return 1;
};
size_t Print::write(const uint8_t *buffer, size_t size){
  for (size_t i=0; i < size; i++)
    write(buffer[i]);
  return 1;
};

class Stream : public Print {
  public:
    virtual int available() = 0;
    virtual int read() = 0;
};

class HardwareSerial : public Stream {
  public:
    virtual void begin(unsigned long);
    virtual void end();
    virtual int available(void);
};
void HardwareSerial::begin(unsigned long inp){
  printf("HWS begun baud %lu\n", inp);
}
void HardwareSerial::end(){
}
int HardwareSerial::available(){
  return 0;
}

class UARTClass : public HardwareSerial {
  public:
    virtual void begin(const uint32_t dwBaudRate);
    void end(void);
    int available(void);
    int read(void);
    size_t write(const uint8_t c);
    size_t write(const char *str) {
      if (str == NULL) return 0;
      return write((const uint8_t *)str, strlen(str));
    }
    size_t write(const uint8_t *buffer, size_t size);
    size_t write(const char *buffer, size_t size) {
      return write((const uint8_t *)buffer, size);
    }

};

void UARTClass::begin(const uint32_t dwBaudRate){
  printf("UARTClass begun baud %u\n", dwBaudRate);
}

void UARTClass::end(){
  printf("UARTClass end\n");
}
int UARTClass::available(void){
  return 0;
}
int UARTClass::read(void){
  return 0;
}
size_t UARTClass::write(const uint8_t c){
  printf("%c", c);
  return 1;
};
size_t UARTClass::write(const uint8_t *buffer, size_t size){
  for (size_t i=0; i < size; i++)
    write(buffer[i]);
  return 1;
};

class USARTClass : public UARTClass {
public:
    void begin(const uint32_t dwBaudRate);
};

void USARTClass::begin(const uint32_t dwBaudRate){
  printf("USARTClass begun\n");
}

UARTClass Serial;
USARTClass Serial1;
USARTClass Serial2;
USARTClass Serial3;

void attachInterrupt(uint32_t pin, void (*callback)(void), uint8_t mode);
void attachInterrupt(uint32_t pin, void (*callback)(void), uint8_t mode) {
  printf("interrupt created\n");
}

void detachInterrupt(uint32_t pin);
void detachInterrupt(uint32_t pin) {
  printf("interrupt detached\n");
}

#ifdef __cplusplus
}
#endif // __cplusplus
