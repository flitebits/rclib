/*
 * SportSensor.h
 *
 * Created: 3/30/2019 2:29:27 PM
 *  Author: Thomas DeWeese
 */ 


#ifndef SPORTSENSOR_H_
#define SPORTSENSOR_H_

#include "Serial.h"

class SportSensor {
  public:
  enum {
    SENSOR_FCS40 = 0x22,
    SENSOR_FCS150 = 0x67,
    
    DATA_CURRENT_10x  = 0x0200,
    DATA_VOLTAGE_100x = 0x0210
  };  
  SportSensor(Serial* serial, bool invert = true, bool use_alt_pins = false);

  i8_t AddSensor(u8_t sensor_id, u16_t data_id, u16_t poll_time_ms);
  void SetSensor(u8_t sensor_idx, u32_t value);
  void SetSensor(u8_t sensor_id, u16_t data_id, u32_t value);
  void Run();

  void AddFcs40Sensors(i8_t* current_idx, i8_t* volt_idx);
  
  protected:
  struct Entry {
    u8_t most_recent_poll_cnt;
    u8_t poll_interval_100ms;
    u8_t most_recent_update_time;

    u8_t sensor_id;
    u16_t data_id;
    u32_t value;
  };

  void ReadAndProcess();
  void SendNoUpdate(u8_t idx);
  void SendEntry(u8_t idx);
  void SendByte(u8_t val);
  
  Serial* serial_;
  u8_t prev_read_;
  u8_t crc_;
  u8_t poll_cnt_;
  
  Entry entries_[10];
  u8_t num_entries_;  
};



#endif /* SPORTSENSOR_H_ */
