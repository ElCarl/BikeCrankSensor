#pragma once

#include <Arduino.h>

class MicrosecondTimer {
  public:
    explicit MicrosecondTimer(const char* name);
    ~MicrosecondTimer();

    // Pauses while printing to avoid skewing results
    void print_time(const char* tag);

  private:
    uint32_t _running_count{0};
    uint32_t _start_us{0};
    uint32_t _chunk_start_us{0};
    const char* _name;
};