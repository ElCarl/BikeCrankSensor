#include "MicrosecondTimer.h"

namespace {
const char* const TAG{"MicrosecondTimer"};
}

MicrosecondTimer::MicrosecondTimer(const char* name) : _name(name) {
    ESP_LOGI(TAG, "Timer %s starting", _name);
    _start_us       = micros();
    _chunk_start_us = _start_us;
}
MicrosecondTimer::~MicrosecondTimer() {
    // Actual timed time
    uint32_t last_chunk_us{micros() - _chunk_start_us};
    _running_count += last_chunk_us;  // Final timer time

    // Overall time including timer logging
    uint32_t total_elapsed_us{micros() - _start_us};

    ESP_LOGI(TAG,
             "Timer %s complete - Timed %u us (Total %u us)",
             _name,
             _running_count,
             total_elapsed_us);
}

// Pauses while printing to avoid skewing results
void MicrosecondTimer::print_time(const char* tag) {
    uint32_t last_chunk_us{micros() - _chunk_start_us};
    _running_count += last_chunk_us;

    // Overall time including timer logging
    uint32_t total_elapsed_us{micros() - _start_us};

    ESP_LOGI(TAG,
             "Timer %s::%s\t - Chunk: %u us, Running: %u us (Total %u us)",
             _name,
             tag,
             last_chunk_us,
             _running_count,
             total_elapsed_us);

    _chunk_start_us = micros();
}
