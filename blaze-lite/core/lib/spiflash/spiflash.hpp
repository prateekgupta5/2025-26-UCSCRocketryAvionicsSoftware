#pragma once
#include <queue>
#include <tuple>
#include <stdint.h>

//common storage interface
#ifndef strg
    #define strg spiflash
#endif

#ifndef KLOG_SIZE
    #define KLOG_SIZE 4098
#endif

namespace spiflash {
    constexpr const uint16_t buffer_size   = 4098;
                    uint16_t buffer_offset = 0;
    
    char writeBuffer[buffer_size];

    std::priority_queue<std::tuple<const uint8_t*, const uint8_t&> queuedwrites;

    //priorities from most to least important
    constexpr const uint8_t
        P_MANDATORY   = 0, //just force writes this at next tick
        P_URGENT      = 1,
        P_IMPORTANT   = 2,
        P_STD         = 3,
        P_UNIMPORTANT = 4,
        P_OPTIONAL    = 5
    ;

    ssize_t read (const size_t offset, const size_t bytes, uint8_t* buffer)  ;

    void queue (const size_t bytes, const uint8_t* data, const uint8_t priority = P_STD /*std priority*/) ;

    ssize_t buffer (const size_t bytes, const uint8_t* data) ;

    ssize_t write (const size_t bytes, const uint8_t* data) ;

    void flush (void) ;

    size_t kLog   (const size_t bytes, const uint8_t* data) ; //just stright up a write
    size_t kWrite (const size_t bytes, const uint8_t* data) ; //just stright up a write
    size_t kFlush (const size_t bytes, const uint8_t* data) ; //just stright up a write

    int  tick (void) ;
}