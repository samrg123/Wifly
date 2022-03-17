#pragma once

#include "util.h"

template<size_t kBannerLen = 80>
struct Banner {
    
    char buffer[kBannerLen+1];

    template<size_t kHeaderN>
    constexpr Banner(const char (&header)[kHeaderN] = "", char fillChar = '-'): buffer{} {
        
        constexpr size_t kHeaderLen = kHeaderN-1;
        static_assert(kBannerLen >= kHeaderLen, "Header is too large for banner buffer");

        char* bufferPos = buffer;

        // build left side of banner
        constexpr int kLeftBannerChars = (kBannerLen-kHeaderLen)/2;
        for(char* end = buffer + kLeftBannerChars; bufferPos < end; ++bufferPos) {
            *bufferPos = fillChar;
        }

        //build middle of banner
        for(int i = 0; i < kHeaderLen; ++i) {
            bufferPos[i] = header[i];
        }
        bufferPos+= kHeaderLen;

        // build right side of banner
        for(char* end = buffer + kBannerLen; bufferPos < end; ++bufferPos) {
            *bufferPos = fillChar;
        }
    }
};
