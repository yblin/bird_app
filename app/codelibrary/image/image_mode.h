//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_IMAGE_MODE_H_
#define CODELIBRARY_IMAGE_IMAGE_MODE_H_

#include "codelibrary/base/log.h"

namespace cl {

/**
 * Define different image modes.
 */
struct ImageMode {
public:
    enum Mode {
        L,             // 1 channel
        RGB,           // 3 channels
        RGBA,          // 4 channels
        MULTI_CHANNEL, // Generic multi-channel (more than 1)
        UNKNOWN        // Unknown mode
    };

    explicit ImageMode(Mode mode = UNKNOWN)
        : mode_(mode) {}

    /**
     * Guess a mode according to the number of channels.
     */
    explicit ImageMode(int n_channels) {
        CHECK(n_channels >= 0);

        switch(n_channels) {
        case 0:
            mode_ = UNKNOWN;
            break;
        case 1:
            mode_ = L;
            break;
        case 3:
            mode_ = RGB;
            break;
        case 4:
            mode_ = RGBA;
            break;

        default:
            mode_ = MULTI_CHANNEL;
        }
    }

    ImageMode& operator=(Mode mode) {
        mode_ = mode;
        return *this;
    }

    bool operator == (Mode mode) const {
        return mode_ == mode;
    }

    bool operator != (Mode mode) const {
        return mode_ != mode;
    }

    /**
     * Get a string representation of the image mode.
     */
    std::string to_string() const {
       switch (mode_) {
       case L:             return "Grayscale";
       case RGB:           return "RGB";
       case RGBA:          return "RGBA";
       case MULTI_CHANNEL: return "Multi-Channel";
       default:            return "Unknown";
       }
    }

    /**
     * Get the number of channels for the given mode.
     * Return 0 if we can not determine the number of channels from the mode.
     */
    int n_channels() const {
        switch (mode_) {
        case L:         return 1;
        case RGB:       return 3;
        case RGBA:      return 4;
        default:        return 0;
        }
    }

    /**
     * Check if the image mode is compatible with the given number of channels.
     */
    bool is_compatible(int n_channels) const {
        CHECK(n_channels >= 0);

        if (n_channels == 0) {
            return mode_ == UNKNOWN;
        }

        switch(n_channels) {
            case 1:  return mode_ == L;
            case 3:  return mode_ == RGB;
            case 4:  return mode_ == RGBA;
            default: return mode_ == MULTI_CHANNEL;
        }
        return false;
    }

    Mode mode() const {
        return mode_;
    }

private:
    Mode mode_;
};

} // namespace cl

#endif // CODELIBRARY_IMAGE_IMAGE_MODE_H_
