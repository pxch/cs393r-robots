#ifndef BLOB_H
#define BLOB_H

#include <constants/VisionConstants.h>
#include <vector>
#include <inttypes.h>

struct Blob {
        uint16_t xi, xf, dx, yi, yf, dy;
        uint16_t lpCount;
        std::vector<uint32_t> lpIndex;
        float diffStart;
        float diffEnd;
        float doubleDiff;
        uint16_t widthStart;
        uint16_t widthEnd;
        uint16_t avgX;
        uint16_t avgY;
        float avgWidth;
        uint16_t correctPixelCount;
        float correctPixelRatio;
        bool invalid;

        Blob() :
                        lpIndex(MAX_BLOB_VISIONPOINTS, 0) {
                xi = 0;
                xf = 0;
                yi = 0;
                yf = 0;
                lpCount = 0;
                avgX = 0;
                avgY = 0;
                correctPixelCount = 0;
                correctPixelRatio = 0;
                invalid = false;
        }

        float getRectRatio() {
                if (dx < dy)
                        return float(dy) / float(dx);
                else
                        return float(dx) / float(dy);
        }
};

bool sortBlobAreaPredicate(Blob* left, Blob* right);

#endif
