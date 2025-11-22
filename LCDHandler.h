#ifndef LCDHANDLER_H
#define LCDHANDLER_H

#include <LiquidCrystal_I2C.h>
#include "Config.h"

class LCDHandler {
public:
    void begin();
    void showMessage(String line1, String line2 = "");
    void showStatus(String status);
private:
    LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);
};

extern LCDHandler Display;

#endif