#include "LCDHandler.h"

LCDHandler Display;

void LCDHandler::begin() {
    lcd.init();
    lcd.backlight();
    showMessage("System Starting", "Please Wait...");
}

void LCDHandler::showMessage(String line1, String line2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    if (line2 != "") {
        lcd.setCursor(0, 1);
        lcd.print(line2);
    }
}

void LCDHandler::showStatus(String status) {
    lcd.setCursor(0, 1);
    lcd.print("                "); // Clear line
    lcd.setCursor(0, 1);
    lcd.print(status);
}