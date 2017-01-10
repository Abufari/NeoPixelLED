#include <Adafruit_NeoPixel.h>
#include <VirtualWire.h>
#include <Wire.h>
#include "Sodaq_DS3231.h"
#include <LiquidCrystal_I2C.h>
#include <TimeLib.h>

#include <TimerThreading.h>

// Pattern types supported:
enum pattern { NONE, RAINBOW_CYCLE, THEATER_CHASE, COLOR_WIPE, SCANNER, FADE, FLASHING, DIM_WHITE, COLORSET };
// Pattern directions supported:
enum direction { FORWARD, REVERSE };

void turnPowerSupplyOn();
void turnPowerSupplyOff();
bool isBetween(int x, int lower, int upper);

// NeoPattern Class - derived from the Adafruit_NeoPixel class
class NeoPatterns : public Adafruit_NeoPixel
{
public:

    // Member Variables:
    pattern ActivePattern; // which pattern is running
    direction Direction;   // direction to run the pattern

    unsigned long Interval;   // milliseconds between updates
    //unsigned long lastUpdate; // last update of position

    uint32_t Color1, Color2;  // What colors are in use
    uint16_t TotalSteps;  // total number of steps in the pattern
    uint16_t Index;   // current step within the pattern

    bool PowerSupplyOn = false;  // Flag if Power switched already on

    void (*OnComplete)();  // Callback on completion of pattern

    TimerThreading UpdateTimer = TimerThreading(0);

    // Constructor - calls base-class constructor to initialize strip
    NeoPatterns(uint16_t pixels, uint8_t pin, uint8_t type, void (*callback)())
        : Adafruit_NeoPixel(pixels, pin, type)
    {
        OnComplete = callback;
    }

    // Update the pattern
    void Update()
    {
        if (ActivePattern != NONE && !PowerSupplyOn) {
            turnPowerSupplyOn();
            PowerSupplyOn = true;
        }
        else if (ActivePattern == NONE) {
            turnPowerSupplyOff();
            PowerSupplyOn = false;
        }
        //if ((millis() - lastUpdate) > Interval) // time to update
        if (UpdateTimer.isTimeToUpdate())
        {
            //lastUpdate = millis();
            switch (ActivePattern)
            {
            case NONE:
                NoneUpdate();
                break;
            case RAINBOW_CYCLE:
                RainbowCycleUpdate();
                break;
            case THEATER_CHASE:
                TheaterChaseUpdate();
                break;
            case COLOR_WIPE:
                ColorWipeUpdate();
                break;
            case SCANNER:
                ScannerUpdate();
                break;
            case FADE:
                FadeUpdate();
                break;
            case FLASHING:
                FlashingUpdate();
                break;
            case COLORSET:
                break;
            default:
                break;
            }
        }
    }

    // Increment the Index and reset at the end
    void Increment()
    {
        if (Direction == FORWARD)
        {
            Index++;
            if (Index >= TotalSteps)
            {
                Index = 0;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the completion callback
                }
            }
        }
        else // Direction == REVERSE
        {
            --Index;
            if (Index <= 0)
            {
                Index = TotalSteps - 1;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the completion callback
                }
            }
        }
    }

    // Initialize for a None
    void None () {
        ActivePattern = NONE;
        ColorSet(Color(0, 0, 0));
    }

    // Update None
    void NoneUpdate() {
    }

    // Initialize for a RainbowCycle
    void RainbowCycle (uint8_t interval, direction dir = FORWARD)
    {
        if (ActivePattern != RAINBOW_CYCLE) {
            ActivePattern = RAINBOW_CYCLE;
            Interval = interval;
            TotalSteps = 255;
            Index = 0;
            Direction = dir;
            UpdateTimer = TimerThreading(Interval);
        }
    }

    // Update the Rainbow Cycle Pattern
    void RainbowCycleUpdate()
    {
        for (int i = 0; i < numPixels(); i++)
        {
            setPixelColor(i, Wheel(((i * 256 / numPixels()) + Index) & 255));
        }
        show();
        Increment();
    }

    // Initialize for a ColorWipe
    void ColorWipe(uint32_t color, uint8_t interval, direction dir = FORWARD)
    {
        if (ActivePattern != COLOR_WIPE) {
            ActivePattern = COLOR_WIPE;
            Interval = interval;
            TotalSteps = numPixels();
            Color1 = color;
            Index = 0;
            Direction = dir;
            UpdateTimer = TimerThreading(Interval);
        }
    }

    // Update the Color Wipe Pattern
    void ColorWipeUpdate()
    {
        setPixelColor(Index, Color1);
        show();
        Increment();
    }

    // Initialize for a Theater Chase
    void TheaterChase(uint32_t color1, uint32_t color2, uint8_t interval,
                      direction dir = FORWARD)
    {
        ActivePattern = THEATER_CHASE;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color1;
        Color2 = color2;
        Index = 0;
        Direction = dir;
        UpdateTimer = TimerThreading(Interval);
    }

    // Update the Theater Chase Pattern
    void TheaterChaseUpdate()
    {
        for (int i = 0; i < numPixels(); i++)
        {
            if ((i + Index) % 3 == 0)
            {
                setPixelColor(i, Color1);
            }
            else
            {
                setPixelColor(i, Color2);
            }
        }
        show();
        Increment();
    }

    // Initialize for a Scanner
    void Scanner(uint32_t color1, uint8_t interval)
    {
        if (ActivePattern != SCANNER) {
            ActivePattern = SCANNER;
            Interval = interval;
            TotalSteps = (numPixels() - 1) * 2;
            Color1 = color1;
            Index = 0;
            UpdateTimer = TimerThreading(Interval);
        }
    }

    // Update the Scanner Pattern
    void ScannerUpdate()
    {
        for (int i = 0; i < numPixels(); i++)
        {
            if (i == Index) // first half of the scan
            {
                setPixelColor(i, Color1);
            }
            else if (i == TotalSteps - Index) // The return strip
            {
                setPixelColor(i, Color1);
            }
            else // fade to black
            {
                setPixelColor(i, DimColor(getPixelColor(i)));
            }
        }
        show();
        Increment();
    }

    // Initialize for a Fade
    void Fade(uint32_t color1, uint32_t color2, uint16_t steps,
              uint8_t interval, direction dir = FORWARD)
    {
        if (ActivePattern != FADE) {
            ActivePattern = FADE;
            Interval = interval;
            TotalSteps = steps;
            Color1 = color1;
            Color2 = color2;
            Index = 0;
            Direction = dir;
            UpdateTimer = TimerThreading(Interval);
        }
    }

    // Update the Fade Pattern
    void FadeUpdate()
    {
        uint8_t red = ((Red(Color1) * (TotalSteps - Index)) + (Red(Color2) *
                       Index)) / TotalSteps;
        uint8_t green = ((Green(Color1) * (TotalSteps - Index)) + (Green(Color2) *
                         Index)) / TotalSteps;
        uint8_t blue = ((Blue(Color1) * (TotalSteps - Index)) + (Blue(Color2) *
                        Index)) / TotalSteps;
        ColorSet(Color(red, green, blue));
        show();
        Increment();
    }

    void Flashing(uint32_t color1, uint32_t color2, uint8_t interval, direction dir = FORWARD)
    {
        if (ActivePattern != FLASHING) {
            ActivePattern = FLASHING;
            Interval = interval;
            TotalSteps = 470;
            Color1 = color1;
            Color2 = color2;
            Index = 0;
            Direction = dir;
            UpdateTimer = TimerThreading(Interval);
        }
    }

    void FlashingUpdate()
    {
        //double y = _exponential_flashing_function(Index, 200, 0, 400);
        float y = _gaussian_flashing_function(Index, 200);
        uint8_t red = Red(Color1) + (int)round(((int)Red(Color2) - (int)Red(Color1)) * y);
        uint8_t green = Green(Color1) + (int)round(((int)Green(Color2) - (int)Green(Color1)) * y);
        uint8_t blue = Blue(Color1) + (int)round(((int)Blue(Color2) - (int)Blue(Color1)) * y);

        ColorSet(Color(red, green, blue));
        show();
        Increment();
    }

    float _flashing_function(uint8_t x, uint8_t x1, uint8_t x2, uint8_t x3) {
        if (x < x1) {
            return (float)x / x1;
        } else if (isBetween(x, x1, x1 + x2)) {
            return 1;
        } else if (isBetween(x, x1 + x2, x2 + 2 * x1)) {
            return - (1 / (float)x1) * ((float)x - (x2 + 2 * x1));
        } else {
            return 0;
        }
    }

    float _exponential_flashing_function(uint8_t x, uint8_t x1, uint8_t x2, uint8_t x3) {
        double tau = 10;
        if (x < x1 + x2) {
            return (float) exp(((float)x - (x1 + x2)) / tau);
        } else {
            return exp(-((float)x - (x1 + x2)) / (tau));
        }
    }

    float _gaussian_flashing_function(uint8_t x, uint8_t x1) {
        float tau = 80;
        return exp(-((float)x - x1) * ((float)x - x1) / tau);
    }

    void SetSynchronousColor(uint32_t color)
    {
        ActivePattern = COLORSET;
        for (int i = 0; i < numPixels(); i++)
        {
            setPixelColor(i, color);
        }
        show();
    }

    // Returns the Red component of a 32-bit color
    uint8_t Red(uint32_t color)
    {
        return (color >> 16) & 0xFF;
    }

    // Returns the Green component of a 32-bit color
    uint8_t Green(uint32_t color)
    {
        return (color >> 8) & 0xFF;
    }

    // Returns the Green component of a 32-bit color
    uint8_t Blue(uint32_t color)
    {
        return color & 0xFF;
    }

    // Return color, dimmed by 75% (used by scanner)
    uint32_t DimColor(uint32_t color)
    {
        uint32_t dimColor = Color(Red(color) >> 1, Green(color) >> 1,
                                  Blue(color) >> 1);
        return dimColor;
    }

    // Input a value 0 to 255 to get a color value.
    // The colors are a transition r - g - b - back to r.
    uint32_t Wheel(byte WheelPos)
    {
        WheelPos = 255 - WheelPos;
        if (WheelPos < 85)
        {
            return Color(255 - WheelPos * 3, 0, WheelPos * 3);
        }
        else if (WheelPos < 170)
        {
            WheelPos -= 85;
            return Color(0, WheelPos * 3, 255 - WheelPos * 3);
        }
        else
        {
            WheelPos -= 170;
            return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
        }
    }

    // Reverse direction of the pattern
    void Reverse()
    {
        if (Direction == FORWARD)
        {
            Direction = REVERSE;
            Index = TotalSteps - 1;
        }
        else
        {
            Direction = FORWARD;
            Index = 0;
        }
    }

    // Set all pixels to a color (synchronously)
    void ColorSet(uint32_t color)
    {
        for (int i = 0; i < numPixels(); i++)
        {
            setPixelColor(i, color);
        }
        show();
    }
};

void StripComplete();

// Define some NeoPatterns for the two rings and the stick
// and pass the addresses of the associated completion routines
NeoPatterns Strip(60, 6, NEO_GRB + NEO_KHZ800, &StripComplete);
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// receiver stuff
const int led_pin = 13;
const int transmit_pin = 12;
const int receive_pin = 11;
const int transmit_en_pin = 3;
const int powerSupply = 2;
const int buttonPin = 8;

int batteryVoltage;

// Initialize everything and prepare to start
void setup() {
    // Initialize all the pixelStrips
    Strip.begin();
    Serial.begin(9600);
    Wire.begin();
    rtc.begin();
    //lcd.begin(16, 2);
    lcd.begin();

    lcd.setCursor(0, 0);
    lcd.print("Prg");
    lcd.setCursor(6, 0);
    lcd.print("Rcv");
    lcd.setCursor(0, 1);
    /*
    lcd.print('r');
    lcd.setCursor(6, 1);
    lcd.print('g');
    lcd.setCursor(12, 1);
    lcd.print('b');
    */

    // Enable internal pullups on the switch inputs
    pinMode(8, INPUT_PULLUP);
    pinMode(9, INPUT_PULLUP);
    pinMode(led_pin, OUTPUT);
    pinMode(powerSupply, OUTPUT);
    pinMode(receive_pin, INPUT);

    // Kick off a pattern
    Strip.None();

    // receiver
    vw_set_tx_pin(transmit_pin);
    vw_set_rx_pin(receive_pin);
    vw_set_ptt_pin(transmit_en_pin);
    vw_set_ptt_inverted(true);
    vw_setup(2000); // Bits per sec

    vw_rx_start();  // Start the receiver
}

void loop() {
    static bool SwitchOn = checkMotion();
    static bool NightLightFlag = false;
    static bool ShutDownFlag = false;
    static int lastStatus = 0;

    if (ShutDownFlag) {
        Strip.None();
        Strip.Update();
        return;
    }

    static TimerThreading lastCheck = TimerThreading(2000);
    static TimerThreading lcdUpdate = TimerThreading(2000);
    if (lastCheck.isTimeToUpdate()) {
        SwitchOn = checkMotion();
    }
    buttonCheck(&NightLightFlag, &ShutDownFlag);

    Strip.Update();

    if (Strip.ActivePattern == NONE) lcd.setBacklight(LOW);
    else lcd.setBacklight(HIGH);
    /*
    if (lcdUpdate.isTimeToUpdate()) {
        char buf[4] = {'\0'};
        lcd.setCursor(2, 1);
        sprintf(buf, "%3d", Strip.Red(Strip.getPixelColor(0)));
        lcd.print(buf);
        lcd.setCursor(8, 1);
        sprintf(buf, "%3d", Strip.Green(Strip.getPixelColor(0)));
        lcd.print(buf);
        lcd.setCursor(13, 1);
        sprintf(buf, "%3d", Strip.Blue(Strip.getPixelColor(0)));
        lcd.print(buf);
    }
    */
    if (lcdUpdate.isTimeToUpdate()) {
        lcd.setCursor(0, 0);
        lcd.print("Prg");
        lcd.setCursor(6, 0);
        lcd.print("Rcv");
        lcd.setCursor(0, 1);
        char buf[4] = {'\0'};
        sprintf(buf, "%3d", batteryVoltage);
        lcd.setCursor(0,1);
        lcd.print("Volt: ");
        lcd.setCursor(6,1);
        lcd.print(buf);
    }
    DateTime now = rtc.now();
    int hours = now.hour();
    int status = lightStatus(hours, SwitchOn, &NightLightFlag);
    lcd.setCursor(4, 0);
    lcd.print(status);
    //status = 3;
    switch (status) {
    case 0: // full light
        Strip.Flashing(Strip.Color(200, 200, 200), Strip.Color(255, 255, 255), 20);
        break;
    case 1: // rainbow
        Strip.RainbowCycle(200);
        break;
    case 2: // soft light
        //Strip.SetSynchronousColor(Strip.Color(255, 147, 41));
        //Strip.SetSynchronousColor(Strip.Color(255, 193, 37));
        //Strip.SetSynchronousColor(Strip.Color(139, 90, 0)); // dunkelorange
        //Strip.SetSynchronousColor(Strip.Color(142, 40, 142)); // lila
        Strip.SetSynchronousColor(Strip.Color(198, 113, 113)); // salmon
        //Strip.SetSynchronousColor(Strip.Color(178, 58, 238)); // orchid
        /*
          uint8_t r, g, b;
          while (!Serial.available());
          r = (uint8_t)Serial.parseInt();
          while (!Serial.available());
          g = (uint8_t)Serial.parseInt();
          while (!Serial.available());
          b = (uint8_t)Serial.parseInt();
          Strip.SetSynchronousColor(Strip.Color(r,g,b));
        */
        break;
    case 3: // night light
        night_light(status != lastStatus);
        break;
    }
    lastStatus = status;
}

void StripComplete() {
}

int lightStatus(int hours, bool SwitchOn, bool* NightLightFlag) {
    int status = 0;
    if (!SwitchOn) {
        status = 3;
    } else if (isBetween(hours, 8, 16)) {
        status = 0; // full light
        *NightLightFlag = false;
    } else if (*NightLightFlag) {
        status = 3;
    } else if (isBetween(hours, 16, 20)) {
        status = 1; // rainbow
    } else if (isBetween(hours, 20, 23)) {
        status = 2; // soft light
    } else if (isBetween(hours, 23, 24)) {
        status = 3; // night light
    } else if (isBetween(hours, 0, 8)) {
        status = 3; // night light
    }
    return status;
}

bool checkMotion() {
    const unsigned long uptime = 60UL * 60 * 1000;
    static unsigned long lastSwitchOn = 60UL * 60 * 1000;
    if (receiveData()) {
        lastSwitchOn = millis();
        return true;
    }

    if (millis() - lastSwitchOn > uptime) {
        return false;
    }

    byte mins = (byte)((uptime + lastSwitchOn - millis()) / 60000UL);
    lcd.setCursor(14, 0);
    char buf[3];
    sprintf(buf, "%2d", mins);
    lcd.print(buf);
    return true;
}

void night_light(bool reinstantiate) {
    static int status = 2;
    if (reinstantiate) status = 2;
    static unsigned long lastSwitchOn = 0;
    switch (status) {
    case 0:
        if (receiveData()) {
            Strip.Fade(Strip.Color(0, 0, 0), Strip.Color(125, 115, 80), 100, 80);
            status++;
        }
        break;
    case 1:
        if (Strip.Index >= 99) {
            lastSwitchOn = millis();
            Strip.SetSynchronousColor(Strip.Color(125, 115, 80));
            status++;
        }
        break;
    case 2:
        if (millis() - lastSwitchOn > 10000UL) {
            if (receiveData()) {       // light is still needed
                lastSwitchOn = millis();
                break;
            }
            Strip.Fade(Strip.Color(125, 115, 80), Strip.Color(0, 0, 0), 100, 80);
            status++;
        }
        break;
    case 3:
        if (receiveData()) {
            lastSwitchOn = millis();
            Strip.SetSynchronousColor(Strip.Color(125, 115, 80));
            status = 2;
            break;
        }
        if (Strip.Index >= 99) {
            Strip.SetSynchronousColor(Strip.Color(0, 0, 0));
            Strip.None();
            status = 0;
        }
        break;
    }
}

bool receiveData() {
    digitalWrite(led_pin, HIGH);
    uint8_t buf[VW_MAX_MESSAGE_LEN];
    uint8_t buflen = VW_MAX_MESSAGE_LEN;
    //vw_wait_rx_max(400); // wait for transmitter to send
    unsigned long waitMessage = millis();
    bool success = false;
    while (!success && millis() - waitMessage < 100) {
        success = vw_get_message(buf, &buflen);
    }
    if (success) {
        char lcd_buf[4];
        sprintf(lcd_buf, "%1d", buf[0]);
        lcd.setCursor(10, 0);
        lcd.print(lcd_buf);
        digitalWrite(led_pin, LOW);
        batteryVoltage = map(buf[1], 0, 255, 300, 430);
        if (buf[0] == 1)
            return true;
        else
            return false;
    } else {
        lcd.setCursor(10, 0);
        lcd.print("0");
        digitalWrite(led_pin, LOW);
        return false;
    }
}

void turnPowerSupplyOn() {
    digitalWrite(powerSupply, HIGH);
    delay(2000);                     // ground power-on wire and wait for it to start
}

void turnPowerSupplyOff() {
    digitalWrite(powerSupply, LOW);
}

bool isBetween(int number, int lower, int upper) { // a inclusive, b exclusive
    if ((unsigned)(number - lower) < (upper - lower))
        return true;
    else return false;
}

uint8_t buttonCheck(bool* NightLightFlag, bool* ShutDownFlag) {
    byte buttonVal = digitalRead(buttonPin);
    static byte buttonLast = HIGH;
    static long btnDnTime = -1;
    static long btnUpTime = -1;
    static bool ignoreUp = false;
    const long debounce = 20;
    const int holdTime = 2000;

    if (buttonVal == LOW && buttonLast == HIGH && (millis() - btnUpTime) > debounce) {
        btnDnTime = millis();
    }

    if (buttonVal == HIGH && buttonLast == LOW && (millis() - btnDnTime) > debounce) {
        if (!ignoreUp) *NightLightFlag = true;
        else ignoreUp = false;
        btnUpTime = millis();
    }

    if (buttonVal == LOW && (millis() - btnDnTime) > holdTime) {
        *ShutDownFlag = true;
        ignoreUp = true;
        btnDnTime = millis();
    }
    buttonLast = buttonVal;
}

