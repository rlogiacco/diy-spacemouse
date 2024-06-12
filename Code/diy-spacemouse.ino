
#include <TinyUSB_Mouse_and_Keyboard.h>
#include <OneButton.h>
#include <Tlv493d.h>
#include <SimpleKalmanFilter.h>
#include <SerialDebug.h>
#include <Adafruit_NeoPixel.h>

//#define SERIAL_DEBUG false

//Adafruit_NeoPixel onboard(1, 16, NEO_GBRW + NEO_KHZ800);
Adafruit_NeoPixel pixels(3, 3, NEO_GRB + NEO_KHZ800);

Tlv493d mag = Tlv493d();

struct { SimpleKalmanFilter x,y,z;} filter = {
      SimpleKalmanFilter(1, 1, 0.2),
      SimpleKalmanFilter(1, 1, 0.2),
      SimpleKalmanFilter(1, 1, 0.2)
};

// Setup buttons
OneButton left(0, true);
OneButton right(1, true);
#define INTERVAL 10000
unsigned long boot;

struct coords { float x,y,z; };

coords offset = {0,0,0};
coords current = {0,0,0};

int calSamples = 300;
int sensivity = 8;
int magRange = 3;
int outRange = 127;      // Max allowed in HID report
float xyThreshold = 0.4; // Center threshold

int inRange = magRange * sensivity;
float zThreshold = xyThreshold * 1.5;

bool isOrbit = false;

void setup() {
  SERIAL_DEBUG_SETUP(9600);
  pixels.begin();
  pixels.fill(pixels.Color(255,0,0));
  pixels.show();

  left.attachClick(goHome);
  left.attachLongPressStart(bootsel);
  left.attachLongPressStop(goHome);

  right.attachClick(fitToScreen);
  right.attachLongPressStart(bootsel);
  right.attachLongPressStop(fitToScreen);

  // mouse and keyboard init
  Mouse.begin();
  Keyboard.begin();

  Wire1.setSDA(10);
  Wire1.setSCL(11);
  Wire1.begin();

  // mag sensor init
  mag.begin(Wire1);
  mag.setAccessMode(mag.MASTERCONTROLLEDMODE);
  mag.disableTemp();
  
  // crude offset calibration on first boot
  pixels.fill(pixels.Color(0,0,255));
  pixels.show();
  DEBUG("Auto calibration in progress...");
  for (int i = 1; i <= calSamples; i++) {
    delay(mag.getMeasurementDelay());
    mag.updateData();
    offset = {offset.x + mag.getX(), offset.y + mag.getY(), offset.z + mag.getZ()};
  }

  offset = { offset.x / calSamples, offset.y / calSamples, offset.z / calSamples };

  pixels.fill(pixels.Color(255,0,255));
  pixels.show();
  DEBUG("Calibration completed!");
  DEBUG("X", offset.x, "Y", offset.y, "Z", offset.z);
  boot = millis();
}

void loop() {
  // keep watching the push buttons
  left.tick();
  right.tick();

  if (millis() - boot > INTERVAL) {
    left.attachLongPressStart(NULL);
    right.attachLongPressStart(NULL);
    pixels.fill(pixels.Color(0,255,0));
    pixels.show();
  }

  // get the mag data
  delay(mag.getMeasurementDelay());
  mag.updateData();

  // update the filters
  current = { 
      filter.x.updateEstimate(mag.getX() - offset.x), 
      filter.y.updateEstimate(mag.getY() - offset.y),
      filter.z.updateEstimate(mag.getZ() - offset.z)
    };

  // check the center threshold
  if (abs(current.x) > xyThreshold || abs(current.y) > xyThreshold) {
    
    // map the magnetometer xy to the allowed 127 range in HID reports
    int xMove = map(current.x, -inRange, inRange, -outRange, outRange);
    int yMove = map(current.y, -inRange, inRange, -outRange, outRange);

    // press shift to orbit in Fusion 360 if the pan threshold is not crossed (zAxis)
    if (abs(current.z) < zThreshold && !isOrbit) {
      Keyboard.press(KEY_LEFT_SHIFT);
      isOrbit = true;
    }

    // pan or orbit by holding the middle mouse button and moving proportionally to the xy axis
    Mouse.press(MOUSE_MIDDLE);
    Mouse.move(yMove, xMove, 0);
  } else {

    // release the mouse and keyboard if within the center threshold
    Mouse.release(MOUSE_MIDDLE);
    Keyboard.releaseAll();
    isOrbit = false;
  }
//  DEBUG("X", current.x, "Y", current.y, "Z", current.z);
}

// go to home view in Fusion 360 by pressing  (CMD + SHIFT + H) shortcut assigned to the custom Add-in command
void goHome() {
  Keyboard.press(KEY_LEFT_GUI);
  Keyboard.press(KEY_LEFT_SHIFT);
  Keyboard.write('h');
  delay(10);
  Keyboard.releaseAll();

  DEBUG("Pressed HOME");
}

// fit to view by pressing the middle mouse button twice
void fitToScreen() {
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);

  DEBUG("Pressed FIT");
}

// reboot into programming mode
void bootsel() {
  reset_usb_boot(0, 0);
}
