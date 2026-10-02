/*
  UQ Micromouse 2026 – ESP32 firmware
  -----------------------------------
  Hardware (matches the wiring list):
    Motor driver  DIR/PWM type (pins VCC GND DIR2 PWM2 PWM1 DIR1)
                  PWM1 25, DIR1 26 (left)   PWM2 27, DIR2 13 (right)
    Encoders      left A/B 34/35          right A/B 36/39
    I2C bus 1     SDA 21, SCL 22  -> 3x VL53L0X
    VL53L0X XSHUT front 16, left 17, right 18
    I2C bus 2     SDA 32, SCL 33  -> colour sensor (TCS34725)
    Button        GPIO 0 (the BOOT button on the ESP32 board)
    Status LED    GPIO 2 (the blue LED on the ESP32 board)

  Libraries (Arduino IDE > Library Manager):
    - "VL53L0X" by Pololu
    - "Adafruit TCS34725"
  Board: "ESP32 Dev Module" (esp32 core 2.x or 3.x both work)

  No display: status goes to the blue LED and to Serial Monitor (115200 baud).
    LED blinks the mode number: 1 = Explore, 2 = Speed run, 3 = Sensors,
    4 = Test, 5 = Clear map. Solid on = running. Fast flicker = goal reached.

  Using it:
    Short press BOOT = next mode, long press (hold ~1 s, then release) = start.
    Modes:
      Explore   – flood-fill search to the centre, maps as it goes, saves map
      Speed run – uses the saved map for the fastest known path
      Sensors   – live ToF / colour / encoder readings (for calibration)
      Test      – checks motor direction, 1 cell straight, 4 right turns
      Clear map – wipe the saved map (do this once the real maze is revealed!)
    Press BOOT during any run to abort.

  Calibration values are all in the CALIBRATION section below. Changing those
  between runs is calibration, not a change to the navigation algorithm.
*/

#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <Preferences.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ======================= PINS =======================
#define L_PWM    25   // driver PWM1
#define L_DIR    26   // driver DIR1
#define R_PWM    27   // driver PWM2
#define R_DIR    13   // driver DIR2
#define L_ENC_A  34
#define L_ENC_B  35
#define R_ENC_A  36
#define R_ENC_B  39
#define XSHUT_F  16
#define XSHUT_L  17
#define XSHUT_R  18
#define I2C1_SDA 21
#define I2C1_SCL 22
#define I2C2_SDA 32
#define I2C2_SCL 33
#define BTN_PIN  0
#define LED_PIN  2

// ======================= CALIBRATION =======================
// Set to 0 if your motors have no encoders (falls back to timed moves).
#define USE_ENCODERS 1

// Motors: BJ-N20-12-1000 (12 V, 1:30 gearbox, 7 PPR encoder)
//   Blue=GND  Green=Enc A  White=Enc B  Yellow=3.3V  Black=Motor-  Red=Motor+
const float WHEEL_DIA_MM         = 32.0;   // 32 mm wheels with silicone tyres
const float COUNTS_PER_WHEEL_REV = 420.0;  // 7 pulses x 2 edges x 30:1 gearbox
const float TRACK_MM             = 90.0;   // MEASURE: distance between the two wheel centres
const float TURN_SCALE           = 1.00;   // >1 if turns come up short, <1 if they overshoot
const float CELL_MM              = 180.0;

// Flip to -1 if a wheel spins the wrong way / an encoder counts backwards
const int L_MOTOR_DIR = 1,  R_MOTOR_DIR = 1;
const int L_ENC_DIR   = 1,  R_ENC_DIR   = 1;

// Timed fallback (only used when USE_ENCODERS is 0)
const int MS_PER_CELL = 900;   // at EXPLORE_PWM
const int MS_PER_90   = 350;   // at TURN_PWM

// Motor power (0-255). The N20s are 12 V motors on a 6-9 V pack, so full PWM is safe.
const int PWM_FREQ    = 10000;
const int MAX_PWM     = 255;
const int MIN_PWM     = 80;    // lowest value that still moves the mouse
const int EXPLORE_PWM = 150;
const int SPEED_PWM   = 210;
const int TURN_PWM    = 130;

// Wall sensing (mm, as read by each ToF). Use Sensors mode to measure.
const int FRONT_WALL_MM    = 120; // closer than this = wall ahead
const int FRONT_CENTRED_MM = 35;  // front reading when centred in a cell facing a wall
const int SIDE_WALL_MM     = 110; // closer than this = wall at the side
const int SIDE_CENTRED_MM  = 40;  // side reading when centred in a cell
const int TOF_OFFSET_F = 0, TOF_OFFSET_L = 0, TOF_OFFSET_R = 0;

// Steering gains
const float K_WALL = 1.2;  // centring between walls
const float K_ENC  = 2.0;  // keep straight from encoders when no walls

// Colour sensor
const bool  COLOUR_GOAL   = false; // true = a coloured/bright tile also counts as the goal
const float COLOUR_FACTOR = 2.0;   // tile counts as "marked" if this many times brighter than the floor

const bool RETURN_TO_START = true; // after reaching the centre, explore back to start (maps more)

// ======================= MAZE =======================
const int N = 9;
const int GOAL_X = 4, GOAL_Y = 4;
const int DX[4] = {0, 1, 0, -1};   // 0=N 1=E 2=S 3=W
const int DY[4] = {1, 0, -1, 0};

uint8_t walls[N][N];   // bit d set = wall on side d
uint8_t known[N][N];   // bit d set = side d has been seen
uint8_t distv[N][N];
int mx = 0, my = 0, hd = 0;   // position + heading. Start = (0,0) facing N, outer walls left & behind.

// ======================= OBJECTS =======================
VL53L0X tofF, tofL, tofR;
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_16X);
Preferences prefs;

int  distF = 2000, distL = 2000, distR = 2000;
bool tofOK[3] = {false, false, false};
bool colourOK = false;
uint16_t floorClear = 0;
uint16_t cr, cg, cb, cc;

volatile long rawL = 0, rawR = 0;
volatile bool aborted = false;

// ======================= PWM / MOTORS =======================
#if ESP_ARDUINO_VERSION_MAJOR < 3
int pwmChan[40];
int nextChan = 0;
#endif

void pwmInit(int pin) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, 8);
#else
  pwmChan[pin] = nextChan;
  ledcSetup(nextChan, PWM_FREQ, 8);
  ledcAttachPin(pin, nextChan);
  nextChan++;
#endif
}

void pwmOut(int pin, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(pwmChan[pin], duty);
#endif
}

// DIR/PWM driver: DIR pin sets direction, PWM pin sets speed
void motorRaw(int pwmPin, int dirPin, int pwm) {
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
  digitalWrite(dirPin, pwm >= 0 ? HIGH : LOW);
  pwmOut(pwmPin, abs(pwm));
}

int lastL = 0, lastR = 0;

void setMotors(int l, int r) {
  lastL = l; lastR = r;
  motorRaw(L_PWM, L_DIR, l * L_MOTOR_DIR);
  motorRaw(R_PWM, R_DIR, r * R_MOTOR_DIR);
}

// Short reverse pulse to stop quickly, then off
void brake() {
  int l = lastL, r = lastR;
  setMotors(l > 0 ? -60 : (l < 0 ? 60 : 0), r > 0 ? -60 : (r < 0 ? 60 : 0));
  delay(25);
  setMotors(0, 0);
  delay(35);
}

// ======================= ENCODERS =======================
void IRAM_ATTR isrL() { if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) rawL--; else rawL++; }
void IRAM_ATTR isrR() { if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) rawR--; else rawR++; }
long encL() { return rawL * L_ENC_DIR; }
long encR() { return rawR * R_ENC_DIR; }
long encDiff() { return encL() - encR(); }
void encReset() { noInterrupts(); rawL = 0; rawR = 0; interrupts(); }
float countsPerMm() { return COUNTS_PER_WHEEL_REV / (PI * WHEEL_DIA_MM); }

// ======================= ABORT / BUTTON =======================
bool checkAbort() {
  if (!aborted && digitalRead(BTN_PIN) == LOW) { aborted = true; setMotors(0, 0); }
  return aborted;
}

// 0 = nothing, 1 = short press, 2 = long press
int readButton() {
  if (digitalRead(BTN_PIN) == HIGH) return 0;
  unsigned long t0 = millis();
  while (digitalRead(BTN_PIN) == LOW) delay(5);
  delay(30);
  return (millis() - t0 > 800) ? 2 : 1;
}

void waitRelease() { while (digitalRead(BTN_PIN) == LOW) delay(5); delay(50); }

// ======================= STATUS (Serial + LED) =======================
void show(const char* a, const char* b = "", const char* c = "", const char* d = "") {
  Serial.printf("%s | %s | %s | %s\n", a, b, c, d);
}

void led(bool on) { digitalWrite(LED_PIN, on ? HIGH : LOW); }

void blinkCount(int n) {
  for (int i = 0; i < n; i++) { led(true); delay(180); led(false); delay(180); }
}

void flicker(int ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)ms) { led(true); delay(50); led(false); delay(50); }
}

// ======================= TOF SENSORS =======================
bool bringUpToF(int pin, VL53L0X &s, uint8_t addr) {
  digitalWrite(pin, HIGH);
  delay(10);
  s.setTimeout(100);
  if (!s.init()) return false;
  s.setAddress(addr);
  s.setMeasurementTimingBudget(20000);
  s.startContinuous(0);
  return true;
}

void initToF() {
  tofOK[0] = bringUpToF(XSHUT_F, tofF, 0x30);
  tofOK[1] = bringUpToF(XSHUT_L, tofL, 0x31);
  tofOK[2] = bringUpToF(XSHUT_R, tofR, 0x32);
}

// Non-blocking read: returns true if a new value arrived
bool pollToF(VL53L0X &s, int &out, int offset) {
  if ((s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return false;
  uint16_t r = s.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  s.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  out = (r == 0 || r >= 2000) ? 2000 : (int)r + offset;
  return true;
}

void updateToF() {
  if (tofOK[0]) pollToF(tofF, distF, TOF_OFFSET_F);
  if (tofOK[1]) pollToF(tofL, distL, TOF_OFFSET_L);
  if (tofOK[2]) pollToF(tofR, distR, TOF_OFFSET_R);
}

void settleToF(int ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)ms) { updateToF(); delay(3); }
}

// ======================= COLOUR SENSOR =======================
void readColour() { if (colourOK) tcs.getRawData(&cr, &cg, &cb, &cc); }

void calibrateFloor() {
  if (!colourOK) return;
  uint32_t sum = 0;
  for (int i = 0; i < 4; i++) { readColour(); sum += cc; }
  floorClear = sum / 4;
}

bool onMarkedTile() {
  if (!colourOK || floorClear == 0) return false;
  readColour();
  return cc > floorClear * COLOUR_FACTOR && cc > floorClear + 40;
}

const char* colourName() {
  if (!colourOK) return "none";
  readColour();
  if (floorClear && cc < floorClear * COLOUR_FACTOR) return "floor";
  uint16_t mxc = max(cr, max(cg, cb)), mnc = min(cr, min(cg, cb));
  if (mxc < mnc * 1.3) return "white";
  if (mxc == cr) return "red";
  if (mxc == cg) return "green";
  return "blue";
}

// ======================= MAZE LOGIC =======================
bool inMaze(int x, int y) { return x >= 0 && y >= 0 && x < N && y < N; }

void setWall(int x, int y, int d, bool w) {
  int nx = x + DX[d], ny = y + DY[d];
  if (!inMaze(nx, ny)) w = true;            // never "remove" an outer wall
  known[x][y] |= (1 << d);
  if (w) walls[x][y] |= (1 << d); else walls[x][y] &= ~(1 << d);
  if (inMaze(nx, ny)) {
    int od = (d + 2) % 4;
    known[nx][ny] |= (1 << od);
    if (w) walls[nx][ny] |= (1 << od); else walls[nx][ny] &= ~(1 << od);
  }
}

void mazeReset() {
  memset(walls, 0, sizeof(walls));
  memset(known, 0, sizeof(known));
  for (int i = 0; i < N; i++) {
    setWall(i, 0, 2, true);
    setWall(i, N - 1, 0, true);
    setWall(0, i, 3, true);
    setWall(N - 1, i, 1, true);
  }
}

bool canGo(int x, int y, int d, bool unknownBlocked) {
  if (walls[x][y] & (1 << d)) return false;
  if (unknownBlocked && !(known[x][y] & (1 << d))) return false;
  return inMaze(x + DX[d], y + DY[d]);
}

void flood(bool toStart, bool unknownBlocked) {
  memset(distv, 255, sizeof(distv));
  uint8_t qx[N * N], qy[N * N];
  int h = 0, t = 0;
  int tx = toStart ? 0 : GOAL_X, ty = toStart ? 0 : GOAL_Y;
  distv[tx][ty] = 0; qx[t] = tx; qy[t] = ty; t++;
  while (h < t) {
    int x = qx[h], y = qy[h]; h++;
    for (int d = 0; d < 4; d++) {
      if (!canGo(x, y, d, unknownBlocked)) continue;
      int nx = x + DX[d], ny = y + DY[d];
      if (distv[nx][ny] == 255) { distv[nx][ny] = distv[x][y] + 1; qx[t] = nx; qy[t] = ny; t++; }
    }
  }
}

// Lowest-distance neighbour. Ties prefer straight, then right, left, back.
int bestDir(bool unknownBlocked) {
  const int order[4] = {0, 1, 3, 2};
  int best = -1, bd = 255;
  for (int k = 0; k < 4; k++) {
    int d = (hd + order[k]) % 4;
    if (!canGo(mx, my, d, unknownBlocked)) continue;
    int v = distv[mx + DX[d]][my + DY[d]];
    if (v < bd) { bd = v; best = d; }
  }
  return best;
}

int knownCells() {
  int n = 0;
  for (int x = 0; x < N; x++) for (int y = 0; y < N; y++) if (known[x][y] == 0x0F) n++;
  return n;
}

void saveMaze() {
  prefs.begin("mouse", false);
  prefs.putBytes("walls", walls, sizeof(walls));
  prefs.putBytes("known", known, sizeof(known));
  prefs.end();
}

void loadMaze() {
  mazeReset();
  prefs.begin("mouse", true);
  if (prefs.getBytesLength("walls") == sizeof(walls)) {
    prefs.getBytes("walls", walls, sizeof(walls));
    prefs.getBytes("known", known, sizeof(known));
  }
  prefs.end();
}

void senseWalls() {
  settleToF(70);
  setWall(mx, my, hd,           distF < FRONT_WALL_MM);
  setWall(mx, my, (hd + 3) % 4, distL < SIDE_WALL_MM);
  setWall(mx, my, (hd + 1) % 4, distR < SIDE_WALL_MM);
}

// ======================= MOTION =======================
// Drives forward up to mm. Stops early at a front wall. Returns mm actually travelled.
float driveStraight(float mm, int topPwm) {
  if (aborted) return 0;
  encReset();
  long diffHold = 0;
  float done = 0;
  unsigned long t0 = millis();
  unsigned long tTotal = (unsigned long)(mm / CELL_MM * MS_PER_CELL * (float)EXPLORE_PWM / topPwm);

  while (true) {
    if (checkAbort()) return done;
    updateToF();

#if USE_ENCODERS
    done = ((encL() + encR()) / 2) / countsPerMm();
#else
    done = mm * (float)(millis() - t0) / tTotal;
#endif
    float remaining = mm - done;
    if (remaining <= 0) break;
    if (distF <= FRONT_CENTRED_MM) break;

    // speed profile: ramp up, ramp down
    int pwm = topPwm;
    pwm = min(pwm, (int)(MIN_PWM + done * 3));
    pwm = min(pwm, (int)(MIN_PWM + remaining * 2));
    pwm = max(pwm, MIN_PWM);

    // steering
    bool lw = distL < SIDE_WALL_MM, rw = distR < SIDE_WALL_MM;
    float corr = 0;
    if (lw && rw)  { corr = K_WALL * (distL - distR) / 2.0; diffHold = encDiff(); }
    else if (lw)   { corr = K_WALL * (distL - SIDE_CENTRED_MM); diffHold = encDiff(); }
    else if (rw)   { corr = K_WALL * (SIDE_CENTRED_MM - distR); diffHold = encDiff(); }
#if USE_ENCODERS
    else           { corr = K_ENC * (encDiff() - diffHold); }
#endif
    corr = constrain(corr, -40, 40);
    setMotors(pwm - (int)corr, pwm + (int)corr);
    delay(2);
  }
  brake();
  return done;
}

// Square up to a wall in front using the front ToF
void alignFront() {
  unsigned long t0 = millis();
  while (millis() - t0 < 500) {
    if (checkAbort()) return;
    updateToF();
    int e = distF - FRONT_CENTRED_MM;
    if (abs(e) <= 3) break;
    int p = constrain(e * 3, -90, 90);
    int floorP = MIN_PWM - 10;
    if (abs(p) < floorP) p = (p > 0) ? floorP : -floorP;
    setMotors(p, p);
    delay(5);
  }
  brake();
}

// q = +1 right 90, -1 left 90, 2 = 180
void turnQuarter(int q) {
  if (aborted || q == 0) return;
  float deg = q * 90.0;
  int dir = deg > 0 ? 1 : -1;
  encReset();
#if USE_ENCODERS
  float arc = PI * TRACK_MM * fabs(deg) / 360.0 * TURN_SCALE;
  long target = (long)(arc * countsPerMm());
  while (true) {
    if (checkAbort()) return;
    long done = (labs(encL()) + labs(encR())) / 2;
    if (done >= target) break;
    long rem = target - done;
    int p = min(TURN_PWM, (int)(MIN_PWM + rem * 2));
    setMotors(dir * p, -dir * p);
    delay(2);
  }
#else
  unsigned long t = (unsigned long)(MS_PER_90 * fabs(deg) / 90.0 * TURN_SCALE);
  unsigned long t0 = millis();
  while (millis() - t0 < t) {
    if (checkAbort()) return;
    setMotors(dir * TURN_PWM, -dir * TURN_PWM);
    delay(2);
  }
#endif
  brake();
  delay(80);
}

void faceDir(int d) {
  int rel = (d - hd + 4) % 4;
  if (rel == 1) turnQuarter(1);
  else if (rel == 3) turnQuarter(-1);
  else if (rel == 2) turnQuarter(2);
  hd = d;
}

// Drive n cells in the current heading, update position. Returns cells moved.
int forwardCells(int n, int pwm) {
  float mm = driveStraight(n * CELL_MM, pwm);
  int moved = (int)(mm / CELL_MM + 0.5);
  if (moved > n) moved = n;
  if (moved == 0 && n > 0) setWall(mx, my, hd, true);   // blocked straight away = wall we missed
  mx += DX[hd] * moved;
  my += DY[hd] * moved;
  settleToF(30);
  if (distF < FRONT_WALL_MM) alignFront();
  return moved;
}

// ======================= SEARCH / RUNS =======================
bool atGoal(bool toStart) {
  if (toStart) return mx == 0 && my == 0;
  if (mx == GOAL_X && my == GOAL_Y) return true;
  return COLOUR_GOAL && onMarkedTile();
}

// Flood-fill exploration. Unknown walls are treated as open.
bool explore(bool toStart) {
  while (!aborted) {
    senseWalls();
    if (atGoal(toStart)) return true;
    flood(toStart, false);
    int d = bestDir(false);
    if (d < 0) return false;            // boxed in: no route
    faceDir(d);
    forwardCells(1, EXPLORE_PWM);
  }
  return false;
}

// Fast run on known cells only, merging straights. Falls back to exploring if the map has gaps.
bool speedRun() {
  while (!aborted) {
    if (atGoal(false)) return true;
    flood(false, true);
    if (distv[mx][my] == 255) return explore(false);
    int d = bestDir(true);
    if (d < 0) return explore(false);
    faceDir(d);
    int n = 1, x = mx + DX[d], y = my + DY[d];
    while (!(x == GOAL_X && y == GOAL_Y) && canGo(x, y, d, true)) {
      int nx = x + DX[d], ny = y + DY[d];
      if (distv[nx][ny] != distv[x][y] - 1) break;
      x = nx; y = ny; n++;
    }
    int moved = forwardCells(n, SPEED_PWM);
    if (moved < n) senseWalls();        // something unexpected: re-check walls
  }
  return false;
}

// ======================= MODES =======================
const char* MODES[] = {"Explore", "Speed run", "Sensors", "Test moves", "Clear map"};
const int NUM_MODES = 5;
int mode = 0;

void countdown() {
  for (int i = 2; i > 0; i--) {
    char b[20]; snprintf(b, sizeof(b), "Hands off... %d", i);
    show(MODES[mode], b);
    delay(700);
  }
}

void doRun(bool fast) {
  countdown();
  aborted = false;
  mx = 0; my = 0; hd = 0;
  calibrateFloor();
  led(true);
  unsigned long t0 = millis();
  bool ok = fast ? speedRun() : explore(false);
  unsigned long t = millis() - t0;
  setMotors(0, 0);
  led(false);
  saveMaze();
  if (ok) flicker(1500);

  char l2[24], l3[24];
  if (ok) {
    snprintf(l2, sizeof(l2), "Time %lu.%02lus", t / 1000, (t % 1000) / 10);
    snprintf(l3, sizeof(l3), "Cells known %d", knownCells());
    show("GOAL!", l2, l3, aborted ? "" : "Returning...");
    Serial.printf("Goal in %lu ms\n", t);
    if (RETURN_TO_START && !aborted) {
      delay(1000);
      explore(true);
      faceDir(0);
      setMotors(0, 0);
      saveMaze();
      show("GOAL!", l2, l3, "Back at start");
    }
  } else {
    snprintf(l2, sizeof(l2), "At cell %d,%d", mx, my);
    show(aborted ? "Aborted" : "No route found", l2, "Map saved", "Press to continue");
  }
  waitRelease();
  while (readButton() == 0) delay(10);
  aborted = false;
}

void sensorMode() {
  waitRelease();
  calibrateFloor();
  while (true) {
    updateToF();
    readColour();
    char a[24], b[24], c[24], d[24];
    snprintf(a, sizeof(a), "F%4d L%4d R%4d", distF, distL, distR);
    snprintf(b, sizeof(b), "r%u g%u b%u", cr, cg, cb);
    snprintf(c, sizeof(c), "c%u %s", cc, colourName());
    snprintf(d, sizeof(d), "encL%ld R%ld", encL(), encR());
    show(a, b, c, d);
    led(onMarkedTile());   // LED lights when the colour sensor sees a marked tile
    if (digitalRead(BTN_PIN) == LOW) { waitRelease(); return; }
    delay(100);
  }
}

void testMode() {
  countdown();
  aborted = false;
  show("Test", "Both wheels fwd?");
  setMotors(MIN_PWM + 20, MIN_PWM + 20);
  unsigned long t0 = millis();
  while (millis() - t0 < 600) { if (checkAbort()) break; delay(5); }
  brake();
  char b[24]; snprintf(b, sizeof(b), "encL%ld R%ld", encL(), encR());
  show("Test", b, "both should be +");
  delay(1500);
  if (!aborted) { show("Test", "1 cell straight"); driveStraight(CELL_MM, EXPLORE_PWM); delay(500); }
  for (int i = 0; i < 4 && !aborted; i++) { show("Test", "4 right turns", "should end facing", "the same way"); turnQuarter(1); delay(300); }
  setMotors(0, 0);
  show("Test done", "", "Press to continue");
  waitRelease();
  while (readButton() == 0) delay(10);
  aborted = false;
}

void runSelected() {
  switch (mode) {
    case 0: doRun(false); break;
    case 1: doRun(true); break;
    case 2: sensorMode(); break;
    case 3: testMode(); break;
    case 4: mazeReset(); saveMaze(); show("Map cleared"); delay(800); break;
  }
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);

  pinMode(XSHUT_F, OUTPUT); digitalWrite(XSHUT_F, LOW);
  pinMode(XSHUT_L, OUTPUT); digitalWrite(XSHUT_L, LOW);
  pinMode(XSHUT_R, OUTPUT); digitalWrite(XSHUT_R, LOW);
  pinMode(BTN_PIN, INPUT_PULLUP);

  pinMode(L_DIR, OUTPUT); pinMode(R_DIR, OUTPUT);
  pwmInit(L_PWM); pwmInit(R_PWM);
  setMotors(0, 0);

#if USE_ENCODERS
  pinMode(L_ENC_A, INPUT); pinMode(L_ENC_B, INPUT);
  pinMode(R_ENC_A, INPUT); pinMode(R_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), isrR, CHANGE);
#endif

  pinMode(LED_PIN, OUTPUT);
  led(false);

  Wire.begin(I2C1_SDA, I2C1_SCL);
  Wire.setClock(400000);
  show("Micromouse", "Starting...");
  delay(20);

  initToF();

  Wire1.begin(I2C2_SDA, I2C2_SCL);
  colourOK = tcs.begin(TCS34725_ADDRESS, &Wire1);

  loadMaze();
  Serial.printf("ToF F%d L%d R%d  colour %d  cells known %d\n",
                tofOK[0], tofOK[1], tofOK[2], colourOK, knownCells());
}

// Prints the menu to Serial once (not every loop)
void printMenu() {
  char l2[24], l3[24], l4[24];
  snprintf(l2, sizeof(l2), "short=next long=go");
  snprintf(l3, sizeof(l3), "Map: %d/81 cells", knownCells());
  snprintf(l4, sizeof(l4), "ToF %c%c%c  Col %s",
           tofOK[0] ? 'F' : '-', tofOK[1] ? 'L' : '-', tofOK[2] ? 'R' : '-',
           colourOK ? "ok" : "--");
  char l1[24]; snprintf(l1, sizeof(l1), "> %d %s", mode + 1, MODES[mode]);
  show(l1, l2, l3, l4);
}

// Non-blocking LED: blinks (mode+1) times, pause, repeat. Button stays responsive.
void menuBlink() {
  static unsigned long t0 = 0;
  unsigned long t = (millis() - t0);
  int blinks = mode + 1;
  unsigned long cycle = blinks * 360UL + 1200UL;
  if (t >= cycle) { t0 = millis(); t = 0; }
  bool on = (t < blinks * 360UL) && ((t % 360UL) < 180UL);
  led(on);
}

bool menuDirty = true;

void loop() {
  if (menuDirty) { printMenu(); menuDirty = false; }
  menuBlink();

  int b = readButton();
  if (b == 1) { mode = (mode + 1) % NUM_MODES; menuDirty = true; led(false); delay(400); }
  else if (b == 2) { led(false); runSelected(); led(false); menuDirty = true; }
  delay(10);
}
