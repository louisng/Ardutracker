#include <Arduboy2.h>

Arduboy2 arduboy;

// ── Shared ────────────────────────────────────────────────────────────────────
static const uint8_t CELL_H = 9;

enum Screen : uint8_t { SCR_TRACKER, SCR_PATTERN };
Screen screen = SCR_TRACKER;

uint8_t rptTimer[4];  // UP=0 DOWN=1 LEFT=2 RIGHT=3
static const uint8_t RPT_START = 20;
static const uint8_t RPT_RATE  = 5;
bool aWasPressed = false;

bool checkRepeat(uint8_t btn, uint8_t idx) {
  if (arduboy.justPressed(btn)) { rptTimer[idx] = RPT_START; return true; }
  if (arduboy.pressed(btn)) {
    if (rptTimer[idx] > 0) { rptTimer[idx]--; return false; }
    rptTimer[idx] = RPT_RATE;
    return true;
  }
  rptTimer[idx] = 0;
  return false;
}

void resetInputState() {
  memset(rptTimer, 0, sizeof(rptTimer));
  aWasPressed = false;
}

// ── Tracker screen ────────────────────────────────────────────────────────────
static const uint8_t COLS     = 8;
static const uint8_t ROWS     = 40;
static const uint8_t VISIBLE  = 6;
static const uint8_t CELL_W   = 16;
static const uint8_t GRID_MAX = 0x1F;  // MAX_PATS - 1; max pattern index

uint8_t grid[ROWS][COLS];     // 0xFF = empty, 0x00–0x1F = pattern index
uint8_t gridLastVal = 0x00;   // memory for A-alone paste on tracker

uint8_t curCol = 0, curRow = 0, scrollTop = 0;

// ── Pattern screen ────────────────────────────────────────────────────────────
static const uint8_t MAX_PATS    = 32;
static const uint8_t PAT_STEPS   = 16;
static const uint8_t PAT_VISIBLE = 7;
static const uint8_t PAT_COL_W   = 64;
static const uint8_t NOTE_EMPTY  = 0xFF;  // blank step
static const uint8_t NOTE_MUTE   = 0xFE;  // mute / stop command
static const uint8_t NOTE_MIN    = 12;    // C0
static const uint8_t NOTE_MAX    = 107;   // B7

uint8_t patNote[MAX_PATS][PAT_STEPS];

uint8_t openPat     = 0;
uint8_t patCurCol   = 0, patCurRow = 0, patScroll = 0;
uint8_t patLastNote = 60;  // C4; memory resets to this on each entry

// Tap / double-click detection (pattern screen, note column)
static const uint8_t DBL_WINDOW = 10;  // frames (~0.33 s at 30 fps)
uint8_t aTapTimer  = 0;
bool    aTapActive = false;
uint8_t aTapPrev   = NOTE_EMPTY;  // cell state before first tap
uint8_t aTapRow    = 0;
uint8_t aTapCol    = 0;

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  arduboy.begin();
  arduboy.setFrameRate(30);
  memset(grid, 0xFF, sizeof(grid));
  memset(patNote, 0xFF, sizeof(patNote));
}

// ── Helpers ───────────────────────────────────────────────────────────────────
void printHex2(uint8_t val) {
  static const char h[] = "0123456789ABCDEF";
  arduboy.print(h[(val >> 4) & 0xF]);
  arduboy.print(h[val & 0xF]);
}

void printNote(uint8_t note) {
  static const char names[12][2] = {
    {'C',' '},{'C','#'},{'D',' '},{'D','#'},{'E',' '},{'F',' '},
    {'F','#'},{'G',' '},{'G','#'},{'A',' '},{'A','#'},{'B',' '}
  };
  arduboy.print(names[note % 12][0]);
  arduboy.print(names[note % 12][1]);
  arduboy.print((char)('0' + note / 12 - 1));  // C4=60 → 60/12-1=4
}

// ── Tracker draw ──────────────────────────────────────────────────────────────
void drawTracker() {
  for (uint8_t c = 0; c < COLS; c++)
    arduboy.drawFastVLine(c * CELL_W, 0, 64, WHITE);
  arduboy.drawFastVLine(127, 0, 64, WHITE);
  for (uint8_t r = 0; r <= VISIBLE + 1; r++)
    arduboy.drawFastHLine(0, r * CELL_H, 128, WHITE);

  arduboy.setTextColor(WHITE);
  for (uint8_t c = 0; c < COLS; c++) {
    arduboy.setCursor(c * CELL_W + 5, 1);
    arduboy.print((char)('1' + c));
  }

  for (uint8_t vr = 0; vr < VISIBLE; vr++) {
    uint8_t dr = vr + scrollTop;
    if (dr >= ROWS) break;
    for (uint8_t c = 0; c < COLS; c++) {
      uint8_t cx = c * CELL_W;
      uint8_t cy = (vr + 1) * CELL_H;
      bool cur = (c == curCol && dr == curRow);
      if (cur) {
        arduboy.fillRect(cx + 1, cy + 1, CELL_W - 2, CELL_H - 2, WHITE);
        arduboy.setTextColor(BLACK);
      } else {
        arduboy.setTextColor(WHITE);
      }
      arduboy.setCursor(cx + 2, cy + 1);
      if (grid[dr][c] == 0xFF) arduboy.print(F("--"));
      else printHex2(grid[dr][c]);
    }
  }
  arduboy.setTextColor(WHITE);
}

// ── Pattern draw ──────────────────────────────────────────────────────────────
void drawPattern() {
  arduboy.drawFastVLine(0,   0, 64, WHITE);
  arduboy.drawFastVLine(64,  0, 64, WHITE);
  arduboy.drawFastVLine(127, 0, 64, WHITE);
  for (uint8_t r = 0; r <= PAT_VISIBLE; r++)
    arduboy.drawFastHLine(0, r * CELL_H, 128, WHITE);

  for (uint8_t vr = 0; vr < PAT_VISIBLE; vr++) {
    uint8_t step = vr + patScroll;
    if (step >= PAT_STEPS) break;
    for (uint8_t c = 0; c < 2; c++) {
      uint8_t cx = c * PAT_COL_W;
      uint8_t cy = vr * CELL_H;
      bool cur = (c == patCurCol && step == patCurRow);
      if (cur) {
        arduboy.fillRect(cx + 1, cy + 1, PAT_COL_W - 2, CELL_H - 2, WHITE);
        arduboy.setTextColor(BLACK);
      } else {
        arduboy.setTextColor(WHITE);
      }
      arduboy.setCursor(cx + 2, cy + 1);
      if (c == 0) {
        uint8_t n = patNote[openPat][step];
        if      (n == NOTE_EMPTY) arduboy.print(F("---"));
        else if (n == NOTE_MUTE)  arduboy.print(F("M  "));
        else                      printNote(n);
      } else {
        arduboy.print(F("--"));
      }
    }
  }
  arduboy.setTextColor(WHITE);
}

// ── Tracker cell edit ─────────────────────────────────────────────────────────
void editCell(int16_t delta) {
  uint8_t *v = &grid[curRow][curCol];
  int16_t next = (int16_t)((*v == 0xFF) ? 0x00 : *v) + delta;
  if (next < 0x00)    next = 0x00;
  if (next > GRID_MAX) next = GRID_MAX;
  *v = (uint8_t)next;
  gridLastVal = *v;  // keep memory in sync
}

// ── Pattern note edit ─────────────────────────────────────────────────────────
// Blank and mute cells both start from patLastNote before the delta is applied.
void editPatNote(int16_t delta) {
  uint8_t *n = &patNote[openPat][patCurRow];
  uint8_t cur = (*n == NOTE_EMPTY || *n == NOTE_MUTE) ? patLastNote : *n;
  int16_t next = (int16_t)cur + delta;
  if (next < NOTE_MIN) next = NOTE_MIN;
  if (next > NOTE_MAX) next = NOTE_MAX;
  *n = (uint8_t)next;
  patLastNote = *n;
}

// ── Pattern A-tap handler ─────────────────────────────────────────────────────
// Single tap on blank  → paste patLastNote.
// Double tap on blank  → set mute (overrides the paste from first tap).
// Double tap on note   → clear to blank.
// Double tap on mute   → clear to blank.
// "Double" requires both taps on the same cell within DBL_WINDOW frames.
void handleATap() {
  uint8_t *n = &patNote[openPat][patCurRow];
  bool doubleTap = aTapActive && aTapTimer > 0 &&
                   patCurRow == aTapRow && patCurCol == aTapCol;
  if (doubleTap) {
    aTapActive = false;
    aTapTimer  = 0;
    *n = (aTapPrev == NOTE_EMPTY) ? NOTE_MUTE : NOTE_EMPTY;
  } else {
    // First tap: record state, start window
    aTapPrev   = *n;
    aTapRow    = patCurRow;
    aTapCol    = patCurCol;
    aTapActive = true;
    aTapTimer  = DBL_WINDOW;
    if (*n == NOTE_EMPTY) *n = patLastNote;  // paste on blank
    // note or mute: no immediate single-tap action
  }
}

// ── Tracker input ─────────────────────────────────────────────────────────────
void handleTrackerInput() {
  bool aHeld = arduboy.pressed(A_BUTTON);
  bool bHeld = arduboy.pressed(B_BUTTON);
  bool noDir = !arduboy.pressed(UP_BUTTON)   && !arduboy.pressed(DOWN_BUTTON) &&
               !arduboy.pressed(LEFT_BUTTON) && !arduboy.pressed(RIGHT_BUTTON);

  if (aHeld != aWasPressed) { resetInputState(); aWasPressed = aHeld; }

  if (aHeld) {
    if (noDir && arduboy.justPressed(A_BUTTON)) {
      // A alone: paste last value into an empty cell
      uint8_t *v = &grid[curRow][curCol];
      if (*v == 0xFF) *v = gridLastVal;
    } else if (!noDir) {
      if (checkRepeat(UP_BUTTON,    0)) editCell(+0x10);
      if (checkRepeat(DOWN_BUTTON,  1)) editCell(-0x10);
      if (checkRepeat(LEFT_BUTTON,  2)) editCell(-0x01);
      if (checkRepeat(RIGHT_BUTTON, 3)) editCell(+0x01);
    }
  } else if (bHeld) {
    if (arduboy.justPressed(RIGHT_BUTTON)) {
      uint8_t val = grid[curRow][curCol];
      if (val != 0xFF && val < MAX_PATS) {
        openPat     = val;
        patCurRow   = 0;
        patCurCol   = 0;
        patScroll   = 0;
        patLastNote = 60;
        aTapActive  = false;
        aTapTimer   = 0;
        resetInputState();
        screen = SCR_PATTERN;
      }
    }
  } else {
    if (checkRepeat(UP_BUTTON, 0) && curRow > 0) {
      curRow--;
      if (curRow < scrollTop) scrollTop = curRow;
    }
    if (checkRepeat(DOWN_BUTTON, 1) && curRow < ROWS - 1) {
      curRow++;
      if (curRow >= scrollTop + VISIBLE) scrollTop = curRow - VISIBLE + 1;
    }
    if (checkRepeat(LEFT_BUTTON,  2) && curCol > 0)        curCol--;
    if (checkRepeat(RIGHT_BUTTON, 3) && curCol < COLS - 1) curCol++;
  }
}

// ── Pattern input ─────────────────────────────────────────────────────────────
void handlePatternInput() {
  if (aTapTimer > 0) aTapTimer--;

  if (arduboy.justPressed(B_BUTTON)) {
    aTapActive  = false;
    aTapTimer   = 0;
    patLastNote = 60;
    resetInputState();
    screen = SCR_TRACKER;
    return;
  }

  bool aHeld = arduboy.pressed(A_BUTTON);
  bool noDir = !arduboy.pressed(UP_BUTTON)   && !arduboy.pressed(DOWN_BUTTON) &&
               !arduboy.pressed(LEFT_BUTTON) && !arduboy.pressed(RIGHT_BUTTON);

  if (aHeld != aWasPressed) { resetInputState(); aWasPressed = aHeld; }

  if (aHeld) {
    if (patCurCol == 0) {
      if (noDir && arduboy.justPressed(A_BUTTON)) {
        handleATap();
      } else if (!noDir) {
        // Direction pressed: cancel pending tap, edit pitch
        aTapActive = false;
        aTapTimer  = 0;
        if (checkRepeat(UP_BUTTON,    0)) editPatNote(+12);
        if (checkRepeat(DOWN_BUTTON,  1)) editPatNote(-12);
        if (checkRepeat(LEFT_BUTTON,  2)) editPatNote(-1);
        if (checkRepeat(RIGHT_BUTTON, 3)) editPatNote(+1);
      }
    }
  } else {
    if (checkRepeat(UP_BUTTON, 0) && patCurRow > 0) {
      patCurRow--;
      if (patCurRow < patScroll) patScroll = patCurRow;
    }
    if (checkRepeat(DOWN_BUTTON, 1) && patCurRow < PAT_STEPS - 1) {
      patCurRow++;
      if (patCurRow >= patScroll + PAT_VISIBLE) patScroll = patCurRow - PAT_VISIBLE + 1;
    }
    if (checkRepeat(LEFT_BUTTON,  2) && patCurCol > 0) patCurCol--;
    if (checkRepeat(RIGHT_BUTTON, 3) && patCurCol < 1) patCurCol++;
  }
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
  if (!arduboy.nextFrame()) return;
  arduboy.pollButtons();

  if (screen == SCR_TRACKER) {
    handleTrackerInput();
    arduboy.clear();
    drawTracker();
  } else {
    handlePatternInput();
    arduboy.clear();
    drawPattern();
  }
  arduboy.display();
}
