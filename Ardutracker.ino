#include <Arduboy2.h>

Arduboy2 arduboy;

// ── Shared ────────────────────────────────────────────────────────────────────
static const uint8_t CELL_H = 9;  // 1px border + 8px font, shared by both screens

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
static const uint8_t COLS    = 8;
static const uint8_t ROWS    = 40;
static const uint8_t VISIBLE = 6;  // data rows (1 row reserved for header)
static const uint8_t CELL_W  = 16;

uint8_t grid[ROWS][COLS];  // 0xFF = empty, 0x00–0xFE = pattern index

uint8_t curCol    = 0;
uint8_t curRow    = 0;
uint8_t scrollTop = 0;

// ── Pattern screen ────────────────────────────────────────────────────────────
static const uint8_t MAX_PATS    = 32;   // patterns 0x00–0x1F supported
static const uint8_t PAT_STEPS   = 16;
static const uint8_t PAT_VISIBLE = 7;   // steps visible at once (no header row)
static const uint8_t PAT_COL_W   = 64;  // 128 / 2 columns
static const uint8_t NOTE_MIN    = 12;  // C0
static const uint8_t NOTE_MAX    = 107; // B7

uint8_t patNote[MAX_PATS][PAT_STEPS];  // 0xFF = no note, else MIDI note number

uint8_t openPat     = 0;
uint8_t patCurCol   = 0;
uint8_t patCurRow   = 0;
uint8_t patScroll   = 0;
uint8_t patLastNote = 60;  // C4; resets to this on each pattern screen entry

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

// 3-char note: "C 4", "C#4", "A#3" …  0xFF → "---"
void printNote(uint8_t note) {
  static const char names[12][2] = {
    {'C',' '},{'C','#'},{'D',' '},{'D','#'},{'E',' '},{'F',' '},
    {'F','#'},{'G',' '},{'G','#'},{'A',' '},{'A','#'},{'B',' '}
  };
  arduboy.print(names[note % 12][0]);
  arduboy.print(names[note % 12][1]);
  arduboy.print((char)('0' + note / 12 - 1));  // C4=60: 60/12-1=4
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
        if (n == 0xFF) arduboy.print(F("---"));
        else printNote(n);
      } else {
        arduboy.print(F("--"));  // col 2: purpose TBD
      }
    }
  }
  arduboy.setTextColor(WHITE);
}

// ── Tracker cell edit ─────────────────────────────────────────────────────────
void editCell(int16_t delta) {
  uint8_t *v = &grid[curRow][curCol];
  int16_t next = (int16_t)((*v == 0xFF) ? 0x00 : *v) + delta;
  if (next < 0x00) next = 0x00;
  if (next > 0xFE) next = 0xFE;
  *v = (uint8_t)next;
}

// ── Pattern note edit ─────────────────────────────────────────────────────────
// Blank cells initialise to patLastNote before delta is applied.
// patLastNote updates after every edit and resets to C4 on screen exit.
void editPatNote(int16_t delta) {
  uint8_t *n = &patNote[openPat][patCurRow];
  uint8_t cur = (*n == 0xFF) ? patLastNote : *n;
  int16_t next = (int16_t)cur + delta;
  if (next < NOTE_MIN) next = NOTE_MIN;
  if (next > NOTE_MAX) next = NOTE_MAX;
  *n = (uint8_t)next;
  patLastNote = *n;
}

// ── Tracker input ─────────────────────────────────────────────────────────────
void handleTrackerInput() {
  bool aHeld = arduboy.pressed(A_BUTTON);
  bool bHeld = arduboy.pressed(B_BUTTON);

  if (aHeld != aWasPressed) { resetInputState(); aWasPressed = aHeld; }

  if (aHeld) {
    if (checkRepeat(UP_BUTTON,    0)) editCell(+0x10);
    if (checkRepeat(DOWN_BUTTON,  1)) editCell(-0x10);
    if (checkRepeat(LEFT_BUTTON,  2)) editCell(-0x01);
    if (checkRepeat(RIGHT_BUTTON, 3)) editCell(+0x01);
  } else if (bHeld) {
    if (arduboy.justPressed(RIGHT_BUTTON)) {
      uint8_t val = grid[curRow][curCol];
      if (val != 0xFF && val < MAX_PATS) {
        openPat     = val;
        patCurRow   = 0;
        patCurCol   = 0;
        patScroll   = 0;
        patLastNote = 60;
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
  if (arduboy.justPressed(B_BUTTON)) {
    patLastNote = 60;
    resetInputState();
    screen = SCR_TRACKER;
    return;
  }

  bool aHeld = arduboy.pressed(A_BUTTON);
  if (aHeld != aWasPressed) { resetInputState(); aWasPressed = aHeld; }

  if (aHeld && patCurCol == 0) {
    // Note column: A + dpad edits pitch
    if (checkRepeat(UP_BUTTON,    0)) editPatNote(+12);  // +1 octave
    if (checkRepeat(DOWN_BUTTON,  1)) editPatNote(-12);  // -1 octave
    if (checkRepeat(LEFT_BUTTON,  2)) editPatNote(-1);   // -1 semitone
    if (checkRepeat(RIGHT_BUTTON, 3)) editPatNote(+1);   // +1 semitone
  } else if (!aHeld) {
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
