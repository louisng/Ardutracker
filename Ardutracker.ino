#include <Arduboy2.h>
#include <EEPROM.h>
#include <avr/interrupt.h>

Arduboy2 arduboy;

// ── Constants ─────────────────────────────────────────────────────────────────
static const uint8_t CELL_H     = 9;
static const uint8_t DBL_WINDOW = 10;

// ── Screen ────────────────────────────────────────────────────────────────────
enum Screen : uint8_t { SCR_TRACKER, SCR_PATTERN, SCR_SETTINGS, SCR_SOUND };
Screen screen = SCR_TRACKER;

// ── Shared input ──────────────────────────────────────────────────────────────
uint8_t rptTimer[4];
static const uint8_t RPT_START = 20;
static const uint8_t RPT_RATE  = 5;
bool aWasPressed  = false;
bool aPressClean  = false;  // A was pressed without direction; fire tap on release

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
  aPressClean = false;
}

// ── Tracker ───────────────────────────────────────────────────────────────────
static const uint8_t COLS     = 8;
static const uint8_t ROWS     = 40;
static const uint8_t VISIBLE  = 6;
static const uint8_t CELL_W   = 16;
static const uint8_t GRID_MAX = 0x1F;

uint8_t grid[ROWS][COLS];
uint8_t gridLastVal = 0x00;
uint8_t curCol = 0, curRow = 0, scrollTop = 0;

uint8_t gridTapTimer  = 0;
bool    gridTapActive = false;
uint8_t gridTapRow    = 0, gridTapCol = 0;

// ── Pattern ───────────────────────────────────────────────────────────────────
static const uint8_t MAX_PATS    = 32;
static const uint8_t PAT_STEPS   = 16;
static const uint8_t PAT_VISIBLE = 7;
static const uint8_t PAT_COL_W   = 64;
static const uint8_t NOTE_EMPTY  = 0xFF;
static const uint8_t NOTE_MUTE   = 0xFE;
static const uint8_t NOTE_MIN    = 12;
static const uint8_t NOTE_MAX    = 107;

uint8_t patNote[MAX_PATS][PAT_STEPS];
uint8_t openPat = 0, patCurCol = 0, patCurRow = 0, patScroll = 0;
uint8_t patLastNote = 60;

uint8_t aTapTimer  = 0;
bool    aTapActive = false;
uint8_t aTapPrev   = NOTE_EMPTY, aTapRow = 0, aTapCol = 0;

// ── Settings ──────────────────────────────────────────────────────────────────
static const uint16_t BPM_MIN = 20;
static const uint16_t BPM_MAX = 300;
uint16_t bpm = 80;

uint32_t tapTimes[4];
uint8_t  tapHead = 0, tapFill = 0;

// ── Sound / columns ───────────────────────────────────────────────────────────
// 8 presets: 2-char name, octave offset relative to written note
static const char  PRESET_NAME[8][3] PROGMEM = {
  "LD","BS","AP","CH","HI","LO","PC","PD"
};
static const int8_t PRESET_OCT[8] PROGMEM = { 0, -2, 0, 0, +1, -1, 0, 0 };

uint8_t colPreset[COLS] = {0, 1, 2, 3, 4, 5, 6, 7};
uint8_t colVolume[COLS] = {8, 8, 8, 8, 8, 8, 8, 8};

uint8_t sndCurCol = 0, sndCurRow = 0;  // 0=preset row, 1=volume row

// ── Playback ──────────────────────────────────────────────────────────────────
bool     playing      = false;
uint8_t  playRow      = 0, playStep = 0;
uint8_t  playStartRow = 0;
uint32_t lastStepMs   = 0;

// C4-B4 frequencies (MIDI 60-71); octave-shift via bit shift
static const uint16_t NOTE_FREQS[12] PROGMEM = {
  262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494
};

// ── Audio engine ──────────────────────────────────────────────────────────────
static const uint8_t  MAX_VOICES  = 4;
static const uint32_t SAMPLE_RATE = 40000UL;

volatile uint16_t voiceDelta[MAX_VOICES];
volatile uint8_t  voiceActive[MAX_VOICES];
uint8_t colVoice[COLS];  // 0xFF = no voice assigned

ISR(TIMER3_COMPA_vect) {
  static uint16_t ph[MAX_VOICES];
  ph[0] += voiceDelta[0]; ph[1] += voiceDelta[1];
  ph[2] += voiceDelta[2]; ph[3] += voiceDelta[3];
  uint8_t out = ((uint8_t)(ph[0] >> 15) & voiceActive[0])
              ^ ((uint8_t)(ph[1] >> 15) & voiceActive[1])
              ^ ((uint8_t)(ph[2] >> 15) & voiceActive[2])
              ^ ((uint8_t)(ph[3] >> 15) & voiceActive[3]);
  if (out) { PORTC = (PORTC | (1 << 6)) & ~(1 << 7); }
  else     { PORTC = (PORTC & ~(1 << 6)) | (1 << 7); }
}

void audioBegin() {
  DDRC  |=  (1 << 6) | (1 << 7);
  PORTC &= ~((1 << 6) | (1 << 7));
  TCCR3A = 0;
  TCCR3B = (1 << WGM32) | (1 << CS30);  // CTC, prescaler=1
  OCR3A  = (uint16_t)(F_CPU / SAMPLE_RATE) - 1;  // 399 at 40kHz
  // TIMSK3 stays 0; voiceOn() enables it when a note starts
}

void voiceOn(uint8_t v, uint16_t freq) {
  uint16_t delta = (uint16_t)((uint32_t)freq * 65536UL / SAMPLE_RATE);
  cli();
  voiceDelta[v]  = delta;
  voiceActive[v] = 1;
  TIMSK3         = (1 << OCIE3A);
  sei();
}

void voiceOff(uint8_t v) {
  cli();
  voiceActive[v] = 0;
  if (!voiceActive[0] && !voiceActive[1] && !voiceActive[2] && !voiceActive[3]) {
    TIMSK3 = 0;
    PORTC &= ~((1 << 6) | (1 << 7));
  }
  sei();
}

void allVoicesOff() {
  cli();
  voiceActive[0] = voiceActive[1] = voiceActive[2] = voiceActive[3] = 0;
  TIMSK3 = 0;
  PORTC &= ~((1 << 6) | (1 << 7));
  sei();
}

uint8_t findFreeVoice() {
  bool busy[MAX_VOICES] = {};
  for (uint8_t c = 0; c < COLS; c++) {
    if (colVoice[c] < MAX_VOICES) busy[colVoice[c]] = true;
  }
  for (uint8_t v = 0; v < MAX_VOICES; v++) {
    if (!busy[v]) return v;
  }
  return 0xFF;
}

// ── EEPROM layout (base offset 16 to clear Arduboy2 reserved bytes) ───────────
// [0-1] magic  [2-321] grid  [322-833] patNote
// [834-835] bpm  [836-843] colPreset  [844-851] colVolume
static const uint16_t EEPROM_BASE   = 16;
static const uint8_t  EEPROM_MAGIC0 = 0xA7;
static const uint8_t  EEPROM_MAGIC1 = 0x5C;

// ── EEPROM save / load ────────────────────────────────────────────────────────
void saveSong() {
  arduboy.clear();
  arduboy.setTextColor(WHITE);
  arduboy.setCursor(28, 28);
  arduboy.print(F("SAVING..."));
  arduboy.display();

  uint16_t addr = EEPROM_BASE;
  EEPROM.update(addr++, EEPROM_MAGIC0);
  EEPROM.update(addr++, EEPROM_MAGIC1);
  uint8_t *p = (uint8_t *)grid;
  for (uint16_t i = 0; i < sizeof(grid);    i++) EEPROM.update(addr++, p[i]);
  p = (uint8_t *)patNote;
  for (uint16_t i = 0; i < sizeof(patNote); i++) EEPROM.update(addr++, p[i]);
  EEPROM.update(addr++, (uint8_t)(bpm & 0xFF));
  EEPROM.update(addr++, (uint8_t)(bpm >> 8));
  for (uint8_t i = 0; i < COLS; i++) EEPROM.update(addr++, colPreset[i]);
  for (uint8_t i = 0; i < COLS; i++) EEPROM.update(addr++, colVolume[i]);
}

void loadSong() {
  if (EEPROM.read(EEPROM_BASE)     != EEPROM_MAGIC0 ||
      EEPROM.read(EEPROM_BASE + 1) != EEPROM_MAGIC1) return;
  uint16_t addr = EEPROM_BASE + 2;
  uint8_t *p = (uint8_t *)grid;
  for (uint16_t i = 0; i < sizeof(grid);    i++) p[i] = EEPROM.read(addr++);
  p = (uint8_t *)patNote;
  for (uint16_t i = 0; i < sizeof(patNote); i++) p[i] = EEPROM.read(addr++);
  bpm = (uint16_t)EEPROM.read(addr) | ((uint16_t)EEPROM.read(addr + 1) << 8);
  addr += 2;
  for (uint8_t i = 0; i < COLS; i++) colPreset[i] = EEPROM.read(addr++);
  for (uint8_t i = 0; i < COLS; i++) colVolume[i]  = EEPROM.read(addr++);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  arduboy.boot();  // skip boot logo/sound to avoid Timer3 ISR conflict
  arduboy.clear();
  arduboy.display();
  arduboy.setFrameRate(30);
  memset(grid, 0xFF, sizeof(grid));
  memset(patNote, 0xFF, sizeof(patNote));
  memset(colVoice, 0xFF, sizeof(colVoice));
  loadSong();
  audioBegin();
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
  arduboy.print((char)('0' + note / 12 - 1));
}

uint16_t midiToFreq(uint8_t note) {
  uint8_t  sem    = note % 12;
  int8_t   octave = (int8_t)(note / 12) - 5;  // relative to C4 block (60/12=5)
  uint16_t freq   = pgm_read_word(&NOTE_FREQS[sem]);
  if (octave > 0) { for (int8_t i = 0; i < octave;  i++) freq <<= 1; }
  else            { for (int8_t i = octave; i < 0;   i++) freq >>= 1; }
  return freq;
}

// ── Draw: Tracker ─────────────────────────────────────────────────────────────
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
    bool playHere = playing && (dr == playRow);
    for (uint8_t c = 0; c < COLS; c++) {
      uint8_t cx = c * CELL_W;
      uint8_t cy = (vr + 1) * CELL_H;
      bool cur = (c == curCol && dr == curRow);
      if (cur) {
        arduboy.fillRect(cx + 1, cy + 1, CELL_W - 2, CELL_H - 2, WHITE);
        arduboy.setTextColor(BLACK);
      } else if (playHere) {
        // Subtle play-row marker: invert just the top 1px of the cell interior
        arduboy.drawFastHLine(cx + 1, cy + 1, CELL_W - 2, WHITE);
        arduboy.setTextColor(WHITE);
      } else {
        arduboy.setTextColor(WHITE);
      }
      arduboy.setCursor(cx + 2, cy + 2);  // +2 to clear the play marker line
      if (grid[dr][c] == 0xFF) arduboy.print(F("--"));
      else                     printHex2(grid[dr][c]);
    }
  }
  arduboy.setTextColor(WHITE);
}

// ── Draw: Pattern ─────────────────────────────────────────────────────────────
void drawPattern() {
  arduboy.drawFastVLine(0,   0, 64, WHITE);
  arduboy.drawFastVLine(64,  0, 64, WHITE);
  arduboy.drawFastVLine(127, 0, 64, WHITE);
  for (uint8_t r = 0; r <= PAT_VISIBLE; r++)
    arduboy.drawFastHLine(0, r * CELL_H, 128, WHITE);

  for (uint8_t vr = 0; vr < PAT_VISIBLE; vr++) {
    uint8_t step = vr + patScroll;
    if (step >= PAT_STEPS) break;
    bool playHere = playing && (step == playStep);
    for (uint8_t c = 0; c < 2; c++) {
      uint8_t cx = c * PAT_COL_W;
      uint8_t cy = vr * CELL_H;
      bool cur = (c == patCurCol && step == patCurRow);
      if (cur) {
        arduboy.fillRect(cx + 1, cy + 1, PAT_COL_W - 2, CELL_H - 2, WHITE);
        arduboy.setTextColor(BLACK);
      } else if (playHere) {
        arduboy.drawFastHLine(cx + 1, cy + 1, PAT_COL_W - 2, WHITE);
        arduboy.setTextColor(WHITE);
      } else {
        arduboy.setTextColor(WHITE);
      }
      arduboy.setCursor(cx + 2, cy + 2);
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

// ── Draw: Settings ────────────────────────────────────────────────────────────
void drawSettings() {
  arduboy.drawRect(0, 0, 128, 64, WHITE);
  arduboy.drawFastHLine(0, 10, 128, WHITE);

  arduboy.setTextColor(WHITE);
  arduboy.setCursor(34, 2);
  arduboy.print(F("SETTINGS"));

  arduboy.setCursor(4, 20);
  arduboy.print(F("TEMPO"));

  arduboy.fillRect(60, 18, 38, 11, WHITE);
  arduboy.setTextColor(BLACK);
  arduboy.setCursor(63, 20);
  if (bpm < 100) arduboy.print(' ');
  arduboy.print(bpm);
  arduboy.setTextColor(WHITE);
  arduboy.setCursor(101, 20);
  arduboy.print(F("BPM"));

  for (uint8_t i = 0; i < 4; i++) {
    uint8_t x = 60 + i * 8;
    if (i < tapFill) arduboy.fillCircle(x, 38, 2, WHITE);
    else             arduboy.drawCircle(x, 38, 2, WHITE);
  }

  arduboy.setCursor(4, 50);
  arduboy.print(F("B:back  A:tap"));
}

// ── Draw: Sound ───────────────────────────────────────────────────────────────
void drawSound() {
  for (uint8_t c = 0; c < COLS; c++)
    arduboy.drawFastVLine(c * CELL_W, 0, 64, WHITE);
  arduboy.drawFastVLine(127, 0, 64, WHITE);
  arduboy.drawFastHLine(0, 0,            128, WHITE);
  arduboy.drawFastHLine(0, CELL_H,       128, WHITE);
  arduboy.drawFastHLine(0, CELL_H * 2,   128, WHITE);
  arduboy.drawFastHLine(0, CELL_H * 3,   128, WHITE);

  arduboy.setTextColor(WHITE);
  for (uint8_t c = 0; c < COLS; c++) {
    arduboy.setCursor(c * CELL_W + 5, 1);
    arduboy.print((char)('1' + c));
  }

  // Preset row
  for (uint8_t c = 0; c < COLS; c++) {
    uint8_t cx = c * CELL_W;
    uint8_t cy = CELL_H;
    bool cur = (c == sndCurCol && sndCurRow == 0);
    if (cur) {
      arduboy.fillRect(cx + 1, cy + 1, CELL_W - 2, CELL_H - 2, WHITE);
      arduboy.setTextColor(BLACK);
    } else {
      arduboy.setTextColor(WHITE);
    }
    arduboy.setCursor(cx + 2, cy + 1);
    arduboy.print((char)pgm_read_byte(&PRESET_NAME[colPreset[c]][0]));
    arduboy.print((char)pgm_read_byte(&PRESET_NAME[colPreset[c]][1]));
  }

  // Volume row
  for (uint8_t c = 0; c < COLS; c++) {
    uint8_t cx = c * CELL_W;
    uint8_t cy = CELL_H * 2;
    bool cur = (c == sndCurCol && sndCurRow == 1);
    if (cur) {
      arduboy.fillRect(cx + 1, cy + 1, CELL_W - 2, CELL_H - 2, WHITE);
      arduboy.setTextColor(BLACK);
    } else {
      arduboy.setTextColor(WHITE);
    }
    arduboy.setCursor(cx + 2, cy + 1);
    uint8_t v = colVolume[c];
    if (v < 10) arduboy.print(' ');
    arduboy.print(v);
  }
  arduboy.setTextColor(WHITE);

  arduboy.setCursor(2, CELL_H * 3 + 4);
  arduboy.print(F("A:preset  A+<>:vol"));
  arduboy.setCursor(2, CELL_H * 3 + 13);
  arduboy.print(F("B:back"));
}

// ── Tracker cell edit ─────────────────────────────────────────────────────────
void editCell(int16_t delta) {
  uint8_t *v = &grid[curRow][curCol];
  int16_t next = (int16_t)((*v == 0xFF) ? 0x00 : *v) + delta;
  if (next < 0x00)     next = 0x00;
  if (next > GRID_MAX) next = GRID_MAX;
  *v = (uint8_t)next;
  gridLastVal = *v;
}

void handleGridTap() {
  uint8_t *v = &grid[curRow][curCol];
  bool doubleTap = gridTapActive && gridTapTimer > 0 &&
                   curRow == gridTapRow && curCol == gridTapCol;
  if (doubleTap) {
    gridTapActive = false;
    gridTapTimer  = 0;
    *v = 0xFF;
  } else {
    gridTapRow    = curRow;
    gridTapCol    = curCol;
    gridTapActive = true;
    gridTapTimer  = DBL_WINDOW;
    if (*v == 0xFF) *v = gridLastVal;
  }
}

// ── Pattern note edit ─────────────────────────────────────────────────────────
void editPatNote(int16_t delta) {
  uint8_t *n = &patNote[openPat][patCurRow];
  uint8_t cur = (*n == NOTE_EMPTY || *n == NOTE_MUTE) ? patLastNote : *n;
  int16_t next = (int16_t)cur + delta;
  if (next < NOTE_MIN) next = NOTE_MIN;
  if (next > NOTE_MAX) next = NOTE_MAX;
  *n = (uint8_t)next;
  patLastNote = *n;
}

void handleATap() {
  uint8_t *n = &patNote[openPat][patCurRow];
  bool doubleTap = aTapActive && aTapTimer > 0 &&
                   patCurRow == aTapRow && patCurCol == aTapCol;
  if (doubleTap) {
    aTapActive = false;
    aTapTimer  = 0;
    *n = (aTapPrev == NOTE_EMPTY) ? NOTE_MUTE : NOTE_EMPTY;
  } else {
    aTapPrev   = *n;
    aTapRow    = patCurRow;
    aTapCol    = patCurCol;
    aTapActive = true;
    aTapTimer  = DBL_WINDOW;
    if (*n == NOTE_EMPTY) *n = patLastNote;
  }
}

// ── BPM helpers ───────────────────────────────────────────────────────────────
void adjustBPM(int16_t delta) {
  int16_t next = (int16_t)bpm + delta;
  if (next < (int16_t)BPM_MIN) next = BPM_MIN;
  if (next > (int16_t)BPM_MAX) next = BPM_MAX;
  bpm = (uint16_t)next;
}

void doTapTempo() {
  uint32_t now = millis();
  tapTimes[tapHead] = now;
  tapHead = (tapHead + 1) & 3;
  if (tapFill < 4) tapFill++;
  if (tapFill >= 2) {
    uint8_t  oldest   = (tapFill < 4) ? 0 : tapHead;
    uint32_t span     = now - tapTimes[oldest];
    uint32_t avg      = span / (tapFill - 1);
    if (avg > 0) {
      uint32_t computed = 60000UL / avg;
      if (computed < BPM_MIN) computed = BPM_MIN;
      if (computed > BPM_MAX) computed = BPM_MAX;
      bpm = (uint16_t)computed;
    }
  }
}

// ── Playback engine ───────────────────────────────────────────────────────────
void stepPlay() {
  uint32_t now    = millis();
  uint32_t stepMs = 60000UL / bpm / 4;  // 16th note
  if (now - lastStepMs < stepMs) return;
  lastStepMs += stepMs;

  for (uint8_t c = 0; c < COLS; c++) {
    uint8_t patIdx = grid[playRow][c];
    if (patIdx == 0xFF || patIdx >= MAX_PATS) continue;
    uint8_t n = patNote[patIdx][playStep];
    if (n == NOTE_MUTE) {
      if (colVoice[c] < MAX_VOICES) { voiceOff(colVoice[c]); colVoice[c] = 0xFF; }
    } else if (n != NOTE_EMPTY) {
      int8_t  oct = (int8_t)pgm_read_byte(&PRESET_OCT[colPreset[c]]);
      int16_t adj = (int16_t)n + (int16_t)oct * 12;
      if (adj < NOTE_MIN) adj = NOTE_MIN;
      if (adj > NOTE_MAX) adj = NOTE_MAX;
      uint16_t freq = midiToFreq((uint8_t)adj);
      if (freq > 0) {
        if (colVoice[c] >= MAX_VOICES) colVoice[c] = findFreeVoice();
        if (colVoice[c] < MAX_VOICES)  voiceOn(colVoice[c], freq);
      }
    }
    // NOTE_EMPTY: sustain — leave voice running
  }

  if (++playStep >= PAT_STEPS) {
    playStep = 0;
    uint8_t nextRow = playRow + 1;
    bool nextOk = false;
    if (nextRow < ROWS) {
      for (uint8_t c = 0; c < COLS; c++) {
        if (grid[nextRow][c] != 0xFF) { nextOk = true; break; }
      }
    }
    playRow = nextOk ? nextRow : playStartRow;
  }
}

// ── A+B play/stop (global, checked before screen dispatch) ────────────────────
bool handlePlayStop() {
  bool abFire = (arduboy.justPressed(A_BUTTON) && arduboy.pressed(B_BUTTON)) ||
                (arduboy.justPressed(B_BUTTON) && arduboy.pressed(A_BUTTON));
  if (!abFire) return false;
  playing = !playing;
  if (playing) {
    playStartRow = curRow;
    playRow      = curRow;
    playStep     = 0;
    lastStepMs   = millis();
    memset(colVoice, 0xFF, sizeof(colVoice));
  } else {
    allVoicesOff();
    memset(colVoice, 0xFF, sizeof(colVoice));
  }
  resetInputState();
  return true;
}

// ── Input: Tracker ────────────────────────────────────────────────────────────
void handleTrackerInput() {
  if (gridTapTimer > 0) gridTapTimer--;

  // Capture release state before any resetInputState() call clears aPressClean
  bool justRelA = arduboy.justReleased(A_BUTTON);
  bool wasClean = aPressClean;

  bool aHeld = arduboy.pressed(A_BUTTON);
  bool bHeld = arduboy.pressed(B_BUTTON);
  bool noDir = !arduboy.pressed(UP_BUTTON)   && !arduboy.pressed(DOWN_BUTTON) &&
               !arduboy.pressed(LEFT_BUTTON) && !arduboy.pressed(RIGHT_BUTTON);

  if (aHeld != aWasPressed) { resetInputState(); aWasPressed = aHeld; }

  if (aHeld) {
    if (arduboy.justPressed(A_BUTTON)) {
      // Store non-empty cell to memory immediately on press
      uint8_t v = grid[curRow][curCol];
      if (v != 0xFF) gridLastVal = v;
      aPressClean = noDir;
    } else if (!noDir) {
      aPressClean   = false;
      gridTapActive = false;
      gridTapTimer  = 0;
      if (checkRepeat(UP_BUTTON,    0)) editCell(+0x10);
      if (checkRepeat(DOWN_BUTTON,  1)) editCell(-0x10);
      if (checkRepeat(LEFT_BUTTON,  2)) editCell(-0x01);
      if (checkRepeat(RIGHT_BUTTON, 3)) editCell(+0x01);
    }
  } else if (bHeld) {
    if (arduboy.justPressed(UP_BUTTON)) {
      sndCurCol = curCol;
      sndCurRow = 0;
      resetInputState();
      screen = SCR_SOUND;
    } else if (arduboy.justPressed(DOWN_BUTTON)) {
      saveSong();
      resetInputState();
    } else if (arduboy.justPressed(RIGHT_BUTTON)) {
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
    } else if (arduboy.justPressed(LEFT_BUTTON)) {
      tapHead = 0;
      tapFill = 0;
      resetInputState();
      screen = SCR_SETTINGS;
    }
  } else {
    if (justRelA && wasClean) handleGridTap();  // paste/clear fires on release
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

// ── Input: Pattern ────────────────────────────────────────────────────────────
void handlePatternInput() {
  if (aTapTimer > 0) aTapTimer--;

  // Capture release state before any resetInputState() call clears aPressClean
  bool justRelA = arduboy.justReleased(A_BUTTON);
  bool wasClean = aPressClean;

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
    if (arduboy.justPressed(A_BUTTON)) {
      // Store non-empty note to memory immediately on press
      if (patCurCol == 0) {
        uint8_t n = patNote[openPat][patCurRow];
        if (n != NOTE_EMPTY && n != NOTE_MUTE) patLastNote = n;
      }
      aPressClean = noDir;
    } else if (patCurCol == 0 && !noDir) {
      aPressClean = false;
      aTapActive  = false;
      aTapTimer   = 0;
      if (checkRepeat(UP_BUTTON,    0)) editPatNote(+12);
      if (checkRepeat(DOWN_BUTTON,  1)) editPatNote(-12);
      if (checkRepeat(LEFT_BUTTON,  2)) editPatNote(-1);
      if (checkRepeat(RIGHT_BUTTON, 3)) editPatNote(+1);
    }
  } else {
    if (justRelA && wasClean && patCurCol == 0) handleATap();  // paste/clear on release
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

// ── Input: Settings ───────────────────────────────────────────────────────────
void handleSettingsInput() {
  if (arduboy.justPressed(B_BUTTON)) {
    resetInputState();
    screen = SCR_TRACKER;
    return;
  }

  bool aHeld = arduboy.pressed(A_BUTTON);
  bool noDir = !arduboy.pressed(UP_BUTTON)   && !arduboy.pressed(DOWN_BUTTON) &&
               !arduboy.pressed(LEFT_BUTTON) && !arduboy.pressed(RIGHT_BUTTON);

  if (aHeld != aWasPressed) { resetInputState(); aWasPressed = aHeld; }

  if (aHeld) {
    if (noDir && arduboy.justPressed(A_BUTTON)) {
      doTapTempo();
    } else if (!noDir) {
      if (checkRepeat(UP_BUTTON,    0)) adjustBPM(+10);
      if (checkRepeat(DOWN_BUTTON,  1)) adjustBPM(-10);
      if (checkRepeat(LEFT_BUTTON,  2)) adjustBPM(-1);
      if (checkRepeat(RIGHT_BUTTON, 3)) adjustBPM(+1);
    }
  }
}

// ── Input: Sound ──────────────────────────────────────────────────────────────
void handleSoundInput() {
  if (arduboy.justPressed(B_BUTTON)) {
    resetInputState();
    screen = SCR_TRACKER;
    return;
  }

  bool aHeld = arduboy.pressed(A_BUTTON);
  bool noDir = !arduboy.pressed(UP_BUTTON)   && !arduboy.pressed(DOWN_BUTTON) &&
               !arduboy.pressed(LEFT_BUTTON) && !arduboy.pressed(RIGHT_BUTTON);

  if (aHeld != aWasPressed) { resetInputState(); aWasPressed = aHeld; }

  if (aHeld) {
    if (noDir && arduboy.justPressed(A_BUTTON)) {
      if (sndCurRow == 0)
        colPreset[sndCurCol] = (colPreset[sndCurCol] + 1) & 7;
    } else if (!noDir && sndCurRow == 1) {
      if (checkRepeat(LEFT_BUTTON,  2) && colVolume[sndCurCol] > 0)  colVolume[sndCurCol]--;
      if (checkRepeat(RIGHT_BUTTON, 3) && colVolume[sndCurCol] < 10) colVolume[sndCurCol]++;
    }
  } else {
    if (checkRepeat(UP_BUTTON,    0) && sndCurRow > 0)        sndCurRow--;
    if (checkRepeat(DOWN_BUTTON,  1) && sndCurRow < 1)        sndCurRow++;
    if (checkRepeat(LEFT_BUTTON,  2) && sndCurCol > 0)        sndCurCol--;
    if (checkRepeat(RIGHT_BUTTON, 3) && sndCurCol < COLS - 1) sndCurCol++;
  }
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
  if (!arduboy.nextFrame()) return;
  arduboy.pollButtons();

  if (handlePlayStop()) {
    // Just redraw without running input handlers this frame
    arduboy.clear();
    switch (screen) {
      case SCR_TRACKER:  drawTracker();  break;
      case SCR_PATTERN:  drawPattern();  break;
      case SCR_SETTINGS: drawSettings(); break;
      case SCR_SOUND:    drawSound();    break;
    }
    arduboy.display();
    return;
  }

  if (playing) stepPlay();

  switch (screen) {
    case SCR_TRACKER:
      handleTrackerInput();
      arduboy.clear();
      drawTracker();
      break;
    case SCR_PATTERN:
      handlePatternInput();
      arduboy.clear();
      drawPattern();
      break;
    case SCR_SETTINGS:
      handleSettingsInput();
      arduboy.clear();
      drawSettings();
      break;
    case SCR_SOUND:
      handleSoundInput();
      arduboy.clear();
      drawSound();
      break;
  }
  arduboy.display();
}
