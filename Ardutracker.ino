#include <Arduboy2.h>
#include <EEPROM.h>
#include <MIDIUSB.h>
#include <avr/interrupt.h>

Arduboy2 arduboy;

// ── Constants ─────────────────────────────────────────────────────────────────
static const uint8_t CELL_H     = 9;
static const uint8_t DBL_WINDOW = 10;

// ── Screen ────────────────────────────────────────────────────────────────────
enum Screen : uint8_t { SCR_TRACKER, SCR_PATTERN, SCR_SETTINGS, SCR_SOUND, SCR_PRESET };
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
// 8 presets: 2-char name; editable oct offset + pulse width stored in presetOct/PW
static const char  PRESET_NAME[8][3] PROGMEM = {
  "LD","BS","AP","CH","HI","LO","PC","PD"
};
// Factory defaults (restored by A-tap in preset editor)
static const int8_t  DEF_PRESET_OCT[8] PROGMEM = {  0, -2,  0,  0, +1, -1,  0,  0 };
static const uint8_t DEF_PRESET_PW[8]  PROGMEM = {  4,  4,  3,  5,  2,  4,  2,  6 };
// PW 1-8: 1=12.5% (thin/bright), 4=50% (square), 8=~100% (fat)

int8_t  presetOct[8];  // editable per-preset octave offset
uint8_t presetPW[8];   // editable per-preset pulse width (1-8)
uint8_t editPreset = 0, editParam = 0;  // preset editor state

uint8_t colPreset[COLS] = {1, 4, 2, 3, 0, 5, 6, 7};  // col1=BS(BD), col2=HI(SN)
uint8_t colMute[COLS]   = {0, 0, 0, 0, 0, 0, 0, 0};   // 0=active, 1=muted (speaker only)
uint8_t colMidi[COLS]   = {1, 2, 3, 4, 5, 6, 7, 8};  // MIDI channel per column (1-16)
uint8_t colMidiNote[COLS];  // currently sounding MIDI note per column; 0xFF=none

uint8_t sndCurCol = 0, sndCurRow = 0;  // 0=preset, 1=mute, 2=midi ch

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
volatile uint8_t  voicePW[MAX_VOICES];  // duty threshold 0-255 (128 = 50% square)
uint8_t colVoice[COLS];  // 0xFF = no voice assigned

ISR(TIMER3_COMPA_vect) {
  static uint16_t ph[MAX_VOICES];
  ph[0] += voiceDelta[0]; ph[1] += voiceDelta[1];
  ph[2] += voiceDelta[2]; ph[3] += voiceDelta[3];
  // Duty cycle: voice is "on" when high byte of phase < voicePW threshold
  uint8_t out = (((uint8_t)(ph[0] >> 8) < voicePW[0]) & voiceActive[0])
              ^ (((uint8_t)(ph[1] >> 8) < voicePW[1]) & voiceActive[1])
              ^ (((uint8_t)(ph[2] >> 8) < voicePW[2]) & voiceActive[2])
              ^ (((uint8_t)(ph[3] >> 8) < voicePW[3]) & voiceActive[3]);
  if (out) { PORTC = (PORTC | (1 << 6)) & ~(1 << 7); }
  else     { PORTC = (PORTC & ~(1 << 6)) | (1 << 7); }
}

void audioBegin() {
  DDRC  |=  (1 << 6) | (1 << 7);
  PORTC &= ~((1 << 6) | (1 << 7));
  for (uint8_t v = 0; v < MAX_VOICES; v++) voicePW[v] = 128;  // default 50% duty
  TCCR3A = 0;
  TCCR3B = (1 << WGM32) | (1 << CS30);  // CTC, prescaler=1
  OCR3A  = (uint16_t)(F_CPU / SAMPLE_RATE) - 1;  // 399 at 40kHz
  // TIMSK3 stays 0; voiceOn() enables it when a note starts
}

void voiceOn(uint8_t v, uint16_t freq, uint8_t pw) {
  uint16_t delta = (uint16_t)((uint32_t)freq * 65536UL / SAMPLE_RATE);
  cli();
  voiceDelta[v]  = delta;
  voicePW[v]     = pw;
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
// [834-835] bpm  [836-843] colPreset  [844-851] colMute  [852-859] colMidi
// [860-867] presetOct  [868-875] presetPW
static const uint16_t EEPROM_BASE   = 16;
static const uint8_t  EEPROM_MAGIC0 = 0xA7;
static const uint8_t  EEPROM_MAGIC1 = 0x5E;  // bumped: mute+preset params

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
  for (uint8_t i = 0; i < COLS; i++) EEPROM.update(addr++, colMute[i]);
  for (uint8_t i = 0; i < COLS; i++) EEPROM.update(addr++, colMidi[i]);
  for (uint8_t i = 0; i < 8;    i++) EEPROM.update(addr++, (uint8_t)presetOct[i]);
  for (uint8_t i = 0; i < 8;    i++) EEPROM.update(addr++, presetPW[i]);
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
  for (uint8_t i = 0; i < COLS; i++) colMute[i]   = EEPROM.read(addr++);
  for (uint8_t i = 0; i < COLS; i++) colMidi[i]   = EEPROM.read(addr++);
  for (uint8_t i = 0; i < 8;    i++) presetOct[i] = (int8_t)EEPROM.read(addr++);
  for (uint8_t i = 0; i < 8;    i++) presetPW[i]  = EEPROM.read(addr++);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  arduboy.boot();  // skip boot logo/sound to avoid Timer3 ISR conflict
  arduboy.clear();
  arduboy.display();
  arduboy.setFrameRate(30);
  memset(grid, 0xFF, sizeof(grid));
  memset(patNote, 0xFF, sizeof(patNote));
  memset(colVoice,    0xFF, sizeof(colVoice));
  memset(colMidiNote, 0xFF, sizeof(colMidiNote));
  for (uint8_t i = 0; i < 8; i++) {
    presetOct[i] = (int8_t)pgm_read_byte(&DEF_PRESET_OCT[i]);
    presetPW[i]  = pgm_read_byte(&DEF_PRESET_PW[i]);
  }
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
  arduboy.drawFastHLine(0, CELL_H * 4,   128, WHITE);

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

  // Mute row
  for (uint8_t c = 0; c < COLS; c++) {
    uint8_t cx = c * CELL_W;
    uint8_t cy = CELL_H * 2;
    bool cur = (c == sndCurCol && sndCurRow == 1);
    bool muted = (colMute[c] != 0);
    if (cur) {
      arduboy.fillRect(cx + 1, cy + 1, CELL_W - 2, CELL_H - 2, WHITE);
      arduboy.setTextColor(BLACK);
    } else {
      arduboy.setTextColor(WHITE);
    }
    arduboy.setCursor(cx + 2, cy + 1);
    arduboy.print(muted ? F("--") : F("ON"));
  }
  // MIDI channel row
  for (uint8_t c = 0; c < COLS; c++) {
    uint8_t cx = c * CELL_W;
    uint8_t cy = CELL_H * 3;
    bool cur = (c == sndCurCol && sndCurRow == 2);
    if (cur) {
      arduboy.fillRect(cx + 1, cy + 1, CELL_W - 2, CELL_H - 2, WHITE);
      arduboy.setTextColor(BLACK);
    } else {
      arduboy.setTextColor(WHITE);
    }
    arduboy.setCursor(cx + 2, cy + 1);
    uint8_t ch = colMidi[c];
    if (ch < 10) arduboy.print(' ');
    arduboy.print(ch);
  }
  arduboy.setTextColor(WHITE);

  arduboy.setCursor(2, CELL_H * 4 + 4);
  arduboy.print(F("A:prs/mute A+<>:ch"));
  arduboy.setCursor(2, CELL_H * 4 + 13);
  arduboy.print(F("B:back  B+^:synths"));
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

// ── MIDI helpers ──────────────────────────────────────────────────────────────
void midiNoteOn(uint8_t ch, uint8_t note, uint8_t vel) {
  midiEventPacket_t ev = {0x09, (uint8_t)(0x90 | (ch - 1)), note, vel};
  MidiUSB.sendMIDI(ev);
}

void midiNoteOff(uint8_t ch, uint8_t note) {
  midiEventPacket_t ev = {0x08, (uint8_t)(0x80 | (ch - 1)), note, 0};
  MidiUSB.sendMIDI(ev);
}

void allMidiOff() {
  for (uint8_t c = 0; c < COLS; c++) {
    if (colMidiNote[c] != 0xFF) {
      midiNoteOff(colMidi[c], colMidiNote[c]);
      colMidiNote[c] = 0xFF;
    }
  }
  MidiUSB.flush();
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
      if (colVoice[c] < MAX_VOICES)  { voiceOff(colVoice[c]); colVoice[c] = 0xFF; }
      if (colMidiNote[c] != 0xFF)    { midiNoteOff(colMidi[c], colMidiNote[c]); colMidiNote[c] = 0xFF; }
    } else if (n != NOTE_EMPTY) {
      uint8_t preset = colPreset[c];
      // Internal voice (octave-shifted, pulse-width from preset)
      if (!colMute[c]) {
        int8_t  oct = presetOct[preset];
        int16_t adj = (int16_t)n + (int16_t)oct * 12;
        if (adj < NOTE_MIN) adj = NOTE_MIN;
        if (adj > NOTE_MAX) adj = NOTE_MAX;
        uint16_t freq = midiToFreq((uint8_t)adj);
        // pw 1-8 → threshold 31-255: (pw*32)-1
        uint8_t pw = (uint8_t)((uint16_t)presetPW[preset] * 32 - 1);
        if (freq > 0) {
          if (colVoice[c] >= MAX_VOICES) colVoice[c] = findFreeVoice();
          if (colVoice[c] < MAX_VOICES)  voiceOn(colVoice[c], freq, pw);
        }
      }
      // MIDI output always sends regardless of mute (DAW controls its own volume)
      if (colMidiNote[c] != 0xFF) midiNoteOff(colMidi[c], colMidiNote[c]);
      midiNoteOn(colMidi[c], n, 100);
      colMidiNote[c] = n;
    }
    // NOTE_EMPTY: sustain — leave voice and MIDI note running
  }
  MidiUSB.flush();

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
    allMidiOff();
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
  bool justRelA = arduboy.justReleased(A_BUTTON);
  bool wasClean = aPressClean;

  bool bHeld = arduboy.pressed(B_BUTTON);
  if (bHeld && arduboy.justPressed(UP_BUTTON)) {
    editPreset = colPreset[sndCurCol];
    editParam  = 0;
    resetInputState();
    screen = SCR_PRESET;
    return;
  }
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
    if (arduboy.justPressed(A_BUTTON)) {
      aPressClean = noDir;
    } else if (!noDir && sndCurRow == 2) {
      aPressClean = false;
      if (checkRepeat(LEFT_BUTTON,  2) && colMidi[sndCurCol] > 1)   colMidi[sndCurCol]--;
      if (checkRepeat(RIGHT_BUTTON, 3) && colMidi[sndCurCol] < 16)  colMidi[sndCurCol]++;
    }
  } else {
    if (justRelA && wasClean) {
      if (sndCurRow == 0) colPreset[sndCurCol] = (colPreset[sndCurCol] + 1) & 7;
      else if (sndCurRow == 1) colMute[sndCurCol] ^= 1;
    }
    if (checkRepeat(UP_BUTTON,    0) && sndCurRow > 0)        sndCurRow--;
    if (checkRepeat(DOWN_BUTTON,  1) && sndCurRow < 2)        sndCurRow++;
    if (checkRepeat(LEFT_BUTTON,  2) && sndCurCol > 0)        sndCurCol--;
    if (checkRepeat(RIGHT_BUTTON, 3) && sndCurCol < COLS - 1) sndCurCol++;
  }
}

// ── Draw: Preset Editor ───────────────────────────────────────────────────────
void drawPreset() {
  uint8_t p = editPreset;
  const uint8_t SX = 26, SW = 72;  // slider start x, total width

  arduboy.drawRect(0, 0, 128, 64, WHITE);
  arduboy.drawFastHLine(0,  9, 128, WHITE);
  arduboy.drawFastHLine(0, 18, 128, WHITE);
  arduboy.drawFastHLine(0, 27, 128, WHITE);

  // Header: preset number + name
  arduboy.setTextColor(WHITE);
  arduboy.setCursor(4, 1);
  arduboy.print(F("P")); arduboy.print(p + 1); arduboy.print(':');
  arduboy.print((char)pgm_read_byte(&PRESET_NAME[p][0]));
  arduboy.print((char)pgm_read_byte(&PRESET_NAME[p][1]));
  arduboy.setCursor(50, 1);
  arduboy.print(F("<>:prst  A:reset"));

  // OCT slider row (y=9)
  bool octSel = (editParam == 0);
  if (octSel) { arduboy.fillRect(1, 10, 126, 7, WHITE); arduboy.setTextColor(BLACK); }
  arduboy.setCursor(3, 10);
  arduboy.print(F("OCT"));
  {
    int8_t  oct    = presetOct[p];
    uint8_t thumbX = (uint8_t)(SX + (uint16_t)(oct + 4) * SW / 8);
    uint8_t ctrX   = SX + SW / 2;
    uint8_t col    = octSel ? BLACK : WHITE;
    arduboy.drawFastHLine(SX, 13, SW, col);
    arduboy.drawFastVLine(ctrX, 12, 3, col);   // center tick
    arduboy.fillRect(thumbX - 1, 11, 3, 5, col);  // thumb
    arduboy.setCursor(102, 10);
    if (oct >= 0) arduboy.print('+');
    arduboy.print(oct);
  }

  // PW slider row (y=18)
  arduboy.setTextColor(WHITE);
  bool pwSel = (editParam == 1);
  if (pwSel) { arduboy.fillRect(1, 19, 126, 7, WHITE); arduboy.setTextColor(BLACK); }
  arduboy.setCursor(3, 19);
  arduboy.print(F("PW "));
  {
    uint8_t pw     = presetPW[p];
    uint8_t fill   = (uint8_t)((uint16_t)(pw - 1) * SW / 7);
    uint8_t col    = pwSel ? BLACK : WHITE;
    arduboy.drawFastHLine(SX, 22, SW, col);
    if (fill > 0) arduboy.fillRect(SX, 20, fill, 5, col);
    arduboy.drawFastVLine(SX + SW - 1, 21, 3, col);  // right-end tick
    arduboy.setCursor(102, 19);
    arduboy.print(pw);
    arduboy.print(F("/8"));
  }

  arduboy.setTextColor(WHITE);
  arduboy.setCursor(4, 30);
  arduboy.print(F("ud:param  A+<>:change"));
  arduboy.setCursor(4, 39);
  arduboy.print(F("B:back  A:rst preset"));
}

// ── Input: Preset Editor ──────────────────────────────────────────────────────
void handlePresetInput() {
  bool justRelA = arduboy.justReleased(A_BUTTON);
  bool wasClean = aPressClean;

  if (arduboy.justPressed(B_BUTTON)) {
    resetInputState();
    screen = SCR_SOUND;
    return;
  }

  bool aHeld = arduboy.pressed(A_BUTTON);
  bool noDir = !arduboy.pressed(UP_BUTTON)   && !arduboy.pressed(DOWN_BUTTON) &&
               !arduboy.pressed(LEFT_BUTTON) && !arduboy.pressed(RIGHT_BUTTON);

  if (aHeld != aWasPressed) { resetInputState(); aWasPressed = aHeld; }

  if (aHeld) {
    if (arduboy.justPressed(A_BUTTON)) {
      aPressClean = noDir;
    } else if (!noDir) {
      aPressClean = false;
      if (editParam == 0) {
        if (checkRepeat(LEFT_BUTTON,  2) && presetOct[editPreset] > -4) presetOct[editPreset]--;
        if (checkRepeat(RIGHT_BUTTON, 3) && presetOct[editPreset] < +4) presetOct[editPreset]++;
      } else {
        if (checkRepeat(LEFT_BUTTON,  2) && presetPW[editPreset] > 1) presetPW[editPreset]--;
        if (checkRepeat(RIGHT_BUTTON, 3) && presetPW[editPreset] < 8) presetPW[editPreset]++;
      }
    }
  } else {
    if (justRelA && wasClean) {
      // Reset this preset to factory defaults
      presetOct[editPreset] = (int8_t)pgm_read_byte(&DEF_PRESET_OCT[editPreset]);
      presetPW[editPreset]  = pgm_read_byte(&DEF_PRESET_PW[editPreset]);
    }
    if (checkRepeat(UP_BUTTON,    0) && editParam > 0) editParam--;
    if (checkRepeat(DOWN_BUTTON,  1) && editParam < 1) editParam++;
    if (checkRepeat(LEFT_BUTTON,  2)) editPreset = (editPreset + 7) & 7;
    if (checkRepeat(RIGHT_BUTTON, 3)) editPreset = (editPreset + 1) & 7;
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
      case SCR_PRESET:   drawPreset();   break;
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
    case SCR_PRESET:
      handlePresetInput();
      arduboy.clear();
      drawPreset();
      break;
  }
  arduboy.display();
}
