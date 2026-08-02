#include <Arduboy2.h>
#include <EEPROM.h>
#include <MIDIUSB.h>
#include <SPI.h>
#include <avr/interrupt.h>

Arduboy2 arduboy;

// ── Constants ─────────────────────────────────────────────────────────────────
static const uint8_t CELL_H     = 9;
static const uint8_t DBL_WINDOW = 10;

// ── Screen ────────────────────────────────────────────────────────────────────
enum Screen : uint8_t { SCR_TRACKER, SCR_PATTERN, SCR_SETTINGS, SCR_SOUND, SCR_PRESET, SCR_SONGS };
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
// Waveforms: 0=SQ (square), 1=TR (triangle PDM), 2=SA (sawtooth PDM), 3=NO (noise LFSR)
static const char    WAVE_NAMES[4][3]    PROGMEM = { "SQ","TR","SA","NO" };
static const uint8_t DEF_PRESET_WAVE[8] PROGMEM = {  0,   0,   0,   1,   3,   0,   3,   1  };
// LD=SQ, BS=SQ, AP=SQ, CH=TR, HI=NO, LO=SQ, PC=NO, PD=TR

int8_t  presetOct[8];   // editable per-preset octave offset
uint8_t presetPW[8];    // editable per-preset pulse width (1-8)
uint8_t presetWave[8];  // editable per-preset waveform (0-3)
uint8_t editPreset = 0, editParam = 0;  // preset editor state (editParam: 0=OCT,1=PW,2=WAVE)

uint8_t colPreset[COLS] = {1, 4, 2, 3, 0, 5, 6, 7};  // col1=BS(BD), col2=HI(SN)
uint8_t colMute[COLS]   = {0, 0, 0, 0, 0, 0, 0, 0};   // 0=active, 1=muted (speaker only)
uint8_t colMidi[COLS]   = {1, 2, 3, 4, 5, 6, 7, 8};  // MIDI channel per column (1-16)
uint8_t colMidiNote[COLS];  // currently sounding MIDI note per column; 0xFF=none

uint8_t sndCurCol = 0, sndCurRow = 0;  // 0=preset, 1=mute, 2=midi ch

// ── FX Flash / Song Slots ─────────────────────────────────────────────────────
// W25Q128 on D7 (PD7). Last 128KB of 16MB chip = 32 × 4KB sectors.
// Slot layout: [0-1] magic  [2-7] name(6)  [8-327] grid  [328-839] patNote
//              [840-841] bpm  [842-849] colPreset  [850-857] colMute
//              [858-865] colMidi  [866-873] presetOct  [874-881] presetPW
//              [882-889] presetWave   total 890 bytes, rest 0xFF
static const uint32_t FX_SONG_BASE  = 0xFE0000UL;  // base address
static const uint32_t FX_SLOT_SIZE  = 0x1000UL;    // 4096 bytes per slot (1 erase sector)
static const uint8_t  SONG_SLOTS    = 32;
static const uint8_t  SONGS_VIS     = 5;            // visible rows on songs screen

uint8_t songSlot    = 0;   // selected slot index (0-31)
uint8_t songScroll  = 0;   // scroll offset for song list
bool    songNaming  = false;
uint8_t songNameCur = 0;   // cursor within name (0-5)
char    songEditName[7];   // name being edited (null-terminated)
static uint8_t songCache[SONGS_VIS][8];  // cached slot headers, refreshed on entry/scroll/save

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
volatile uint8_t  voicePW[MAX_VOICES];    // duty threshold 0-255 (128 = 50% square)
volatile uint8_t  voiceWave[MAX_VOICES];  // 0=square, 1=triangle, 2=sawtooth, 3=noise
volatile int16_t  voiceErr[MAX_VOICES];   // PDM error accumulators for TR/SA
uint8_t colVoice[COLS];  // 0xFF = no voice assigned

ISR(TIMER3_COMPA_vect) {
  static uint16_t ph[MAX_VOICES];
  static uint16_t nsr[MAX_VOICES] = {0xACE1, 0x7E5B, 0x3CA9, 0x1F83};  // noise LFSR state
  uint8_t out = 0;
  for (uint8_t v = 0; v < MAX_VOICES; v++) {
    if (!voiceActive[v]) continue;
    ph[v] += voiceDelta[v];
    uint8_t bit;
    uint8_t w = voiceWave[v];
    if (w == 0) {
      bit = ((uint8_t)(ph[v] >> 8) < voicePW[v]);
    } else if (w == 1) {
      uint16_t p  = ph[v];
      uint8_t tri = (p < 0x8000) ? (uint8_t)(p >> 7) : (uint8_t)((0xFFFF - p) >> 7);
      int16_t e   = voiceErr[v] + ((int16_t)tri - 128);
      bit          = (e >= 0);
      voiceErr[v]  = e - (bit ? 256 : 0);
    } else if (w == 2) {
      uint8_t saw = (uint8_t)(ph[v] >> 8);
      int16_t e   = voiceErr[v] + ((int16_t)saw - 128);
      bit          = (e >= 0);
      voiceErr[v]  = e - (bit ? 256 : 0);
    } else {
      nsr[v] = (nsr[v] >> 1) ^ (uint16_t)(-(nsr[v] & 1u) & 0xB400u);
      bit     = (uint8_t)(nsr[v] & 1u);
    }
    out ^= bit;
  }
  if (out) { PORTC = (PORTC | (1 << 6)) & ~(1 << 7); }
  else     { PORTC = (PORTC & ~(1 << 6)) | (1 << 7); }
}

void audioBegin() {
  DDRC  |=  (1 << 6) | (1 << 7);
  PORTC &= ~((1 << 6) | (1 << 7));
  for (uint8_t v = 0; v < MAX_VOICES; v++) {
    voicePW[v]   = 128;
    voiceWave[v] = 0;
    voiceErr[v]  = 0;
  }
  TCCR3A = 0;
  TCCR3B = (1 << WGM32) | (1 << CS30);  // CTC, prescaler=1
  OCR3A  = (uint16_t)(F_CPU / SAMPLE_RATE) - 1;  // 399 at 40kHz
  // TIMSK3 stays 0; voiceOn() enables it when a note starts
}

void voiceOn(uint8_t v, uint16_t freq, uint8_t pw, uint8_t wave) {
  uint16_t delta = (uint16_t)((uint32_t)freq * 65536UL / SAMPLE_RATE);
  cli();
  voiceDelta[v]  = delta;
  voicePW[v]     = pw;
  voiceWave[v]   = wave;
  voiceErr[v]    = 0;
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
// [860-867] presetOct  [868-875] presetPW  [876-883] presetWave
static const uint16_t EEPROM_BASE   = 16;
static const uint8_t  EEPROM_MAGIC0 = 0xA7;
static const uint8_t  EEPROM_MAGIC1 = 0x5F;  // bumped: added waveform per preset

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
  for (uint8_t i = 0; i < 8;    i++) EEPROM.update(addr++, presetWave[i]);
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
  for (uint8_t i = 0; i < 8;    i++) presetOct[i]  = (int8_t)EEPROM.read(addr++);
  for (uint8_t i = 0; i < 8;    i++) presetPW[i]   = EEPROM.read(addr++);
  for (uint8_t i = 0; i < 8;    i++) presetWave[i] = EEPROM.read(addr++);
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
    presetOct[i]  = (int8_t)pgm_read_byte(&DEF_PRESET_OCT[i]);
    presetPW[i]   = pgm_read_byte(&DEF_PRESET_PW[i]);
    presetWave[i] = pgm_read_byte(&DEF_PRESET_WAVE[i]);
  }
  loadSong();
  audioBegin();
  fxBegin();
  memcpy(songEditName, "SONG 1", 7);
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

  // Extra pixel above beat-boundary rows (steps 4, 8, 12) to form a thicker divider
  for (uint8_t vr = 0; vr < PAT_VISIBLE; vr++) {
    uint8_t step = vr + patScroll;
    if (step > 0 && step % 4 == 0)
      arduboy.drawFastHLine(0, vr * CELL_H + 1, 128, WHITE);
  }

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

// ── FX Flash driver ───────────────────────────────────────────────────────────
static inline void fxSel()   { PORTE &= ~(1 << 6); }
static inline void fxDesel() { PORTE |=  (1 << 6); }

static void fxWaitBusy() {
  fxSel();
  SPI.transfer(0x05);  // RDSR1
  while (SPI.transfer(0) & 0x01);
  fxDesel();
}

static void fxWren() {
  fxSel(); SPI.transfer(0x06); fxDesel();  // Write Enable
}

void fxBegin() {
  DDRE  |=  (1 << 6);   // PE6 = Arduino pin 7 = FX flash CS
  PORTE |=  (1 << 6);   // CS deselected (high)
  SPI.begin();
}

void fxRead(uint32_t addr, void* buf, uint16_t len) {
  fxSel();
  SPI.transfer(0x03);
  SPI.transfer((uint8_t)(addr >> 16));
  SPI.transfer((uint8_t)(addr >> 8));
  SPI.transfer((uint8_t) addr);
  uint8_t* p = (uint8_t*)buf;
  for (uint16_t i = 0; i < len; i++) p[i] = SPI.transfer(0);
  fxDesel();
}

// Write 1-255 bytes within one 256-byte page (must not cross page boundary)
static void fxWriteSmall(uint32_t addr, const uint8_t* buf, uint8_t len) {
  fxWren();
  fxSel();
  SPI.transfer(0x02);
  SPI.transfer((uint8_t)(addr >> 16));
  SPI.transfer((uint8_t)(addr >> 8));
  SPI.transfer((uint8_t) addr);
  for (uint8_t i = 0; i < len; i++) SPI.transfer(buf[i]);
  fxDesel();
  fxWaitBusy();
}


static void fxEraseSector(uint32_t addr) {
  fxWren();
  fxSel();
  SPI.transfer(0x20);  // Sector Erase 4KB
  SPI.transfer((uint8_t)(addr >> 16));
  SPI.transfer((uint8_t)(addr >> 8));
  SPI.transfer((uint8_t) addr);
  fxDesel();
  fxWaitBusy();
}

bool fxSlotValid(uint8_t slot) {
  uint8_t m[2];
  fxRead(FX_SONG_BASE + (uint32_t)slot * FX_SLOT_SIZE, m, 2);
  return m[0] == EEPROM_MAGIC0 && m[1] == EEPROM_MAGIC1;
}

void refreshSongCache() {
  for (uint8_t r = 0; r < SONGS_VIS; r++) {
    uint8_t s = r + songScroll;
    if (s < SONG_SLOTS)
      fxRead(FX_SONG_BASE + (uint32_t)s * FX_SLOT_SIZE, songCache[r], 8);
    else
      memset(songCache[r], 0, 8);
  }
}

void fxLoadSlot(uint8_t slot) {
  uint32_t base = FX_SONG_BASE + (uint32_t)slot * FX_SLOT_SIZE;
  arduboy.clear();
  arduboy.setTextColor(WHITE);
  arduboy.setCursor(16, 28); arduboy.print(F("LOADING SLOT "));
  if (slot < 9) arduboy.print(' ');
  arduboy.print(slot + 1);
  arduboy.display();

  fxRead(base + 8,   (uint8_t*)grid,    sizeof(grid));
  fxRead(base + 328, (uint8_t*)patNote, sizeof(patNote));
  uint8_t misc[50];
  fxRead(base + 840, misc, 50);
  bpm = (uint16_t)misc[0] | ((uint16_t)misc[1] << 8);
  memcpy(colPreset,  misc +  2, 8);
  memcpy(colMute,    misc + 10, 8);
  memcpy(colMidi,    misc + 18, 8);
  memcpy(presetOct,  misc + 26, 8);
  memcpy(presetPW,   misc + 34, 8);
  memcpy(presetWave, misc + 42, 8);
}

void fxSaveSlot(uint8_t slot, const char* name6) {
  uint32_t base = FX_SONG_BASE + (uint32_t)slot * FX_SLOT_SIZE;
  arduboy.clear();
  arduboy.setTextColor(WHITE);
  arduboy.setCursor(16, 28); arduboy.print(F("SAVING SLOT "));
  if (slot < 9) arduboy.print(' ');
  arduboy.print(slot + 1);
  arduboy.display();

  fxEraseSector(base);

  // Page 0 (bytes 0-255): magic(2) + name(6) + grid[0..247](248)
  uint8_t hdr[8] = { EEPROM_MAGIC0, EEPROM_MAGIC1 };
  memcpy(hdr + 2, name6, 6);
  fxWriteSmall(base,     hdr,               8);
  fxWriteSmall(base + 8, (uint8_t*)grid,  248);

  // Page 1 (bytes 256-511): grid[248..319](72) + patNote[0..183](184)
  fxWriteSmall(base + 256, (uint8_t*)grid + 248, 72);
  fxWriteSmall(base + 328, (uint8_t*)patNote,   184);

  // Page 2 (bytes 512-767): patNote[184..439](256) — split into two 128-byte writes
  fxWriteSmall(base + 512, (uint8_t*)patNote + 184, 128);
  fxWriteSmall(base + 640, (uint8_t*)patNote + 312, 128);

  // Page 3 (bytes 768-1023): patNote[440..511](72) + misc(50)
  fxWriteSmall(base + 768, (uint8_t*)patNote + 440, 72);
  uint8_t misc[50];
  misc[0] = (uint8_t)(bpm & 0xFF);
  misc[1] = (uint8_t)(bpm >> 8);
  memcpy(misc +  2, colPreset,  8);
  memcpy(misc + 10, colMute,    8);
  memcpy(misc + 18, colMidi,    8);
  memcpy(misc + 26, presetOct,  8);
  memcpy(misc + 34, presetPW,   8);
  memcpy(misc + 42, presetWave, 8);
  fxWriteSmall(base + 840, misc, 50);
}

// ── Character helpers for slot naming ─────────────────────────────────────────
// Charset: 0=' ', 1-26='A'-'Z', 27-36='0'-'9'  (37 total)
static char charFromIdx(uint8_t idx) {
  if (idx == 0)   return ' ';
  if (idx <= 26)  return (char)('A' + idx - 1);
  return (char)('0' + idx - 27);
}

static uint8_t idxFromChar(char c) {
  if (c >= 'A' && c <= 'Z') return (uint8_t)(c - 'A' + 1);
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0' + 27);
  return 0;  // space or unknown
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
        uint8_t pw   = (uint8_t)((uint16_t)presetPW[preset] * 32 - 1);
        uint8_t wave = presetWave[preset];
        if (freq > 0) {
          if (colVoice[c] >= MAX_VOICES) colVoice[c] = findFreeVoice();
          if (colVoice[c] < MAX_VOICES)  voiceOn(colVoice[c], freq, pw, wave);
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
      if (playing) { allVoicesOff(); allMidiOff(); playing = false; memset(colVoice, 0xFF, sizeof(colVoice)); }
      songSlot   = 0;
      songScroll = 0;
      songNaming = false;
      resetInputState();
      refreshSongCache();
      screen = SCR_SONGS;
      return;
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
  arduboy.drawFastHLine(0, 36, 128, WHITE);

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
    arduboy.drawFastVLine(ctrX, 12, 3, col);
    arduboy.fillRect(thumbX - 1, 11, 3, 5, col);
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
    uint8_t pw   = presetPW[p];
    uint8_t fill = (uint8_t)((uint16_t)(pw - 1) * SW / 7);
    uint8_t col  = pwSel ? BLACK : WHITE;
    arduboy.drawFastHLine(SX, 22, SW, col);
    if (fill > 0) arduboy.fillRect(SX, 20, fill, 5, col);
    arduboy.drawFastVLine(SX + SW - 1, 21, 3, col);
    arduboy.setCursor(102, 19);
    arduboy.print(pw);
    arduboy.print(F("/8"));
  }

  // WAVE selector row (y=27) — 4 options: SQ TR SA NO
  arduboy.setTextColor(WHITE);
  bool waveSel = (editParam == 2);
  if (waveSel) { arduboy.fillRect(1, 28, 126, 7, WHITE); arduboy.setTextColor(BLACK); }
  arduboy.setCursor(3, 28);
  arduboy.print(F("WV"));
  {
    uint8_t selWv = presetWave[p];
    for (uint8_t i = 0; i < 4; i++) {
      uint8_t wx = 28 + i * 25;
      bool cur = (selWv == i);
      if (cur) {
        uint8_t bc = waveSel ? BLACK : WHITE;
        uint8_t tc = waveSel ? WHITE : BLACK;
        arduboy.fillRect(wx - 1, 28, 14, 7, bc);
        arduboy.setTextColor(tc);
      } else {
        arduboy.setTextColor(waveSel ? BLACK : WHITE);
      }
      arduboy.setCursor(wx, 28);
      arduboy.print((char)pgm_read_byte(&WAVE_NAMES[i][0]));
      arduboy.print((char)pgm_read_byte(&WAVE_NAMES[i][1]));
    }
  }

  arduboy.setTextColor(WHITE);
  arduboy.setCursor(4, 38);
  arduboy.print(F("ud:param  A+<>:edit"));
  arduboy.setCursor(4, 47);
  arduboy.print(F("B:back  A:rst prst"));
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
      } else if (editParam == 1) {
        if (checkRepeat(LEFT_BUTTON,  2) && presetPW[editPreset] > 1) presetPW[editPreset]--;
        if (checkRepeat(RIGHT_BUTTON, 3) && presetPW[editPreset] < 8) presetPW[editPreset]++;
      } else {
        if (checkRepeat(LEFT_BUTTON,  2) && presetWave[editPreset] > 0) presetWave[editPreset]--;
        if (checkRepeat(RIGHT_BUTTON, 3) && presetWave[editPreset] < 3) presetWave[editPreset]++;
      }
    }
  } else {
    if (justRelA && wasClean) {
      // Reset this preset to factory defaults
      presetOct[editPreset]  = (int8_t)pgm_read_byte(&DEF_PRESET_OCT[editPreset]);
      presetPW[editPreset]   = pgm_read_byte(&DEF_PRESET_PW[editPreset]);
      presetWave[editPreset] = pgm_read_byte(&DEF_PRESET_WAVE[editPreset]);
    }
    if (checkRepeat(UP_BUTTON,    0) && editParam > 0) editParam--;
    if (checkRepeat(DOWN_BUTTON,  1) && editParam < 2) editParam++;
    if (checkRepeat(LEFT_BUTTON,  2)) editPreset = (editPreset + 7) & 7;
    if (checkRepeat(RIGHT_BUTTON, 3)) editPreset = (editPreset + 1) & 7;
  }
}

// ── Draw: Song Manager ────────────────────────────────────────────────────────
void drawSongs() {
  arduboy.drawRect(0, 0, 128, 64, WHITE);
  arduboy.drawFastHLine(0,  9, 128, WHITE);
  arduboy.drawFastHLine(0, 54, 128, WHITE);

  // Header
  arduboy.setTextColor(WHITE);
  arduboy.setCursor(4, 1);
  arduboy.print(F("SONGS"));
  if (songNaming) {
    arduboy.setCursor(40, 1);
    arduboy.print(F("^v:char <>:cur A:ok"));
  } else {
    arduboy.setCursor(40, 1);
    arduboy.print(F("A:load  A+^:save"));
  }

  // Slot list (5 visible rows at y=10,19,28,37,46)
  for (uint8_t r = 0; r < SONGS_VIS; r++) {
    uint8_t s = r + songScroll;
    if (s >= SONG_SLOTS) break;
    uint8_t y = 10 + r * 9;
    bool sel = (s == songSlot);

    if (sel) {
      arduboy.fillRect(1, y, 126, 8, WHITE);
      arduboy.setTextColor(BLACK);
    } else {
      arduboy.setTextColor(WHITE);
    }

    // Slot number
    arduboy.setCursor(3, y + 1);
    if (s < 9) arduboy.print(' ');
    arduboy.print(s + 1);
    arduboy.print(' ');

    // Name or naming editor
    if (sel && songNaming) {
      for (uint8_t i = 0; i < 6; i++) {
        uint8_t cx = 21 + i * 6;
        if (i == songNameCur) {
          arduboy.fillRect(cx - 1, y, 7, 8, BLACK);
          arduboy.setTextColor(WHITE);
          arduboy.setCursor(cx, y + 1);
          arduboy.print(songEditName[i]);
          arduboy.setTextColor(BLACK);
        } else {
          arduboy.setCursor(cx, y + 1);
          arduboy.print(songEditName[i]);
        }
      }
    } else {
      uint8_t *hdr = songCache[r];
      arduboy.setCursor(21, y + 1);
      if (hdr[0] == EEPROM_MAGIC0 && hdr[1] == EEPROM_MAGIC1) {
        for (uint8_t i = 0; i < 6; i++) arduboy.print((char)hdr[i + 2]);
      } else {
        arduboy.print(F("------"));
      }
    }
  }

  arduboy.setTextColor(WHITE);
  arduboy.setCursor(4, 56);
  if (songNaming) arduboy.print(F("B:cancel"));
  else            arduboy.print(F("A+v:rename  B:back"));
}

// ── Input: Song Manager ───────────────────────────────────────────────────────
void handleSongsInput() {
  bool justRelA = arduboy.justReleased(A_BUTTON);
  bool wasClean = aPressClean;

  bool aHeld = arduboy.pressed(A_BUTTON);
  bool noDir = !arduboy.pressed(UP_BUTTON)   && !arduboy.pressed(DOWN_BUTTON) &&
               !arduboy.pressed(LEFT_BUTTON) && !arduboy.pressed(RIGHT_BUTTON);

  if (songNaming) {
    if (arduboy.justPressed(B_BUTTON)) {
      songNaming = false;
      resetInputState();
      return;
    }
    if (aHeld != aWasPressed) { resetInputState(); aWasPressed = aHeld; }
    if (aHeld) {
      if (arduboy.justPressed(A_BUTTON)) aPressClean = noDir;
    } else {
      if (justRelA && wasClean) {
        // Confirm name and save
        fxSaveSlot(songSlot, songEditName);
        songNaming = false;
        refreshSongCache();
        resetInputState();
        return;
      }
      if (checkRepeat(UP_BUTTON, 0)) {
        uint8_t idx = idxFromChar(songEditName[songNameCur]);
        songEditName[songNameCur] = charFromIdx((idx + 1) % 37);
      }
      if (checkRepeat(DOWN_BUTTON, 1)) {
        uint8_t idx = idxFromChar(songEditName[songNameCur]);
        songEditName[songNameCur] = charFromIdx((idx + 36) % 37);
      }
      if (checkRepeat(LEFT_BUTTON,  2) && songNameCur > 0) songNameCur--;
      if (checkRepeat(RIGHT_BUTTON, 3) && songNameCur < 5) songNameCur++;
    }
    return;
  }

  // Normal list mode
  if (arduboy.justPressed(B_BUTTON)) {
    resetInputState();
    screen = SCR_TRACKER;
    return;
  }

  if (aHeld != aWasPressed) { resetInputState(); aWasPressed = aHeld; }

  if (aHeld) {
    if (arduboy.justPressed(A_BUTTON)) {
      aPressClean = noDir;
    } else if (!noDir) {
      aPressClean = false;
      if (arduboy.justPressed(UP_BUTTON)) {
        // Save to slot — use existing name if valid, else enter naming
        uint8_t hdr[8];
        fxRead(FX_SONG_BASE + (uint32_t)songSlot * FX_SLOT_SIZE, hdr, 8);
        if (hdr[0] == EEPROM_MAGIC0 && hdr[1] == EEPROM_MAGIC1) {
          char name6[7];
          memcpy(name6, hdr + 2, 6);
          name6[6] = '\0';
          fxSaveSlot(songSlot, name6);
          refreshSongCache();
          resetInputState();
        } else {
          // Empty slot — build default name "SONG nn"
          songEditName[0] = 'S'; songEditName[1] = 'O';
          songEditName[2] = 'N'; songEditName[3] = 'G';
          songEditName[4] = ' ';
          uint8_t n = songSlot + 1;
          if (n >= 10) { songEditName[4] = charFromIdx(n / 10 + 27); }
          songEditName[5] = charFromIdx(n % 10 + 27);
          songEditName[6] = '\0';
          songNameCur = 0;
          songNaming  = true;
          resetInputState();
        }
      } else if (arduboy.justPressed(DOWN_BUTTON)) {
        // Rename (enter name editor, save on confirm)
        uint8_t hdr[8];
        fxRead(FX_SONG_BASE + (uint32_t)songSlot * FX_SLOT_SIZE, hdr, 8);
        if (hdr[0] == EEPROM_MAGIC0 && hdr[1] == EEPROM_MAGIC1) {
          memcpy(songEditName, hdr + 2, 6);
        } else {
          uint8_t n = songSlot + 1;
          songEditName[0] = 'S'; songEditName[1] = 'O';
          songEditName[2] = 'N'; songEditName[3] = 'G';
          songEditName[4] = ' ';
          if (n >= 10) { songEditName[4] = charFromIdx(n / 10 + 27); }
          songEditName[5] = charFromIdx(n % 10 + 27);
        }
        songEditName[6] = '\0';
        songNameCur = 0;
        songNaming  = true;
        resetInputState();
      }
    }
  } else {
    if (justRelA && wasClean) {
      // Load slot
      if (fxSlotValid(songSlot)) {
        fxLoadSlot(songSlot);
        allVoicesOff();
        memset(colVoice,    0xFF, sizeof(colVoice));
        memset(colMidiNote, 0xFF, sizeof(colMidiNote));
        resetInputState();
        screen = SCR_TRACKER;
      }
    }
    if (checkRepeat(UP_BUTTON, 0) && songSlot > 0) {
      songSlot--;
      if (songSlot < songScroll) { songScroll = songSlot; refreshSongCache(); }
    }
    if (checkRepeat(DOWN_BUTTON, 1) && songSlot < SONG_SLOTS - 1) {
      songSlot++;
      if (songSlot >= songScroll + SONGS_VIS) { songScroll = songSlot - SONGS_VIS + 1; refreshSongCache(); }
    }
  }
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
  if (!arduboy.nextFrame()) return;
  arduboy.pollButtons();

  if (handlePlayStop()) {
    arduboy.clear();
    switch (screen) {
      case SCR_TRACKER:  drawTracker();  break;
      case SCR_PATTERN:  drawPattern();  break;
      case SCR_SETTINGS: drawSettings(); break;
      case SCR_SOUND:    drawSound();    break;
      case SCR_PRESET:   drawPreset();   break;
      case SCR_SONGS:    drawSongs();    break;
    }
    arduboy.display();
    return;
  }

  if (playing) stepPlay();

  // Input pass — handlers may change screen; draw pass re-reads screen after
  switch (screen) {
    case SCR_TRACKER:  handleTrackerInput();  break;
    case SCR_PATTERN:  handlePatternInput();  break;
    case SCR_SETTINGS: handleSettingsInput(); break;
    case SCR_SOUND:    handleSoundInput();    break;
    case SCR_PRESET:   handlePresetInput();   break;
    case SCR_SONGS:    handleSongsInput();    break;
  }

  // Draw pass — always matches current screen so transitions are instant
  arduboy.clear();
  switch (screen) {
    case SCR_TRACKER:  drawTracker();  break;
    case SCR_PATTERN:  drawPattern();  break;
    case SCR_SETTINGS: drawSettings(); break;
    case SCR_SOUND:    drawSound();    break;
    case SCR_PRESET:   drawPreset();   break;
    case SCR_SONGS:    drawSongs();    break;
  }
  arduboy.display();
}
