#include <Arduboy2.h>

Arduboy2 arduboy;

static const uint8_t COLS    = 8;
static const uint8_t ROWS    = 40;
static const uint8_t VISIBLE = 6;   // data rows visible at once (1 row reserved for header)
static const uint8_t CELL_W  = 16;  // pixels per column (128 / 8)
static const uint8_t CELL_H  = 9;   // pixels per row (1px border + 8px font)

// Grid data: 0xFF = empty cell, 0x00–0xFE = value
uint8_t grid[ROWS][COLS];

uint8_t curCol    = 0;
uint8_t curRow    = 0;
uint8_t scrollTop = 0;

// Key-repeat state: UP=0, DOWN=1, LEFT=2, RIGHT=3
uint8_t rptTimer[4];
static const uint8_t RPT_START = 20;
static const uint8_t RPT_RATE  = 5;

bool aWasPressed = false;

bool checkRepeat(uint8_t btn, uint8_t idx) {
  if (arduboy.justPressed(btn)) {
    rptTimer[idx] = RPT_START;
    return true;
  }
  if (arduboy.pressed(btn)) {
    if (rptTimer[idx] > 0) { rptTimer[idx]--; return false; }
    rptTimer[idx] = RPT_RATE;
    return true;
  }
  rptTimer[idx] = 0;
  return false;
}

void setup() {
  arduboy.begin();
  arduboy.setFrameRate(30);
  memset(grid, 0xFF, sizeof(grid));
  memset(rptTimer, 0, sizeof(rptTimer));
}

void printHex2(uint8_t val) {
  static const char h[] = "0123456789ABCDEF";
  arduboy.print(h[(val >> 4) & 0xF]);
  arduboy.print(h[val & 0xF]);
}

void drawGrid() {
  // Vertical lines for all 8 columns + right screen edge
  for (uint8_t c = 0; c < COLS; c++)
    arduboy.drawFastVLine(c * CELL_W, 0, 64, WHITE);
  arduboy.drawFastVLine(127, 0, 64, WHITE);

  // Horizontal lines: header top, header/data divider, 5 data dividers, bottom
  // y = 0, 9, 18, 27, 36, 45, 54, 63
  for (uint8_t r = 0; r <= VISIBLE + 1; r++)
    arduboy.drawFastHLine(0, r * CELL_H, 128, WHITE);

  // Instrument labels in header row, centered ("1"–"8")
  arduboy.setTextColor(WHITE);
  for (uint8_t c = 0; c < COLS; c++) {
    arduboy.setCursor(c * CELL_W + 5, 1);
    arduboy.print((char)('1' + c));
  }

  // Data cells (offset by 1 row to sit below the header)
  for (uint8_t vr = 0; vr < VISIBLE; vr++) {
    uint8_t dr = vr + scrollTop;
    if (dr >= ROWS) break;

    for (uint8_t c = 0; c < COLS; c++) {
      uint8_t cx = c * CELL_W;
      uint8_t cy = (vr + 1) * CELL_H;
      bool isCursor = (c == curCol && dr == curRow);

      if (isCursor) {
        arduboy.fillRect(cx + 1, cy + 1, CELL_W - 2, CELL_H - 2, WHITE);
        arduboy.setTextColor(BLACK);
      } else {
        arduboy.setTextColor(WHITE);
      }

      arduboy.setCursor(cx + 2, cy + 1);
      if (grid[dr][c] == 0xFF) {
        arduboy.print(F("--"));
      } else {
        printHex2(grid[dr][c]);
      }
    }
  }
  arduboy.setTextColor(WHITE);
}

// Clamp-edits the current cell by delta. Empty cells start at 0x00.
// Values stay within 0x00–0xFE (0xFF is reserved for "empty").
void editCell(int16_t delta) {
  uint8_t *v = &grid[curRow][curCol];
  int16_t next = (int16_t)((*v == 0xFF) ? 0x00 : *v) + delta;
  if (next < 0x00) next = 0x00;
  if (next > 0xFE) next = 0xFE;
  *v = (uint8_t)next;
}

void handleInput() {
  bool aIsPressed = arduboy.pressed(A_BUTTON);

  // Reset repeat timers when switching between edit and navigation modes
  if (aIsPressed != aWasPressed) {
    memset(rptTimer, 0, sizeof(rptTimer));
    aWasPressed = aIsPressed;
  }

  if (aIsPressed) {
    // Edit mode: A + D-pad changes the current cell value
    if (checkRepeat(UP_BUTTON,    0)) editCell(+0x10);
    if (checkRepeat(DOWN_BUTTON,  1)) editCell(-0x10);
    if (checkRepeat(LEFT_BUTTON,  2)) editCell(-0x01);
    if (checkRepeat(RIGHT_BUTTON, 3)) editCell(+0x01);
  } else {
    // Navigation mode
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

void loop() {
  if (!arduboy.nextFrame()) return;
  arduboy.pollButtons();
  handleInput();
  arduboy.clear();
  drawGrid();
  arduboy.display();
}
