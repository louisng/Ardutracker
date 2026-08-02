#include <Arduboy2.h>

Arduboy2 arduboy;

static const uint8_t COLS    = 8;
static const uint8_t ROWS    = 40;
static const uint8_t VISIBLE = 7;   // rows visible at once
static const uint8_t CELL_W  = 16;  // pixels per column (128 / 8)
static const uint8_t CELL_H  = 9;   // pixels per row (border + 8px font)

// Grid data: 0xFF = empty cell, 0x00–0xFE = value
uint8_t grid[ROWS][COLS];

uint8_t curCol    = 0;
uint8_t curRow    = 0;
uint8_t scrollTop = 0;  // index of the first visible row

// Key-repeat state: UP=0, DOWN=1, LEFT=2, RIGHT=3
uint8_t rptTimer[4];
static const uint8_t RPT_START = 20;  // frames before first repeat
static const uint8_t RPT_RATE  = 5;   // frames between repeats

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
  // Vertical borders: 8 column-left edges + right screen edge
  for (uint8_t c = 0; c < COLS; c++)
    arduboy.drawFastVLine(c * CELL_W, 0, 64, WHITE);
  arduboy.drawFastVLine(127, 0, 64, WHITE);

  // Horizontal borders: 7 visible rows need 8 lines (top + 6 dividers + bottom)
  for (uint8_t r = 0; r <= VISIBLE; r++)
    arduboy.drawFastHLine(0, r * CELL_H, 128, WHITE);

  // Cell contents
  for (uint8_t vr = 0; vr < VISIBLE; vr++) {
    uint8_t dr = vr + scrollTop;
    if (dr >= ROWS) break;

    for (uint8_t c = 0; c < COLS; c++) {
      uint8_t cx = c * CELL_W;
      uint8_t cy = vr * CELL_H;
      bool isCursor = (c == curCol && dr == curRow);

      if (isCursor) {
        // Fill interior white, draw text in black (inverted cursor)
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

void handleInput() {
  if (checkRepeat(UP_BUTTON, 0) && curRow > 0) {
    curRow--;
    if (curRow < scrollTop) scrollTop = curRow;
  }
  if (checkRepeat(DOWN_BUTTON, 1) && curRow < ROWS - 1) {
    curRow++;
    if (curRow >= scrollTop + VISIBLE) scrollTop = curRow - VISIBLE + 1;
  }
  if (checkRepeat(LEFT_BUTTON, 2) && curCol > 0)    curCol--;
  if (checkRepeat(RIGHT_BUTTON, 3) && curCol < COLS - 1) curCol++;
}

void loop() {
  if (!arduboy.nextFrame()) return;
  arduboy.pollButtons();
  handleInput();
  arduboy.clear();
  drawGrid();
  arduboy.display();
}
