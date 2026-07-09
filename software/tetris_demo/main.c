/* Tetris for the icepi_zero LCD engine.
 *
 * Pure firmware; no LVGL, no framebuffer. The board is a grid of colored
 * blocks and every block is one hardware lcd_fill_rect() op, so the CPU
 * only ever touches the ~handful of cells that changed between frames.
 * Input is the FT6336U capacitive panel: four on-screen touch zones along
 * the bottom (LEFT / ROTATE / RIGHT / DROP).
 *
 * Target gateware: icepi_zero_lcd.py (LCD engine + ctp_i2c touch, no SNN).
 */

#include <stdint.h>
#include <stdbool.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/soc.h>

#include "lcd.h"
#include "touch.h"
#include "log.h"

/* ------------------------------------------------------------------ *
 *  Layout (panel is 320 wide x 480 tall, portrait)
 * ------------------------------------------------------------------ */
#define COLS        10
#define ROWS        20
#define CELL        20                     /* px per board cell        */
#define BOARD_X     8
#define BOARD_Y     8
#define BOARD_W     (COLS * CELL)          /* 200 */
#define BOARD_H     (ROWS * CELL)          /* 400 */

#define PANEL_X     (BOARD_X + BOARD_W + 8)/* 216: right info column   */

/* Bottom touch-button strip. */
#define BTN_Y0      416
#define BTN_H       60
#define BTN_W       80                     /* four 80px buttons        */
enum { BTN_NONE = -1, BTN_LEFT = 0, BTN_ROTATE, BTN_RIGHT, BTN_DROP, BTN_PAUSE };

/* Pause button lives in the right info column, below the stats. */
#define PAUSE_X     PANEL_X
#define PAUSE_Y     270
#define PAUSE_W     96
#define PAUSE_H     54

/* ------------------------------------------------------------------ *
 *  Colors (RGB565; lcd_fill_rect sends these MSB-first for us)
 * ------------------------------------------------------------------ */
#define C_BLACK   0x0000
#define C_BG      0x0000
#define C_GRID    0x2104   /* dark grey cell separators */
#define C_FRAME   0x8410   /* board border / chrome     */
#define C_BTN     0x4208   /* button background         */
#define C_BTN_HI  0x9CD3   /* pressed button            */
#define C_ICON    0xFFFF
#define C_GHOST   0x39E7   /* game-over dimmed block    */

/* Per-piece colors, indexed by color id 1..7 (0 = empty). */
static const uint16_t PIECE_COLOR[8] = {
	C_BG,     /* 0 empty          */
	0x07FF,   /* 1 I  cyan        */
	0xFFE0,   /* 2 O  yellow      */
	0xF81F,   /* 3 T  magenta     */
	0x07E0,   /* 4 S  green       */
	0xF800,   /* 5 Z  red         */
	0x001F,   /* 6 J  blue        */
	0xFD20,   /* 7 L  orange      */
};

/* ------------------------------------------------------------------ *
 *  Tetromino table: 7 pieces x 4 rotations x 4 blocks, each block a
 *  (x,y) coord inside a 4x4 box (x right, y down). color = id into
 *  PIECE_COLOR above.
 * ------------------------------------------------------------------ */
typedef struct { int8_t x, y; } cell_t;
typedef struct { cell_t b[4][4]; uint8_t color; } piece_t;

static const piece_t PIECES[7] = {
	/* I */ {{
		{{0,1},{1,1},{2,1},{3,1}}, {{2,0},{2,1},{2,2},{2,3}},
		{{0,2},{1,2},{2,2},{3,2}}, {{1,0},{1,1},{1,2},{1,3}},
	}, 1},
	/* O */ {{
		{{1,0},{2,0},{1,1},{2,1}}, {{1,0},{2,0},{1,1},{2,1}},
		{{1,0},{2,0},{1,1},{2,1}}, {{1,0},{2,0},{1,1},{2,1}},
	}, 2},
	/* T */ {{
		{{1,0},{0,1},{1,1},{2,1}}, {{1,0},{1,1},{2,1},{1,2}},
		{{0,1},{1,1},{2,1},{1,2}}, {{1,0},{0,1},{1,1},{1,2}},
	}, 3},
	/* S */ {{
		{{1,0},{2,0},{0,1},{1,1}}, {{1,0},{1,1},{2,1},{2,2}},
		{{1,1},{2,1},{0,2},{1,2}}, {{0,0},{0,1},{1,1},{1,2}},
	}, 4},
	/* Z */ {{
		{{0,0},{1,0},{1,1},{2,1}}, {{2,0},{1,1},{2,1},{1,2}},
		{{0,1},{1,1},{1,2},{2,2}}, {{1,0},{0,1},{1,1},{0,2}},
	}, 5},
	/* J */ {{
		{{0,0},{0,1},{1,1},{2,1}}, {{1,0},{2,0},{1,1},{1,2}},
		{{0,1},{1,1},{2,1},{2,2}}, {{1,0},{1,1},{0,2},{1,2}},
	}, 6},
	/* L */ {{
		{{2,0},{0,1},{1,1},{2,1}}, {{1,0},{1,1},{1,2},{2,2}},
		{{0,1},{1,1},{2,1},{0,2}}, {{0,0},{1,0},{1,1},{1,2}},
	}, 7},
};

/* ------------------------------------------------------------------ *
 *  Game state
 * ------------------------------------------------------------------ */
static uint8_t board[ROWS][COLS];   /* locked cells, color id           */
static uint8_t shadow[ROWS][COLS];  /* color id currently on screen      */

static int  cur_type, cur_rot, cur_x, cur_y;   /* active piece          */
static int  next_type;
static uint32_t score, lines_total;
static int  level;
static bool game_over;
static bool paused;

/* 7-bag randomizer */
static uint8_t bag[7];
static int     bag_pos = 7;
static uint32_t rng_state = 0x1234abcdu;

static uint32_t rng_next(void)
{
	rng_state ^= rng_state << 13;
	rng_state ^= rng_state >> 17;
	rng_state ^= rng_state << 5;
	return rng_state;
}

static int bag_draw(void)
{
	if(bag_pos >= 7) {
		for(int i = 0; i < 7; i++) bag[i] = i;
		for(int i = 6; i > 0; i--) {      /* Fisher-Yates */
			int j = rng_next() % (i + 1);
			uint8_t t = bag[i]; bag[i] = bag[j]; bag[j] = t;
		}
		bag_pos = 0;
	}
	return bag[bag_pos++];
}

/* ------------------------------------------------------------------ *
 *  Low-level drawing helpers (all lcd_fill_rect)
 * ------------------------------------------------------------------ */
static inline void fill(int x, int y, int w, int h, uint16_t c)
{
	if(w > 0 && h > 0)
		lcd_fill_rect(x, y, w, h, c);
}

/* Draw a single board cell's interior (1px grid gap on top/left). */
static void draw_cell(int r, int c, uint16_t color)
{
	fill(BOARD_X + c * CELL + 1, BOARD_Y + r * CELL + 1,
	     CELL - 1, CELL - 1, color);
}

/* Filled triangle, one row of fills. dir: 0=left 1=right 2=down. */
static void draw_tri(int cx, int cy, int len, int dir, uint16_t color)
{
	for(int d = -len; d <= len; d++) {
		int ad = d < 0 ? -d : d;
		if(dir == 0)        /* apex left  */
			fill(cx - len + ad, cy + d, (len - ad) + 1, 1, color);
		else if(dir == 1)   /* apex right */
			fill(cx - len,       cy + d, (len - ad) + 1, 1, color);
	}
	if(dir == 2) {          /* apex down  */
		for(int k = 0; k <= len; k++)
			fill(cx - (len - k), cy - len + k, 2 * (len - k) + 1, 1, color);
	}
}

/* Open square ring, reads as the rotate/cycle icon. */
static void draw_ring(int cx, int cy, int r, uint16_t color)
{
	int t = 3;
	fill(cx - r, cy - r, 2 * r, t, color);          /* top    */
	fill(cx - r, cy + r - t, 2 * r, t, color);      /* bottom */
	fill(cx - r, cy - r, t, 2 * r, color);          /* left   */
	fill(cx + r - t, cy - r, t, r, color);          /* right, upper half only */
}

/* ------------------------------------------------------------------ *
 *  7-segment number rendering (score / level / lines)
 * ------------------------------------------------------------------ */
#define DW 10   /* digit width  */
#define DH 18   /* digit height */
#define DT 2    /* segment thickness */
#define DGAP 3

static const uint8_t SEG[10] = {
	0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

static void draw_digit(int x, int y, int v, uint16_t color, uint16_t bg)
{
	fill(x, y, DW, DH, bg);
	if(v < 0) return;                 /* blank */
	uint8_t s = SEG[v];
	int mid = y + DH / 2 - DT / 2;
	if(s & 0x01) fill(x + DT, y, DW - 2 * DT, DT, color);              /* a top    */
	if(s & 0x02) fill(x + DW - DT, y + DT, DT, DH / 2 - DT, color);    /* b t-right */
	if(s & 0x04) fill(x + DW - DT, mid, DT, DH / 2 - DT, color);       /* c b-right */
	if(s & 0x08) fill(x + DT, y + DH - DT, DW - 2 * DT, DT, color);    /* d bottom  */
	if(s & 0x10) fill(x, mid, DT, DH / 2 - DT, color);                 /* e b-left  */
	if(s & 0x20) fill(x, y + DT, DT, DH / 2 - DT, color);              /* f t-left  */
	if(s & 0x40) fill(x + DT, mid, DW - 2 * DT, DT, color);            /* g middle  */
}

/* Right-aligned unsigned number, `ndig` fixed digits. */
static void draw_number(int x_right, int y, uint32_t v, int ndig, uint16_t color)
{
	for(int i = 0; i < ndig; i++) {
		int digit = (i == 0 || v > 0) ? (int)(v % 10) : -1;  /* leading blank */
		draw_digit(x_right - (i + 1) * (DW + DGAP) + DGAP, y,
		           digit, color, C_BG);
		v /= 10;
	}
}

/* ------------------------------------------------------------------ *
 *  Static chrome, drawn once (and on restart)
 * ------------------------------------------------------------------ */
static void draw_pause_btn(void);

static void draw_static(void)
{
	fill(0, 0, LCD_WIDTH, LCD_HEIGHT, C_BLACK);

	/* Board frame + grid: fill grid color, punch empty interiors. */
	fill(BOARD_X - 2, BOARD_Y - 2, BOARD_W + 4, BOARD_H + 4, C_FRAME);
	fill(BOARD_X, BOARD_Y, BOARD_W, BOARD_H, C_GRID);
	for(int r = 0; r < ROWS; r++)
		for(int c = 0; c < COLS; c++)
			draw_cell(r, c, C_BG);

	/* Buttons. */
	for(int i = 0; i < 4; i++) {
		int bx = i * BTN_W;
		fill(bx + 2, BTN_Y0 + 2, BTN_W - 4, BTN_H - 4, C_BTN);
		int cx = bx + BTN_W / 2, cy = BTN_Y0 + BTN_H / 2;
		if(i == BTN_LEFT)        draw_tri(cx, cy, 14, 0, C_ICON);
		else if(i == BTN_ROTATE) draw_ring(cx, cy, 13, C_ICON);
		else if(i == BTN_RIGHT)  draw_tri(cx, cy, 14, 1, C_ICON);
		else                     draw_tri(cx, cy, 14, 2, C_ICON);
	}

	draw_pause_btn();
}

/* Pause button. Icon reflects state: two bars while running (tap to
 * pause), a play triangle while paused (tap to resume). */
static void draw_pause_btn(void)
{
	int cx = PAUSE_X + PAUSE_W / 2, cy = PAUSE_Y + PAUSE_H / 2;
	fill(PAUSE_X, PAUSE_Y, PAUSE_W, PAUSE_H, paused ? C_BTN_HI : C_BTN);
	if(paused) {
		draw_tri(cx, cy, 16, 1, C_ICON);          /* resume: play */
	} else {
		fill(cx - 10, cy - 14, 7, 28, C_ICON);    /* pause: || bars */
		fill(cx + 3,  cy - 14, 7, 28, C_ICON);
	}
}

/* Boot / press-to-start splash: clears the panel, shows a banner of all
 * seven tetrominoes and a big play triangle. */
static void draw_splash(void)
{
	fill(0, 0, LCD_WIDTH, LCD_HEIGHT, C_BLACK);
	const int mc = 8, y0 = 40, step = 5 * mc;
	for(int t = 0; t < 7; t++) {
		const piece_t *p = &PIECES[t];
		uint16_t col = PIECE_COLOR[p->color];
		int bx = 10 + t * step;
		for(int i = 0; i < 4; i++)
			fill(bx + p->b[0][i].x * mc + 1, y0 + p->b[0][i].y * mc + 1,
			     mc - 1, mc - 1, col);
	}
	draw_tri(LCD_WIDTH / 2, LCD_HEIGHT / 2, 44, 1, C_ICON);
}

/* NEXT-piece preview box in the right column. */
static int prev_next = -1;
static void draw_next(void)
{
	if(next_type == prev_next) return;
	prev_next = next_type;
	const int bx = PANEL_X, by = 40, mc = 14;  /* mini-cell 14px */
	fill(bx, by, 4 * mc, 4 * mc, C_BG);
	const piece_t *p = &PIECES[next_type];
	uint16_t col = PIECE_COLOR[p->color];
	for(int i = 0; i < 4; i++) {
		int gx = p->b[0][i].x, gy = p->b[0][i].y;
		fill(bx + gx * mc + 1, by + gy * mc + 1, mc - 1, mc - 1, col);
	}
}

/* score / level / lines — redraw only when values change */
static uint32_t shown_score = 0xffffffffu, shown_lines = 0xffffffffu;
static int      shown_level = -1;
static void draw_stats(void)
{
	int rx = LCD_WIDTH - 6;            /* right margin */
	if(score != shown_score) {
		draw_number(rx, 120, score, 6, 0x07FF);
		shown_score = score;
	}
	if((uint32_t)level != (uint32_t)shown_level) {
		draw_number(rx, 160, level, 2, 0xFFE0);
		shown_level = level;
	}
	if(lines_total != shown_lines) {
		draw_number(rx, 200, lines_total, 4, 0x07E0);
		shown_lines = lines_total;
	}
}

/* ------------------------------------------------------------------ *
 *  Board diff render: reconcile shadow[] with board[]+active piece
 * ------------------------------------------------------------------ */
static void render_board(void)
{
	static uint8_t disp[ROWS][COLS];
	for(int r = 0; r < ROWS; r++)
		for(int c = 0; c < COLS; c++)
			disp[r][c] = board[r][c];

	if(!game_over) {
		const piece_t *p = &PIECES[cur_type];
		for(int i = 0; i < 4; i++) {
			int gx = cur_x + p->b[cur_rot][i].x;
			int gy = cur_y + p->b[cur_rot][i].y;
			if(gx >= 0 && gx < COLS && gy >= 0 && gy < ROWS)
				disp[gy][gx] = p->color;
		}
	}

	for(int r = 0; r < ROWS; r++)
		for(int c = 0; c < COLS; c++)
			if(disp[r][c] != shadow[r][c]) {
				draw_cell(r, c, PIECE_COLOR[disp[r][c]]);
				shadow[r][c] = disp[r][c];
			}
}

/* ------------------------------------------------------------------ *
 *  Game rules
 * ------------------------------------------------------------------ */
static bool collides(int type, int rot, int px, int py)
{
	const piece_t *p = &PIECES[type];
	for(int i = 0; i < 4; i++) {
		int gx = px + p->b[rot][i].x;
		int gy = py + p->b[rot][i].y;
		if(gx < 0 || gx >= COLS || gy >= ROWS) return true;
		if(gy >= 0 && board[gy][gx])           return true;
	}
	return false;
}

static void lock_piece(void)
{
	const piece_t *p = &PIECES[cur_type];
	for(int i = 0; i < 4; i++) {
		int gx = cur_x + p->b[cur_rot][i].x;
		int gy = cur_y + p->b[cur_rot][i].y;
		if(gy >= 0 && gy < ROWS && gx >= 0 && gx < COLS)
			board[gy][gx] = p->color;
	}
}

static int clear_lines(void)
{
	int cleared = 0;
	for(int r = ROWS - 1; r >= 0; r--) {
		bool full = true;
		for(int c = 0; c < COLS; c++)
			if(!board[r][c]) { full = false; break; }
		if(full) {
			for(int rr = r; rr > 0; rr--)
				for(int c = 0; c < COLS; c++)
					board[rr][c] = board[rr - 1][c];
			for(int c = 0; c < COLS; c++) board[0][c] = 0;
			cleared++;
			r++;   /* recheck this row after the shift */
		}
	}
	return cleared;
}

static void spawn(void)
{
	cur_type = next_type;
	next_type = bag_draw();
	cur_rot = 0;
	cur_x = 3;
	cur_y = 0;
	if(collides(cur_type, cur_rot, cur_x, cur_y))
		game_over = true;
}

static void apply_clear_score(int n)
{
	static const uint32_t pts[5] = {0, 100, 300, 500, 800};
	if(n > 0) {
		score += pts[n] * (uint32_t)level;
		lines_total += n;
		level = 1 + lines_total / 10;
	}
}

static void step_gravity(void)
{
	if(!collides(cur_type, cur_rot, cur_x, cur_y + 1)) {
		cur_y++;
	} else {
		lock_piece();
		apply_clear_score(clear_lines());
		spawn();
	}
}

static void hard_drop(void)
{
	while(!collides(cur_type, cur_rot, cur_x, cur_y + 1))
		cur_y++;
	lock_piece();
	apply_clear_score(clear_lines());
	spawn();
}

static void do_action(int btn)
{
	switch(btn) {
	case BTN_LEFT:
		if(!collides(cur_type, cur_rot, cur_x - 1, cur_y)) cur_x--;
		break;
	case BTN_RIGHT:
		if(!collides(cur_type, cur_rot, cur_x + 1, cur_y)) cur_x++;
		break;
	case BTN_ROTATE: {
		int nr = (cur_rot + 1) & 3;
		if(!collides(cur_type, nr, cur_x, cur_y)) cur_rot = nr;
		break;
	}
	case BTN_DROP:
		hard_drop();
		break;
	}
}

/* ------------------------------------------------------------------ *
 *  Touch input -> button id
 * ------------------------------------------------------------------ */
static bool touched(void)
{
	uint8_t buf[7];
	if(!ft6336u_read(0x02, buf, sizeof(buf))) return false;
	return (buf[0] & 0x0f) != 0;
}

static int read_button(void)
{
	uint8_t buf[7];
	if(!ft6336u_read(0x02, buf, sizeof(buf))) return BTN_NONE;
	if((buf[0] & 0x0f) == 0)                   return BTN_NONE;
	unsigned x = ((buf[1] & 0x0f) << 8) | buf[2];
	unsigned y = ((buf[3] & 0x0f) << 8) | buf[4];
	if(x >= PAUSE_X && x < PAUSE_X + PAUSE_W &&
	   y >= PAUSE_Y && y < PAUSE_Y + PAUSE_H)
		return BTN_PAUSE;
	if(y < BTN_Y0) return BTN_NONE;            /* only the button strip */
	int b = x / BTN_W;
	if(b > BTN_DROP) b = BTN_DROP;
	return b;
}

/* ------------------------------------------------------------------ *
 *  Reset + game-over presentation
 * ------------------------------------------------------------------ */
static void reset_game(void)
{
	for(int r = 0; r < ROWS; r++)
		for(int c = 0; c < COLS; c++) { board[r][c] = 0; shadow[r][c] = 0; }
	score = 0; lines_total = 0; level = 1; game_over = false; paused = false;
	bag_pos = 7;
	prev_next = -1;
	shown_score = shown_lines = 0xffffffffu; shown_level = -1;
	next_type = bag_draw();
	spawn();
	draw_static();
	draw_next();
	draw_stats();
	render_board();
}

static void show_game_over(void)
{
	/* Dim every locked block to grey, leave the layout intact. */
	for(int r = 0; r < ROWS; r++)
		for(int c = 0; c < COLS; c++)
			if(board[r][c]) draw_cell(r, c, C_GHOST);
	log_puts("GAME OVER  score="); log_uint(score);
	log_puts(" lines="); log_uint(lines_total); log_nl();

	/* Wait for release, then a fresh tap anywhere to restart. */
	while(touched()) busy_wait(20);
	while(!touched()) busy_wait(20);
	while(touched()) busy_wait(20);
}

/* ------------------------------------------------------------------ *
 *  main
 * ------------------------------------------------------------------ */
#define FRAME_MS       16
#define REPEAT_DELAY   14     /* frames before L/R auto-repeat kicks in */
#define REPEAT_RATE    3      /* frames between repeats                 */

static int gravity_frames(void)
{
	int g = 48 - (level - 1) * 4;
	return g < 5 ? 5 : g;
}

int main(void)
{
	uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setie(1);
#endif
	log_puts("Tetris on the LCD engine"); log_nl();

	lcd_pads_apply();
	lcd_init();
	busy_wait(200);              /* FT6336U wake-up after shared reset */
	touch_init();
	ft6336u_probe();

	/* Seed the RNG from how long the player takes to first touch. */
	log_puts("touch anywhere to start"); log_nl();
	draw_splash();
	while(!touched())
		rng_state += 0x9e3779b9u;   /* churn the seed while we wait */
	while(touched()) busy_wait(20);   /* wait release */

	reset_game();

	int last_btn = BTN_NONE, press_frames = 0, grav_ctr = 0;

	for(;;) {
		int btn = read_button();

		if(btn != BTN_NONE) {
			if(btn != last_btn) {           /* fresh press */
				if(btn == BTN_PAUSE) {
					paused = !paused;
					draw_pause_btn();
				} else if(!paused) {
					do_action(btn);
				}
				press_frames = 0;
			} else if(!paused) {            /* held */
				press_frames++;
				if((btn == BTN_LEFT || btn == BTN_RIGHT) &&
				   press_frames >= REPEAT_DELAY &&
				   ((press_frames - REPEAT_DELAY) % REPEAT_RATE) == 0)
					do_action(btn);
			}
		}
		last_btn = btn;

		if(!paused && ++grav_ctr >= gravity_frames()) {
			grav_ctr = 0;
			step_gravity();
		}

		draw_next();
		draw_stats();
		render_board();

		if(game_over) {
			show_game_over();
			reset_game();
			last_btn = BTN_NONE; grav_ctr = 0;
		}

		busy_wait(FRAME_MS);
	}
	return 0;
}
