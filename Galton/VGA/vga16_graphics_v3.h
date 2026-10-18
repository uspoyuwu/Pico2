/**
 * Hunter Adams (vha3@cornell.edu)
 * modifed for 16 colors by BRL4
 * 
 *rp2350 ONLY -- too much memory for rp2040

 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green 
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - 4 DMA channels
 *  - 2 x 38.4 kBytes of RAM (for doublebuffer pixel data, 1 bit per pixel)
 *
 * ONE BIT PER PIXEL, MONOCHROME.
 *
 * Eight pixels to a byte rather than two, so each buffer is 38,400 bytes
 * instead of 153,600 and the pair costs 75 KB instead of 300 KB. One bit
 * cannot drive four colour pins, so rgb.pio narrows its OUT group to a
 * single pin and the picture is that colour on black; see rgb.pio for
 * which pin and how to change it.
 *
 * Any non-zero colour lights a pixel and zero clears it, so existing calls
 * still compile and still mean the right thing. The enum below is kept so
 * that old code builds, but only BLACK is distinguishable from the rest.
 *
 * Two call-level changes:
 *   clearRect  x1 and x2 must now be multiples of EIGHT, not two.
 *   readPixel  returns 0 or 1, not a colour index.
 */


// Give the I/O pins that we're using some names that make sense - usable in main()
 enum vga_pins {HSYNC=16, VSYNC, LO_GRN, HI_GRN, BLUE_PIN, RED_PIN} ;

// Kept so that code written against the 16 colour driver still compiles.
// On screen only BLACK and not-BLACK can be told apart.
enum colors {BLACK, DARK_GREEN, MED_GREEN, GREEN,
            DARK_BLUE, BLUE, LIGHT_BLUE, CYAN,
            RED, DARK_ORANGE, ORANGE, YELLOW,
            MAGENTA, PINK, LIGHT_PINK, WHITE} ;

// Augmentations
void drawCell(short x, short y, char color) ;
int checkNeighbors(short x, short y) ;
int isAlive(short x, short y) ;


// VGA init -- Do this before any other libraries
void initVGA(void) ;

// ========================
// sync signals from DMA channel to thread
// signal to thread to draw
int draw_start_signal(void);
// returns 1 for 60 fps, 2 for 30 fps, 3 for no buffer
int get_buffer_type(void) ;

// =========================
// shapes and fills
void drawPixel(short x, short y, char color) ;
void drawVLine(short x, short y, short h, char color) ;
void drawHLine(int x, int y, int w, char color) ; // faster mod 5/11/2025
void drawLine(short x0, short y0, short x1, short y1, char color) ;
void drawRect(short x, short y, short w, short h, char color);
void drawCircle(short x0, short y0, short r, char color) ;
void drawCircleHelper( short x0, short y0, short r, unsigned char cornername, char color) ;
void fillCircle(short x0, short y0, short r, char color) ;
void fillCircleHelper(short x0, short y0, short r, unsigned char cornername, short delta, char color) ;
void drawRoundRect(short x, short y, short w, short h, short r, char color) ;
void fillRoundRect(short x, short y, short w, short h, short r, char color) ;
void fillRect(short x, short y, short w, short h, char color) ;
void fillTri(float x0, float y0, float x1, float y1, float x2, float y2, char color) ;
void drawMultiLine(int num_lines,  short point_list[][2], char color) ;
// ===================
// USE THESE functions for text!
// All text starts at even x value -- a odd x is shifted left one pixel
int drawTextGLCD(short x, short y, char * string, char color, char bakgnd_color);
int drawTextAscii(short x, short y, char * str, char color, char bgcolor);
int drawTextVGA437(short x, short y, char * string, char color, char bakgnd_color);
int drawTextArial24(short x, short y, char * str, char color, char bgcolor);
int drawTextTiny8(short x, short y, char * str, char color, char bgcolor) ;
int drawTextGrotesk32(short x, short y, char * str, char color, char bgcolor) ;
// ====================
//
// ====================
// specialized clear routines
// fast clear -- x1 and x2 must be MULTIPLES OF EIGHT at 1 bit per pixel
void clearRect(short x1, short y1, short x2, short y2, short c) ;
// clears the whole frame below top value to a color
void clearLowFrame(short, short) ;
// clear a range of y values
void clearRegion(short y1, short y2, short c) ;

// ====================
// buffer handling
// Static text needs to be duplicated into both buffers
// copy buffer 0 to 1 and 1 to 0
void copy_buffer0to1(void) ;
void copy_buffer1to0(void) ;
// updates whichever buffer you did not just draw in
// copies current draw-buffer to the other one
void copy_buffer_to_other(void) ;

// ====================
// two seldom used functions
// === get the color of apixel from the frame buffer
short readPixel(short, short) ;

// draw a little cross
void crosshair(short x, short y, short c) ;

// ==================================================
// !!!!!!!!!!!! dont use for new code !!!!!!!!!!!!!!!! 
// depricated characters and strings
// see above for recommended routines
void drawChar(short x, short y, unsigned char c, char color, char bg, unsigned char size) ;
void setCursor(short x, short y);
void setTextColor(char c);
void setTextColor2(char c, char bg);
void setTextSize(unsigned char s);
void setTextWrap(char w);
void tft_write(unsigned char c) ;
void writeString(char* str) ;
// ===  strings added 10/11/2023 brl4
void drawCharBig(short x, short y, unsigned char c, char color, char bg) ;
void writeStringBig(char* str) ;
void setTextColorBig(char, char); //works, but can use usual setTextColor2
// 5x7 font
void writeStringBold(char* str);
void drawBoldTextGLCD(short x, short y, char * str, char textcolor, char textbgcolor, char size);
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!