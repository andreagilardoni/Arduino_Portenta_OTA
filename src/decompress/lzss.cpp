/**************************************************************************************
   INCLUDE
 **************************************************************************************/

#include "lzss.h"

#include <stdlib.h>
#include <stdint.h>

/**************************************************************************************
   DEFINE
 **************************************************************************************/

#define EI 11             /* typically 10..13 */
#define EJ  4             /* typically 4..5 */
#define P   1             /* If match length <= P then output one character */
#define N (1 << EI)       /* buffer size */
#define F ((1 << EJ) + 1) /* lookahead buffer size */

#define LZSS_EOF       (-1)

#define FPUTC_BUF_SIZE (64)
#define FGETC_BUF_SIZE (64)

/**************************************************************************************
   GLOBAL VARIABLES
 **************************************************************************************/

static uint32_t LZSS_FILE_SIZE = 0;
static FILE * update_file = 0;
static FILE * target_file = 0;

int bit_buffer = 0, bit_mask = 128;
unsigned char buffer[N * 2];

static char write_buf[FPUTC_BUF_SIZE];
static size_t write_buf_num_bytes = 0;
static size_t bytes_written_fputc = 0;
static ArduinoPortentaOtaWatchdogResetFuncPointer wdog_feed_func = 0;

/**************************************************************************************
   PUBLIC FUNCTIONS
 **************************************************************************************/

void lzss_init(FILE * update_file_ptr, FILE * target_file_ptr, uint32_t const lzss_file_size, ArduinoPortentaOtaWatchdogResetFuncPointer wdog_feed_func_ptr)
{
  update_file = update_file_ptr;
  target_file = target_file_ptr;
  LZSS_FILE_SIZE = lzss_file_size;
  wdog_feed_func = wdog_feed_func_ptr;
}

void lzss_flush()
{
  bytes_written_fputc += write_buf_num_bytes;

  if (wdog_feed_func)
    wdog_feed_func();

  fwrite(write_buf, 1, write_buf_num_bytes, target_file);

  write_buf_num_bytes = 0;
}

/**************************************************************************************
   PRIVATE FUNCTIONS
 **************************************************************************************/

void lzss_fputc(int const c)
{
  /* Buffer the decompressed data into a buffer so
   * we can perform block writes and don't need to
   * write every byte singly on the flash (which 
   * wouldn't be possible anyway).
   */
  write_buf[write_buf_num_bytes] = static_cast<char>(c);
  write_buf_num_bytes++;

  /* The write buffer is full of decompressed
   * data, write it to the flash now.
   */
  if (write_buf_num_bytes == FPUTC_BUF_SIZE)
    lzss_flush();
}

int lzss_fgetc()
{
  static uint8_t read_buf[FGETC_BUF_SIZE];
  static size_t read_buf_pos = FGETC_BUF_SIZE;
  static size_t bytes_read_fgetc = 0;
  static size_t bytes_read_from_modem = 0;

  /* lzss_file_size is set within SSUBoot:main 
   * and contains the size of the LZSS file. Once
   * all those bytes have been read its time to return
   * LZSS_EOF in order to signal that the end of
   * the file has been reached.
   */
  if (bytes_read_fgetc == LZSS_FILE_SIZE)
    return LZSS_EOF;

  /* If there is no data left to be read from the read buffer
   * than read a new block and store it into the read buffer.
   */
  if (read_buf_pos == FGETC_BUF_SIZE)
  {
    /* Read the next block from the flash memory. */
    bytes_read_from_modem += fread(read_buf, 1, FGETC_BUF_SIZE, update_file);
    /* Reset the read buffer position. */
    read_buf_pos = 0;
  }

  uint8_t const c = read_buf[read_buf_pos];
  read_buf_pos++;
  bytes_read_fgetc++;

  return c;
}

/**************************************************************************************
   LZSS FUNCTIONS
 **************************************************************************************/

int getbit(int n) /* get n bits */
{
    int i, x;
    static int buf, mask = 0;
    
    x = 0;
    for (i = 0; i < n; i++) {
        if (mask == 0) {
            if ((buf = lzss_fgetc()) == LZSS_EOF) return LZSS_EOF;
            mask = 128;
        }
        x <<= 1;
        if (buf & mask) x++;
        mask >>= 1;
    }
    return x;
}

void lzss_decode(void)
{
    int i, j, k, r, c;
    
    for (i = 0; i < N - F; i++) buffer[i] = ' ';
    r = N - F;
    while ((c = getbit(1)) != LZSS_EOF) {
        if (c) {
            if ((c = getbit(8)) == LZSS_EOF) break;
            lzss_fputc(c);
            buffer[r++] = c;  r &= (N - 1);
        } else {
            if ((i = getbit(EI)) == LZSS_EOF) break;
            if ((j = getbit(EJ)) == LZSS_EOF) break;
            for (k = 0; k <= j + 1; k++) {
                c = buffer[(i + k) & (N - 1)];
                lzss_fputc(c);
                buffer[r++] = c;  r &= (N - 1);
            }
        }
    }
}


/**************************************************************************************
   LZSS DECODER CLASS IMPLEMENTATION
 **************************************************************************************/

// get the number of bits the algorithm will try to get given the state
int LZSSDecoder::bits_required(LZSSDecoder::FSM_STATES s) {
    switch(s) {
    case FSM_0:
        return 1;
    case FSM_1:
        return 8;
    case FSM_2:
        return EI;
    case FSM_3:
        return EJ;
    }
}

LZSSDecoder::LZSSDecoder(std::function<int()> getc_cbk, std::function<void(const uint8_t)> putc_cbk)
: put_char_cbk(putc_cbk), get_char_cbk(getc_cbk), state(FSM_0), available(0) {
    for (int i = 0; i < N - F; i++) buffer[i] = ' ';
    r = N - F;
}


LZSSDecoder::LZSSDecoder(std::function<void(const uint8_t)> putc_cbk)
: put_char_cbk(putc_cbk), state(FSM_0), available_bits(0) {
    for (int i = 0; i < N - F; i++) buffer[i] = ' ';
    r = N - F;
}

LZSSDecoder::status LZSSDecoder::handle_state() {
    LZSSDecoder::status res = IN_PROGRESS;

    int c = getbit(bits_required(this->state));

    if(c == LZSS_BUFFER_EMPTY) {
        res = NOT_COMPLETED;
    } else if (c == LZSS_EOF) {
        res = DONE;
        this->state = FSM_EOF;
    } else {
        switch(this->state) {
            case FSM_0:
                if(c) {
                    this->state = FSM_1;
                } else {
                    this->state = FSM_2;
                }
                break;
            case FSM_1:
                putc(c);
                buffer[r++] = c;
                r &= (N - 1); // equivalent to r = r % N

                this->state = FSM_0;
                break;
            case FSM_2:
                this->i = c;
                this->state = FSM_3;
                break;
            case FSM_3: {
                int j = c;
                for (int k = 0; k <= j + 1; k++) { // TODO improve by using memcpy
                    c = buffer[(this->i + k) & (N - 1)]; // equivalent to buffer[(i+k) % N]
                    putc(c);
                    buffer[r++] = c;  r &= (N - 1); // equivalent to r = r % N
                }
                this->state = FSM_0;

                break;
            }
        }
    }

    return res;
}

LZSSDecoder::status LZSSDecoder::decompress(const char* buffer, uint32_t size) {
    this->in_buffer = (uint8_t*)buffer;

    // I won't ever exceed the (2^32)-1 value, since It is impossible to have a buffer of 512MB on an embedded dev
    // thus this value fits a 32 bit uint
    this->available_bits += size*8;
    status res = IN_PROGRESS;

    while((res = handle_state()) == IN_PROGRESS);

    if(res == NOT_COMPLETED && this->available_bits > 0) {
        this->save_excess();
    }

    return res;
}

int LZSSDecoder::getbit(int n) { // get n bits from buffer

    // check that we have all the available bits required
    if(in_buffer == nullptr || n > (available_bits+excess_len)) {
        return LZSS_BUFFER_EMPTY;
    }

    // get the required bits
    static int buf, mask=0;
    int i, x=0;

    // TODO this function couls be improved by extracting bits>1 with a single mask
    for (i = 0; i < n; i++) {
        if (mask == 0) {
            if ((buf = getc()) == LZSS_EOF) return LZSS_EOF;// no need to decrement available_bits
            mask = 128;
        }
        x <<= 1;
        if (buf & mask) x++;
        mask >>= 1;
    }
    available_bits -= n;
    return x;
}

int LZSSDecoder::getc() {
    int c;
    if(excess_len > 0) {
        c = excess_bits;
        excess_len -= 8;
    } else {
        c = *in_buffer;
        in_buffer++;
    }
    return c;
}

void LZSSDecoder::save_excess() { // TODO Explain that
    if(this->available_bits >= 8) {
        this->excess_bits = *this->in_buffer;
        this->excess_len = 8;
    }
}
