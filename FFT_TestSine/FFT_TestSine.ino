/*
  fft_test_sine
  example sketch for testing the fft code.
  This generates a simple sine wave data set consisting
  of two summed signals with frequencies f1 and f2, transforms it, calculates
  and prints the amplitude of the transform.
*/

#include <avr/pgmspace.h>
#define N_WAVE      1024    /* full length of Sinewave[] */
#define LOG2_N_WAVE 10      /* log2(N_WAVE) */

/*
  Henceforth "int16_t" implies 16-bit word. If this is not
  the case in your architecture, please replace "int16_t"
  with a type definition which *is* a 16-bit word.
*/

/*
  Since we only use 3/4 of N_WAVE, we define only
  this many samples, in order to conserve data space.
*/
const int16_t PROGMEM Sinewave[N_WAVE - N_WAVE / 4] = {
  0,    201,    402,    603,    804,   1005,   1206,   1406,
  1607,   1808,   2009,   2209,   2410,   2610,   2811,   3011,
  3211,   3411,   3611,   3811,   4011,   4210,   4409,   4608,
  4807,   5006,   5205,   5403,   5601,   5799,   5997,   6195,
  6392,   6589,   6786,   6982,   7179,   7375,   7571,   7766,
  7961,   8156,   8351,   8545,   8739,   8932,   9126,   9319,
  9511,   9703,   9895,  10087,  10278,  10469,  10659,  10849,
  11038,  11227,  11416,  11604,  11792,  11980,  12166,  12353,
  12539,  12724,  12909,  13094,  13278,  13462,  13645,  13827,
  14009,  14191,  14372,  14552,  14732,  14911,  15090,  15268,
  15446,  15623,  15799,  15975,  16150,  16325,  16499,  16672,
  16845,  17017,  17189,  17360,  17530,  17699,  17868,  18036,
  18204,  18371,  18537,  18702,  18867,  19031,  19194,  19357,
  19519,  19680,  19840,  20000,  20159,  20317,  20474,  20631,
  20787,  20942,  21096,  21249,  21402,  21554,  21705,  21855,
  22004,  22153,  22301,  22448,  22594,  22739,  22883,  23027,
  23169,  23311,  23452,  23592,  23731,  23869,  24006,  24143,
  24278,  24413,  24546,  24679,  24811,  24942,  25072,  25201,
  25329,  25456,  25582,  25707,  25831,  25954,  26077,  26198,
  26318,  26437,  26556,  26673,  26789,  26905,  27019,  27132,
  27244,  27355,  27466,  27575,  27683,  27790,  27896,  28001,
  28105,  28208,  28309,  28410,  28510,  28608,  28706,  28802,
  28897,  28992,  29085,  29177,  29268,  29358,  29446,  29534,
  29621,  29706,  29790,  29873,  29955,  30036,  30116,  30195,
  30272,  30349,  30424,  30498,  30571,  30643,  30713,  30783,
  30851,  30918,  30984,  31049,  31113,  31175,  31236,  31297,
  31356,  31413,  31470,  31525,  31580,  31633,  31684,  31735,
  31785,  31833,  31880,  31926,  31970,  32014,  32056,  32097,
  32137,  32176,  32213,  32249,  32284,  32318,  32350,  32382,
  32412,  32441,  32468,  32495,  32520,  32544,  32567,  32588,
  32609,  32628,  32646,  32662,  32678,  32692,  32705,  32717,
  32727,  32736,  32744,  32751,  32757,  32761,  32764,  32766,
  32767,  32766,  32764,  32761,  32757,  32751,  32744,  32736,
  32727,  32717,  32705,  32692,  32678,  32662,  32646,  32628,
  32609,  32588,  32567,  32544,  32520,  32495,  32468,  32441,
  32412,  32382,  32350,  32318,  32284,  32249,  32213,  32176,
  32137,  32097,  32056,  32014,  31970,  31926,  31880,  31833,
  31785,  31735,  31684,  31633,  31580,  31525,  31470,  31413,
  31356,  31297,  31236,  31175,  31113,  31049,  30984,  30918,
  30851,  30783,  30713,  30643,  30571,  30498,  30424,  30349,
  30272,  30195,  30116,  30036,  29955,  29873,  29790,  29706,
  29621,  29534,  29446,  29358,  29268,  29177,  29085,  28992,
  28897,  28802,  28706,  28608,  28510,  28410,  28309,  28208,
  28105,  28001,  27896,  27790,  27683,  27575,  27466,  27355,
  27244,  27132,  27019,  26905,  26789,  26673,  26556,  26437,
  26318,  26198,  26077,  25954,  25831,  25707,  25582,  25456,
  25329,  25201,  25072,  24942,  24811,  24679,  24546,  24413,
  24278,  24143,  24006,  23869,  23731,  23592,  23452,  23311,
  23169,  23027,  22883,  22739,  22594,  22448,  22301,  22153,
  22004,  21855,  21705,  21554,  21402,  21249,  21096,  20942,
  20787,  20631,  20474,  20317,  20159,  20000,  19840,  19680,
  19519,  19357,  19194,  19031,  18867,  18702,  18537,  18371,
  18204,  18036,  17868,  17699,  17530,  17360,  17189,  17017,
  16845,  16672,  16499,  16325,  16150,  15975,  15799,  15623,
  15446,  15268,  15090,  14911,  14732,  14552,  14372,  14191,
  14009,  13827,  13645,  13462,  13278,  13094,  12909,  12724,
  12539,  12353,  12166,  11980,  11792,  11604,  11416,  11227,
  11038,  10849,  10659,  10469,  10278,  10087,   9895,   9703,
  9511,   9319,   9126,   8932,   8739,   8545,   8351,   8156,
  7961,   7766,   7571,   7375,   7179,   6982,   6786,   6589,
  6392,   6195,   5997,   5799,   5601,   5403,   5205,   5006,
  4807,   4608,   4409,   4210,   4011,   3811,   3611,   3411,
  3211,   3011,   2811,   2610,   2410,   2209,   2009,   1808,
  1607,   1406,   1206,   1005,    804,    603,    402,    201,
  0,   -201,   -402,   -603,   -804,  -1005,  -1206,  -1406,
  -1607,  -1808,  -2009,  -2209,  -2410,  -2610,  -2811,  -3011,
  -3211,  -3411,  -3611,  -3811,  -4011,  -4210,  -4409,  -4608,
  -4807,  -5006,  -5205,  -5403,  -5601,  -5799,  -5997,  -6195,
  -6392,  -6589,  -6786,  -6982,  -7179,  -7375,  -7571,  -7766,
  -7961,  -8156,  -8351,  -8545,  -8739,  -8932,  -9126,  -9319,
  -9511,  -9703,  -9895, -10087, -10278, -10469, -10659, -10849,
  -11038, -11227, -11416, -11604, -11792, -11980, -12166, -12353,
  -12539, -12724, -12909, -13094, -13278, -13462, -13645, -13827,
  -14009, -14191, -14372, -14552, -14732, -14911, -15090, -15268,
  -15446, -15623, -15799, -15975, -16150, -16325, -16499, -16672,
  -16845, -17017, -17189, -17360, -17530, -17699, -17868, -18036,
  -18204, -18371, -18537, -18702, -18867, -19031, -19194, -19357,
  -19519, -19680, -19840, -20000, -20159, -20317, -20474, -20631,
  -20787, -20942, -21096, -21249, -21402, -21554, -21705, -21855,
  -22004, -22153, -22301, -22448, -22594, -22739, -22883, -23027,
  -23169, -23311, -23452, -23592, -23731, -23869, -24006, -24143,
  -24278, -24413, -24546, -24679, -24811, -24942, -25072, -25201,
  -25329, -25456, -25582, -25707, -25831, -25954, -26077, -26198,
  -26318, -26437, -26556, -26673, -26789, -26905, -27019, -27132,
  -27244, -27355, -27466, -27575, -27683, -27790, -27896, -28001,
  -28105, -28208, -28309, -28410, -28510, -28608, -28706, -28802,
  -28897, -28992, -29085, -29177, -29268, -29358, -29446, -29534,
  -29621, -29706, -29790, -29873, -29955, -30036, -30116, -30195,
  -30272, -30349, -30424, -30498, -30571, -30643, -30713, -30783,
  -30851, -30918, -30984, -31049, -31113, -31175, -31236, -31297,
  -31356, -31413, -31470, -31525, -31580, -31633, -31684, -31735,
  -31785, -31833, -31880, -31926, -31970, -32014, -32056, -32097,
  -32137, -32176, -32213, -32249, -32284, -32318, -32350, -32382,
  -32412, -32441, -32468, -32495, -32520, -32544, -32567, -32588,
  -32609, -32628, -32646, -32662, -32678, -32692, -32705, -32717,
  -32727, -32736, -32744, -32751, -32757, -32761, -32764, -32766,
};


#define FFT_N      256                     // set to 64 point fft
#define LOG2_FFT_N 8                       // log2(N_WAVE)
#define twoPi 6.28318531


int16_t fr[FFT_N] = {0}, fi[FFT_N] = {0};

void setup() {
  Serial.begin(115200); // output on the serial port
  delay(2000);
  int i, k;
  float f1 = 2.0, f2 = 5.0; //the two input frequencies (bin values)

  for (i = 0 ; i < FFT_N ; i++) { // create samples

    // amplitudes are 1000 for f1 and 500 for f2
    k = 800 * sin(2 * PI * f1 * i / FFT_N) + 500.*sin(2 * PI * f2 * i / FFT_N);

    fr[i] = k; // real data
    fi[i] = 0; // imaginary part
  }


  // BEGIN ----- multiply with hamming window
  for (uint16_t i = 0; i < (FFT_N >> 1); i++)
  {
    double ratio = ((double) i / (double)(FFT_N - 1));
    double weighingFactor = (0.54 - (0.46 * cos(twoPi * ratio)))*255;

    fr[i] *= weighingFactor;
    fr[FFT_N - (i + 1)] *= weighingFactor;
  }
  // END ----- multiply with hamming window


  fix_fft(fr, fi, LOG2_FFT_N, 0);

  // print bin index and amplitude

  Serial.println(F("bin\tamplitude"));
  for (i = 0; i < FFT_N / 2; i++) {
    Serial.print(i);
    Serial.print("\t");
    float mag2 = (int32_t)fr[i] * fr[i] + (int32_t)fi[i] * fi[i];
    Serial.println(sqrt(mag2));
  }
}

void loop() {}

/* fix_fft.c - Fixed-point in-place Fast Fourier Transform  */
/*
  All data are fixed-point int16_t integers, in which -32768
  to +32768 represent -1.0 to +1.0 respectively. Integer
  arithmetic is used for speed, instead of the more natural
  floating-point.

  For the forward FFT (time -> freq), fixed scaling is
  performed to prevent arithmetic overflow, and to map a 0dB
  sine/cosine wave (i.e. amplitude = 32767) to two -6dB freq
  coefficients. The return value is always 0.

  For the inverse FFT (freq -> time), fixed scaling cannot be
  done, as two 0dB coefficients would sum to a peak amplitude
  of 64K, overflowing the 32k range of the fixed-point integers.
  Thus, the fix_fft() routine performs variable scaling, and
  returns a value which is the number of bits LEFT by which
  the output must be shifted to get the actual amplitude
  (i.e. if fix_fft() returns 3, each value of fr[] and fi[]
  must be multiplied by 8 (2**3) for proper scaling.
  Clearly, this cannot be done within fixed-point int16_t
  integers. In practice, if the result is to be used as a
  filter, the scale_shift can usually be ignored, as the
  result will be approximately correctly normalized as is.

  Written by:  Tom Roberts  11/8/89
  Made portable:  Malcolm Slaney 12/15/94 malcolm@interval.com
  Enhanced:  Dimitrios P. Bouras  14 Jun 2006 dbouras@ieee.org
*/

/*
  FIX_MPY() - fixed-point multiplication & scaling.
  Substitute inline assembly for hardware-specific
  optimization suited to a particular DSP processor.
  Scaling ensures that result remains 16-bit.
*/
inline int16_t FIX_MPY(int16_t a, int16_t b)
{
  /* shift right one less bit (i.e. 15-1) */
  int32_t c = ( (int32_t)a * (int32_t)b) >> 14;
  /* last bit shifted out = rounding-bit */
  b = c & 0x01;
  /* last shift + rounding bit */
  a = (c >> 1) + b;
  return a;
}

/*
  fix_fft() - perform forward/inverse fast Fourier transform.
  fr[n],fi[n] are real and imaginary arrays, both INPUT AND
  RESULT (in-place FFT), with 0 <= n < 2**m; set inverse to
  0 for forward transform (FFT), or 1 for iFFT.
*/
int fix_fft(int16_t fr[], int16_t fi[], int16_t m, int16_t inverse)
{
  int mr, nn, i, j, l, k, istep, n, scale, shift;
  int16_t qr, qi, tr, ti, wr, wi;

  n = 1 << m;

  /* max FFT size = N_WAVE */
  if (n > N_WAVE)
    return -1;

  mr = 0;
  nn = n - 1;
  scale = 0;

  /* decimation in time - re-order data */
  for (m = 1; m <= nn; ++m) {
    l = n;
    do {
      l >>= 1;
    } while (mr + l > nn);
    mr = (mr & (l - 1)) + l;

    if (mr <= m)
      continue;
    tr = fr[m];
    fr[m] = fr[mr];
    fr[mr] = tr;
    ti = fi[m];
    fi[m] = fi[mr];
    fi[mr] = ti;
  }

  l = 1;
  k = LOG2_N_WAVE - 1;
  while (l < n) {
    if (inverse) {
      /* variable scaling, depending upon data */
      shift = 0;
      for (i = 0; i < n; ++i) {
        j = fr[i];
        if (j < 0)
          j = -j;
        m = fi[i];
        if (m < 0)
          m = -m;
        if (j > 16383 || m > 16383) {
          shift = 1;
          break;
        }
      }
      if (shift)
        ++scale;
    } else {
      /*
        fixed scaling, for proper normalization --
        there will be log2(n) passes, so this results
        in an overall factor of 1/n, distributed to
        maximize arithmetic accuracy.
      */
      shift = 1;
    }
    /*
      it may not be obvious, but the shift will be
      performed on each data point exactly once,
      during this pass.
    */
    istep = l << 1;
    for (m = 0; m < l; ++m) {
      j = m << k;
      /* 0 <= j < N_WAVE/2 */
      wr =  pgm_read_word_near(Sinewave + j + N_WAVE / 4);
      wi = -pgm_read_word_near(Sinewave + j);
      if (inverse)
        wi = -wi;
      if (shift) {
        wr >>= 1;
        wi >>= 1;
      }
      for (i = m; i < n; i += istep) {
        j = i + l;
        tr = FIX_MPY(wr, fr[j]) - FIX_MPY(wi, fi[j]);
        ti = FIX_MPY(wr, fi[j]) + FIX_MPY(wi, fr[j]);
        qr = fr[i];
        qi = fi[i];
        if (shift) {
          qr >>= 1;
          qi >>= 1;
        }
        fr[j] = qr - tr;
        fi[j] = qi - ti;
        fr[i] = qr + tr;
        fi[i] = qi + ti;
      }
    }
    --k;
    l = istep;
  }
  return scale;
}

/*
  fix_fftr() - forward/inverse FFT on array of real numbers.
  Real FFT/iFFT using half-size complex FFT by distributing
  even/odd samples into real/imaginary arrays respectively.
  In order to save data space (i.e. to avoid two arrays, one
  for real, one for imaginary samples), we proceed in the
  following two steps: a) samples are rearranged in the real
  array so that all even samples are in places 0-(N/2-1) and
  all imaginary samples in places (N/2)-(N-1), and b) fix_fft
  is called with fr and fi pointing to index 0 and index N/2
  respectively in the original array. The above guarantees
  that fix_fft "sees" consecutive real samples as alternating
  real and imaginary samples in the complex array.
*/
int fix_fftr(int16_t f[], int m, int inverse)
{
  int i, N = 1 << (m - 1), scale = 0;
  int16_t tt, *fr = f, *fi = &f[N];

  if (inverse)
    scale = fix_fft(fi, fr, m - 1, inverse);
  for (i = 1; i < N; i += 2) {
    tt = f[N + i - 1];
    f[N + i - 1] = f[i];
    f[i] = tt;
  }
  if (! inverse)
    scale = fix_fft(fi, fr, m - 1, inverse);
  return scale;
}
