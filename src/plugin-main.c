/*
UDP MJPEG Preview Streamer for OBS
Copyright (C) 2024

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program. If not, see <https://www.gnu.org/licenses/>
*/

#include <obs-module.h>
#include <obs-frontend-api.h>
#include <graphics/graphics.h>
#include <util/platform.h>
#include <util/threading.h>
#include <plugin-support.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef SOCKET socket_t;
#define SOCKET_INVALID INVALID_SOCKET
#define SOCKET_ERROR_VAL SOCKET_ERROR
#define CLOSE_SOCKET closesocket
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
typedef int socket_t;
#define SOCKET_INVALID -1
#define SOCKET_ERROR_VAL -1
#define CLOSE_SOCKET close
#endif

/* Default configuration */
#define DEFAULT_WIDTH 700
#define DEFAULT_HEIGHT 700
#define DEFAULT_FPS 144
#define DEFAULT_PORT 5000
#define DEFAULT_HOST "127.0.0.1"
#define DEFAULT_QUALITY 80

/* Maximum UDP packet size (considering MTU) */
#define MAX_UDP_PACKET_SIZE 65507

/* Simple JPEG encoder structure */
struct jpeg_encoder {
	uint8_t *buffer;
	size_t buffer_size;
	int quality;
};

/* Main plugin data structure */
struct udp_mjpeg_streamer {
	obs_output_t *output;
	obs_source_t *source;

	/* Configuration */
	int width;
	int height;
	int fps;
	int port;
	char *host;
	int quality;

	/* Networking */
	socket_t sock;
	struct sockaddr_in dest_addr;
	bool socket_initialized;
	uint32_t frame_id;
	uint8_t *packet_buffer;

	/* Threading */
	pthread_t capture_thread;
	volatile bool active;
	volatile bool stopping;
	pthread_mutex_t mutex;

	/* Frame capture */
	gs_texrender_t *texrender;
	gs_stagesurf_t *stagesurface;
	uint8_t *video_data;
	uint32_t video_linesize;

	/* JPEG encoding buffer */
	uint8_t *jpeg_buffer;
	size_t jpeg_buffer_size;

	/* Timing */
	uint64_t last_frame_time;
	uint64_t frame_interval_ns;

	/* Statistics */
	uint64_t frames_sent;
	uint64_t bytes_sent;
};

/* Forward declarations */
static const char *udp_mjpeg_getname(void *unused);
static void *udp_mjpeg_create(obs_data_t *settings, obs_output_t *output);
static void udp_mjpeg_destroy(void *data);
static bool udp_mjpeg_start(void *data);
static void udp_mjpeg_stop(void *data, uint64_t ts);
static void udp_mjpeg_defaults(obs_data_t *settings);
static obs_properties_t *udp_mjpeg_properties(void *unused);

/* Simple JPEG marker writing utilities */
static void write_byte(uint8_t **buf, uint8_t val)
{
	**buf = val;
	(*buf)++;
}

static void write_word(uint8_t **buf, uint16_t val)
{
	write_byte(buf, (uint8_t)(val >> 8));
	write_byte(buf, (uint8_t)(val & 0xFF));
}

/* Standard JPEG quantization tables (scaled by quality) */
static const uint8_t std_luminance_qt[64] = {
	16, 11, 10, 16, 24,  40,  51,  61,  12, 12, 14, 19, 26,  58,  60,  55,
	14, 13, 16, 24, 40,  57,  69,  56,  14, 17, 22, 29, 51,  87,  80,  62,
	18, 22, 37, 56, 68,  109, 103, 77,  24, 35, 55, 64, 81,  104, 113, 92,
	49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99};

static const uint8_t std_chrominance_qt[64] = {
	17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99,
	24, 26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99};

/* Zigzag order for DCT coefficients */
static const uint8_t zigzag[64] = {0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18,
				   11, 4,  5,  12, 19, 26, 33, 40, 48, 41, 34, 27, 20,
				   13, 6,  7,  14, 21, 28, 35, 42, 49, 56, 57, 50, 43,
				   36, 29, 22, 15, 23, 30, 37, 44, 51, 58, 59, 52, 45,
				   38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63};

/* Standard Huffman tables */
static const uint8_t dc_luminance_bits[17] = {0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t dc_luminance_val[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

static const uint8_t dc_chrominance_bits[17] = {0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0};
static const uint8_t dc_chrominance_val[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

static const uint8_t ac_luminance_bits[17] = {0, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d};
static const uint8_t ac_luminance_val[] = {
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61,
	0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52,
	0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25,
	0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45,
	0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64,
	0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83,
	0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
	0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3,
	0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8,
	0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};

static const uint8_t ac_chrominance_bits[17] = {0, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77};
static const uint8_t ac_chrominance_val[] = {
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61,
	0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33,
	0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18,
	0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44,
	0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63,
	0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97,
	0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
	0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca,
	0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7,
	0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};

/* Bit writer structure */
struct bit_writer {
	uint8_t *buffer;
	uint8_t *ptr;
	size_t max_size;
	uint32_t bit_buffer;
	int bit_count;
};

static void bit_writer_init(struct bit_writer *bw, uint8_t *buffer, size_t max_size)
{
	bw->buffer = buffer;
	bw->ptr = buffer;
	bw->max_size = max_size;
	bw->bit_buffer = 0;
	bw->bit_count = 0;
}

static void bit_writer_put_bits(struct bit_writer *bw, int bits, int value)
{
	bw->bit_buffer = (bw->bit_buffer << bits) | (value & ((1 << bits) - 1));
	bw->bit_count += bits;

	while (bw->bit_count >= 8) {
		uint8_t byte_val = (uint8_t)(bw->bit_buffer >> (bw->bit_count - 8));
		*bw->ptr++ = byte_val;
		if (byte_val == 0xFF) {
			*bw->ptr++ = 0x00; /* Byte stuffing */
		}
		bw->bit_count -= 8;
	}
}

static void bit_writer_flush(struct bit_writer *bw)
{
	if (bw->bit_count > 0) {
		bw->bit_buffer <<= (8 - bw->bit_count);
		uint8_t byte_val = (uint8_t)(bw->bit_buffer & 0xFF);
		*bw->ptr++ = byte_val;
		if (byte_val == 0xFF) {
			*bw->ptr++ = 0x00;
		}
	}
}

static size_t bit_writer_size(struct bit_writer *bw)
{
	return (size_t)(bw->ptr - bw->buffer);
}

/* Huffman code generation */
struct huffman_table {
	uint16_t code[256];
	uint8_t size[256];
};

static void generate_huffman_table(struct huffman_table *ht, const uint8_t *bits, const uint8_t *val)
{
	int huffsize[257];
	int huffcode[257];
	int p = 0;

	for (int l = 1; l <= 16; l++) {
		for (int i = 1; i <= bits[l]; i++) {
			huffsize[p++] = l;
		}
	}
	huffsize[p] = 0;

	int code = 0;
	int si = huffsize[0];
	p = 0;

	while (huffsize[p]) {
		while (huffsize[p] == si) {
			huffcode[p++] = code++;
		}
		code <<= 1;
		si++;
	}

	for (int i = 0; i < 256; i++) {
		ht->code[i] = 0;
		ht->size[i] = 0;
	}

	p = 0;
	int count = 0;
	for (int l = 1; l <= 16; l++) {
		count += bits[l];
	}

	for (int i = 0; i < count; i++) {
		ht->code[val[i]] = (uint16_t)huffcode[i];
		ht->size[val[i]] = (uint8_t)huffsize[i];
	}
}

/* DCT and encoding state */
struct jpeg_state {
	struct huffman_table dc_lum_ht;
	struct huffman_table ac_lum_ht;
	struct huffman_table dc_chr_ht;
	struct huffman_table ac_chr_ht;
	uint8_t lum_qt[64];
	uint8_t chr_qt[64];
	int16_t prev_dc_y;
	int16_t prev_dc_cb;
	int16_t prev_dc_cr;
};

static void jpeg_state_init(struct jpeg_state *state, int quality)
{
	generate_huffman_table(&state->dc_lum_ht, dc_luminance_bits, dc_luminance_val);
	generate_huffman_table(&state->ac_lum_ht, ac_luminance_bits, ac_luminance_val);
	generate_huffman_table(&state->dc_chr_ht, dc_chrominance_bits, dc_chrominance_val);
	generate_huffman_table(&state->ac_chr_ht, ac_chrominance_bits, ac_chrominance_val);

	/* Scale quantization tables based on quality */
	int scale = (quality < 50) ? (5000 / quality) : (200 - quality * 2);

	for (int i = 0; i < 64; i++) {
		int lq = (std_luminance_qt[i] * scale + 50) / 100;
		int cq = (std_chrominance_qt[i] * scale + 50) / 100;
		state->lum_qt[i] = (uint8_t)(lq < 1 ? 1 : (lq > 255 ? 255 : lq));
		state->chr_qt[i] = (uint8_t)(cq < 1 ? 1 : (cq > 255 ? 255 : cq));
	}

	state->prev_dc_y = 0;
	state->prev_dc_cb = 0;
	state->prev_dc_cr = 0;
}

/* Simple integer DCT (fast but lower quality) */
static void fdct(int16_t *data)
{
	/* Simplified integer DCT */
	int tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7;
	int tmp10, tmp11, tmp12, tmp13;
	int z1, z2, z3, z4, z5;
	int16_t *dataptr = data;

	/* Pass 1: process rows */
	for (int i = 0; i < 8; i++) {
		tmp0 = dataptr[0] + dataptr[7];
		tmp7 = dataptr[0] - dataptr[7];
		tmp1 = dataptr[1] + dataptr[6];
		tmp6 = dataptr[1] - dataptr[6];
		tmp2 = dataptr[2] + dataptr[5];
		tmp5 = dataptr[2] - dataptr[5];
		tmp3 = dataptr[3] + dataptr[4];
		tmp4 = dataptr[3] - dataptr[4];

		tmp10 = tmp0 + tmp3;
		tmp13 = tmp0 - tmp3;
		tmp11 = tmp1 + tmp2;
		tmp12 = tmp1 - tmp2;

		dataptr[0] = (int16_t)((tmp10 + tmp11) << 2);
		dataptr[4] = (int16_t)((tmp10 - tmp11) << 2);

		z1 = (tmp12 + tmp13) * 4433;
		dataptr[2] = (int16_t)((z1 + tmp13 * 6270) >> 11);
		dataptr[6] = (int16_t)((z1 - tmp12 * 15137) >> 11);

		z1 = tmp4 + tmp7;
		z2 = tmp5 + tmp6;
		z3 = tmp4 + tmp6;
		z4 = tmp5 + tmp7;
		z5 = (z3 + z4) * 9633;

		tmp4 = tmp4 * 2446;
		tmp5 = tmp5 * 16819;
		tmp6 = tmp6 * 25172;
		tmp7 = tmp7 * 12299;
		z1 = z1 * -7373;
		z2 = z2 * -20995;
		z3 = z3 * -16069;
		z4 = z4 * -3196;

		z3 += z5;
		z4 += z5;

		dataptr[7] = (int16_t)((tmp4 + z1 + z3) >> 11);
		dataptr[5] = (int16_t)((tmp5 + z2 + z4) >> 11);
		dataptr[3] = (int16_t)((tmp6 + z2 + z3) >> 11);
		dataptr[1] = (int16_t)((tmp7 + z1 + z4) >> 11);

		dataptr += 8;
	}

	/* Pass 2: process columns */
	dataptr = data;
	for (int i = 0; i < 8; i++) {
		tmp0 = dataptr[0 * 8] + dataptr[7 * 8];
		tmp7 = dataptr[0 * 8] - dataptr[7 * 8];
		tmp1 = dataptr[1 * 8] + dataptr[6 * 8];
		tmp6 = dataptr[1 * 8] - dataptr[6 * 8];
		tmp2 = dataptr[2 * 8] + dataptr[5 * 8];
		tmp5 = dataptr[2 * 8] - dataptr[5 * 8];
		tmp3 = dataptr[3 * 8] + dataptr[4 * 8];
		tmp4 = dataptr[3 * 8] - dataptr[4 * 8];

		tmp10 = tmp0 + tmp3;
		tmp13 = tmp0 - tmp3;
		tmp11 = tmp1 + tmp2;
		tmp12 = tmp1 - tmp2;

		dataptr[0 * 8] = (int16_t)((tmp10 + tmp11) >> 2);
		dataptr[4 * 8] = (int16_t)((tmp10 - tmp11) >> 2);

		z1 = (tmp12 + tmp13) * 4433;
		dataptr[2 * 8] = (int16_t)((z1 + tmp13 * 6270) >> 15);
		dataptr[6 * 8] = (int16_t)((z1 - tmp12 * 15137) >> 15);

		z1 = tmp4 + tmp7;
		z2 = tmp5 + tmp6;
		z3 = tmp4 + tmp6;
		z4 = tmp5 + tmp7;
		z5 = (z3 + z4) * 9633;

		tmp4 = tmp4 * 2446;
		tmp5 = tmp5 * 16819;
		tmp6 = tmp6 * 25172;
		tmp7 = tmp7 * 12299;
		z1 = z1 * -7373;
		z2 = z2 * -20995;
		z3 = z3 * -16069;
		z4 = z4 * -3196;

		z3 += z5;
		z4 += z5;

		dataptr[7 * 8] = (int16_t)((tmp4 + z1 + z3) >> 15);
		dataptr[5 * 8] = (int16_t)((tmp5 + z2 + z4) >> 15);
		dataptr[3 * 8] = (int16_t)((tmp6 + z2 + z3) >> 15);
		dataptr[1 * 8] = (int16_t)((tmp7 + z1 + z4) >> 15);

		dataptr++;
	}
}

/* Calculate the number of bits needed to represent a value */
static int calc_bits(int val)
{
	int absval = val < 0 ? -val : val;
	int bits = 0;
	while (absval) {
		absval >>= 1;
		bits++;
	}
	return bits;
}

/* Encode a DC coefficient */
static void encode_dc(struct bit_writer *bw, struct huffman_table *ht, int diff)
{
	int bits = calc_bits(diff);
	int val = diff;

	if (diff < 0) {
		val = diff - 1;
	}

	/* Output Huffman code for category */
	bit_writer_put_bits(bw, ht->size[bits], ht->code[bits]);

	/* Output additional bits for value */
	if (bits > 0) {
		bit_writer_put_bits(bw, bits, val & ((1 << bits) - 1));
	}
}

/* Encode AC coefficients */
static void encode_ac(struct bit_writer *bw, struct huffman_table *ht, int16_t *block)
{
	int run = 0;

	for (int i = 1; i < 64; i++) {
		int val = block[zigzag[i]];

		if (val == 0) {
			run++;
		} else {
			while (run > 15) {
				/* Output ZRL (zero run length = 16 zeros) */
				bit_writer_put_bits(bw, ht->size[0xF0], ht->code[0xF0]);
				run -= 16;
			}

			int bits = calc_bits(val);
			int symbol = (run << 4) | bits;

			bit_writer_put_bits(bw, ht->size[symbol], ht->code[symbol]);

			if (val < 0) {
				val = val - 1;
			}
			bit_writer_put_bits(bw, bits, val & ((1 << bits) - 1));
			run = 0;
		}
	}

	if (run > 0) {
		/* Output EOB */
		bit_writer_put_bits(bw, ht->size[0], ht->code[0]);
	}
}

/* Encode a single MCU (8x8 block) */
static void encode_block(struct bit_writer *bw, struct jpeg_state *state, int16_t *block,
			 const uint8_t *qt, struct huffman_table *dc_ht, struct huffman_table *ac_ht,
			 int16_t *prev_dc)
{
	/* Quantize and prepare for encoding */
	for (int i = 0; i < 64; i++) {
		block[i] = (int16_t)((block[i] + (qt[i] >> 1)) / qt[i]);
	}

	/* Encode DC coefficient (difference from previous) */
	int dc_diff = block[0] - *prev_dc;
	*prev_dc = block[0];
	encode_dc(bw, dc_ht, dc_diff);

	/* Encode AC coefficients */
	encode_ac(bw, ac_ht, block);
}

/* RGB to YCbCr conversion */
static void rgb_to_ycbcr(uint8_t r, uint8_t g, uint8_t b, int *y, int *cb, int *cr)
{
	*y = (int)(0.299 * r + 0.587 * g + 0.114 * b);
	*cb = (int)(-0.168736 * r - 0.331264 * g + 0.5 * b + 128);
	*cr = (int)(0.5 * r - 0.418688 * g - 0.081312 * b + 128);
}

/* Write JPEG headers */
static size_t write_jpeg_header(uint8_t *buffer, int width, int height, struct jpeg_state *state)
{
	uint8_t *p = buffer;

	/* SOI - Start of Image */
	write_byte(&p, 0xFF);
	write_byte(&p, 0xD8);

	/* APP0 - JFIF marker */
	write_byte(&p, 0xFF);
	write_byte(&p, 0xE0);
	write_word(&p, 16);
	write_byte(&p, 'J');
	write_byte(&p, 'F');
	write_byte(&p, 'I');
	write_byte(&p, 'F');
	write_byte(&p, 0);
	write_byte(&p, 1);
	write_byte(&p, 1);
	write_byte(&p, 0);
	write_word(&p, 1);
	write_word(&p, 1);
	write_byte(&p, 0);
	write_byte(&p, 0);

	/* DQT - Define Quantization Tables (luminance) */
	write_byte(&p, 0xFF);
	write_byte(&p, 0xDB);
	write_word(&p, 67);
	write_byte(&p, 0);
	for (int i = 0; i < 64; i++) {
		write_byte(&p, state->lum_qt[zigzag[i]]);
	}

	/* DQT - Define Quantization Tables (chrominance) */
	write_byte(&p, 0xFF);
	write_byte(&p, 0xDB);
	write_word(&p, 67);
	write_byte(&p, 1);
	for (int i = 0; i < 64; i++) {
		write_byte(&p, state->chr_qt[zigzag[i]]);
	}

	/* SOF0 - Start of Frame (baseline DCT) */
	write_byte(&p, 0xFF);
	write_byte(&p, 0xC0);
	write_word(&p, 17);
	write_byte(&p, 8); /* precision */
	write_word(&p, (uint16_t)height);
	write_word(&p, (uint16_t)width);
	write_byte(&p, 3); /* number of components */
	write_byte(&p, 1);
	write_byte(&p, 0x11);
	write_byte(&p, 0); /* Y: 1x1, QT 0 */
	write_byte(&p, 2);
	write_byte(&p, 0x11);
	write_byte(&p, 1); /* Cb: 1x1, QT 1 */
	write_byte(&p, 3);
	write_byte(&p, 0x11);
	write_byte(&p, 1); /* Cr: 1x1, QT 1 */

	/* DHT - Define Huffman Table (DC luminance) */
	write_byte(&p, 0xFF);
	write_byte(&p, 0xC4);
	int len = 19;
	for (int i = 1; i <= 16; i++)
		len += dc_luminance_bits[i];
	write_word(&p, (uint16_t)len);
	write_byte(&p, 0x00);
	for (int i = 1; i <= 16; i++)
		write_byte(&p, dc_luminance_bits[i]);
	for (int i = 0; i < 12; i++)
		write_byte(&p, dc_luminance_val[i]);

	/* DHT - Define Huffman Table (AC luminance) */
	write_byte(&p, 0xFF);
	write_byte(&p, 0xC4);
	len = 19;
	for (int i = 1; i <= 16; i++)
		len += ac_luminance_bits[i];
	write_word(&p, (uint16_t)len);
	write_byte(&p, 0x10);
	for (int i = 1; i <= 16; i++)
		write_byte(&p, ac_luminance_bits[i]);
	for (int i = 0; i < 162; i++)
		write_byte(&p, ac_luminance_val[i]);

	/* DHT - Define Huffman Table (DC chrominance) */
	write_byte(&p, 0xFF);
	write_byte(&p, 0xC4);
	len = 19;
	for (int i = 1; i <= 16; i++)
		len += dc_chrominance_bits[i];
	write_word(&p, (uint16_t)len);
	write_byte(&p, 0x01);
	for (int i = 1; i <= 16; i++)
		write_byte(&p, dc_chrominance_bits[i]);
	for (int i = 0; i < 12; i++)
		write_byte(&p, dc_chrominance_val[i]);

	/* DHT - Define Huffman Table (AC chrominance) */
	write_byte(&p, 0xFF);
	write_byte(&p, 0xC4);
	len = 19;
	for (int i = 1; i <= 16; i++)
		len += ac_chrominance_bits[i];
	write_word(&p, (uint16_t)len);
	write_byte(&p, 0x11);
	for (int i = 1; i <= 16; i++)
		write_byte(&p, ac_chrominance_bits[i]);
	for (int i = 0; i < 162; i++)
		write_byte(&p, ac_chrominance_val[i]);

	/* SOS - Start of Scan */
	write_byte(&p, 0xFF);
	write_byte(&p, 0xDA);
	write_word(&p, 12);
	write_byte(&p, 3); /* number of components */
	write_byte(&p, 1);
	write_byte(&p, 0x00); /* Y: DC table 0, AC table 0 */
	write_byte(&p, 2);
	write_byte(&p, 0x11); /* Cb: DC table 1, AC table 1 */
	write_byte(&p, 3);
	write_byte(&p, 0x11); /* Cr: DC table 1, AC table 1 */
	write_byte(&p, 0);
	write_byte(&p, 63);
	write_byte(&p, 0);

	return (size_t)(p - buffer);
}

/* Encode RGBA image to JPEG */
static size_t encode_jpeg(uint8_t *output, size_t max_size, uint8_t *rgba, int width, int height,
			  int quality)
{
	struct jpeg_state state;
	jpeg_state_init(&state, quality);

	size_t header_size = write_jpeg_header(output, width, height, &state);

	struct bit_writer bw;
	bit_writer_init(&bw, output + header_size, max_size - header_size - 2);

	int16_t block_y[64], block_cb[64], block_cr[64];

	/* Process image in 8x8 blocks */
	for (int by = 0; by < height; by += 8) {
		for (int bx = 0; bx < width; bx += 8) {
			/* Extract block */
			for (int j = 0; j < 8; j++) {
				for (int i = 0; i < 8; i++) {
					int x = bx + i;
					int y = by + j;

					/* Handle boundary */
					if (x >= width)
						x = width - 1;
					if (y >= height)
						y = height - 1;

					int idx = (y * width + x) * 4;
					uint8_t r = rgba[idx];
					uint8_t g = rgba[idx + 1];
					uint8_t b = rgba[idx + 2];

					int yval, cb, cr;
					rgb_to_ycbcr(r, g, b, &yval, &cb, &cr);

					block_y[j * 8 + i] = (int16_t)(yval - 128);
					block_cb[j * 8 + i] = (int16_t)(cb - 128);
					block_cr[j * 8 + i] = (int16_t)(cr - 128);
				}
			}

			/* Apply DCT */
			fdct(block_y);
			fdct(block_cb);
			fdct(block_cr);

			/* Encode blocks */
			encode_block(&bw, &state, block_y, state.lum_qt, &state.dc_lum_ht,
				     &state.ac_lum_ht, &state.prev_dc_y);
			encode_block(&bw, &state, block_cb, state.chr_qt, &state.dc_chr_ht,
				     &state.ac_chr_ht, &state.prev_dc_cb);
			encode_block(&bw, &state, block_cr, state.chr_qt, &state.dc_chr_ht,
				     &state.ac_chr_ht, &state.prev_dc_cr);
		}
	}

	bit_writer_flush(&bw);

	/* Write EOI marker */
	uint8_t *p = output + header_size + bit_writer_size(&bw);
	*p++ = 0xFF;
	*p++ = 0xD9;

	return header_size + bit_writer_size(&bw) + 2;
}

/* Initialize socket */
static bool init_socket(struct udp_mjpeg_streamer *stream)
{
#ifdef _WIN32
	WSADATA wsa_data;
	if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
		obs_log(LOG_ERROR, "WSAStartup failed");
		return false;
	}
#endif

	stream->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (stream->sock == SOCKET_INVALID) {
		obs_log(LOG_ERROR, "Failed to create socket");
		return false;
	}

	/* Set socket options for low latency */
	int sndbuf = 1024 * 1024; /* 1MB send buffer */
	setsockopt(stream->sock, SOL_SOCKET, SO_SNDBUF, (const char *)&sndbuf, sizeof(sndbuf));

	/* Setup destination address */
	memset(&stream->dest_addr, 0, sizeof(stream->dest_addr));
	stream->dest_addr.sin_family = AF_INET;
	stream->dest_addr.sin_port = htons((uint16_t)stream->port);
	inet_pton(AF_INET, stream->host, &stream->dest_addr.sin_addr);

	stream->socket_initialized = true;
	obs_log(LOG_INFO, "Socket initialized to port %d", stream->port);

	return true;
}

/* Cleanup socket */
static void cleanup_socket(struct udp_mjpeg_streamer *stream)
{
	if (stream->socket_initialized) {
		CLOSE_SOCKET(stream->sock);
#ifdef _WIN32
		WSACleanup();
#endif
		stream->socket_initialized = false;
	}
}

/* Send JPEG frame over UDP */
static bool send_frame(struct udp_mjpeg_streamer *stream, uint8_t *data, size_t size)
{
	if (!stream->socket_initialized) {
		return false;
	}

	/*
	 * Handle frame fragmentation for large JPEG frames.
	 * Small frames (< 65507 bytes) are sent as a single UDP packet.
	 * Large frames are split into chunks with a simple header protocol.
	 * Note: For full MJPEG streaming compatibility, consider implementing
	 * RTP/MJPEG (RFC 2435) fragmentation in the future.
	 */

	if (size <= MAX_UDP_PACKET_SIZE) {
		int result = sendto(stream->sock, (const char *)data, (int)size, 0,
				    (struct sockaddr *)&stream->dest_addr,
				    sizeof(stream->dest_addr));
		if (result == SOCKET_ERROR_VAL) {
			return false;
		}
		stream->bytes_sent += size;
		stream->frames_sent++;
		return true;
	} else {
		/* Frame is too large, split into chunks with simple header:
		   [4 bytes frame_id][4 bytes chunk_id][4 bytes total_chunks][data] */
		stream->frame_id++;

		size_t chunk_data_size = MAX_UDP_PACKET_SIZE - 12; /* Header is 12 bytes */
		uint32_t total_chunks = (uint32_t)((size + chunk_data_size - 1) / chunk_data_size);

		if (!stream->packet_buffer)
			return false;

		size_t offset = 0;
		for (uint32_t chunk = 0; chunk < total_chunks; chunk++) {
			size_t chunk_size =
				(offset + chunk_data_size > size) ? (size - offset) : chunk_data_size;

			/* Write header */
			memcpy(stream->packet_buffer, &stream->frame_id, 4);
			memcpy(stream->packet_buffer + 4, &chunk, 4);
			memcpy(stream->packet_buffer + 8, &total_chunks, 4);
			memcpy(stream->packet_buffer + 12, data + offset, chunk_size);

			int result = sendto(stream->sock, (const char *)stream->packet_buffer,
					    (int)(chunk_size + 12), 0,
					    (struct sockaddr *)&stream->dest_addr,
					    sizeof(stream->dest_addr));

			if (result == SOCKET_ERROR_VAL) {
				return false;
			}

			offset += chunk_size;
		}

		stream->bytes_sent += size + (total_chunks * 12);
		stream->frames_sent++;
		return true;
	}
}

/* Capture and process frame from OBS preview */
static void capture_frame(struct udp_mjpeg_streamer *stream)
{
	obs_source_t *source = obs_frontend_get_current_preview_scene();
	if (!source) {
		source = obs_frontend_get_current_scene();
	}

	if (!source) {
		return;
	}

	uint32_t src_width = obs_source_get_width(source);
	uint32_t src_height = obs_source_get_height(source);

	if (src_width == 0 || src_height == 0) {
		obs_source_release(source);
		return;
	}

	obs_enter_graphics();

	/* Create or recreate texrender if needed */
	if (!stream->texrender) {
		stream->texrender = gs_texrender_create(GS_RGBA, GS_ZS_NONE);
	}

	if (!stream->stagesurface || gs_stagesurface_get_width(stream->stagesurface) != (uint32_t)stream->width ||
	    gs_stagesurface_get_height(stream->stagesurface) != (uint32_t)stream->height) {

		if (stream->stagesurface) {
			gs_stagesurface_destroy(stream->stagesurface);
		}
		stream->stagesurface =
			gs_stagesurface_create((uint32_t)stream->width, (uint32_t)stream->height, GS_RGBA);
	}

	/* Render source to texture */
	gs_texrender_reset(stream->texrender);

	if (gs_texrender_begin(stream->texrender, (uint32_t)stream->width, (uint32_t)stream->height)) {
		struct vec4 clear_color;
		vec4_zero(&clear_color);
		gs_clear(GS_CLEAR_COLOR, &clear_color, 0.0f, 0);
		gs_ortho(0.0f, (float)src_width, 0.0f, (float)src_height, -100.0f, 100.0f);

		obs_source_video_render(source);
		gs_texrender_end(stream->texrender);

		/* Stage the texture */
		gs_texture_t *tex = gs_texrender_get_texture(stream->texrender);
		if (tex) {
			gs_stage_texture(stream->stagesurface, tex);

			/* Map and copy the data */
			uint8_t *video_data = NULL;
			uint32_t video_linesize = 0;

			if (gs_stagesurface_map(stream->stagesurface, &video_data, &video_linesize)) {
				/* Allocate buffer for frame data */
				size_t frame_size = (size_t)(stream->width * stream->height * 4);
				uint8_t *frame_buffer = (uint8_t *)bmalloc(frame_size);

				if (frame_buffer) {
					/* Copy frame data */
					for (int y = 0; y < stream->height; y++) {
						memcpy(frame_buffer + y * stream->width * 4,
						       video_data + y * video_linesize,
						       (size_t)(stream->width * 4));
					}

					/* Encode to JPEG */
					size_t jpeg_size =
						encode_jpeg(stream->jpeg_buffer, stream->jpeg_buffer_size,
							    frame_buffer, stream->width, stream->height,
							    stream->quality);

					/* Send over UDP */
					send_frame(stream, stream->jpeg_buffer, jpeg_size);

					bfree(frame_buffer);
				}

				gs_stagesurface_unmap(stream->stagesurface);
			}
		}
	}

	obs_leave_graphics();
	obs_source_release(source);
}

/* Main capture thread */
static void *capture_thread_func(void *data)
{
	struct udp_mjpeg_streamer *stream = (struct udp_mjpeg_streamer *)data;

	obs_log(LOG_INFO, "Capture thread started");

	while (stream->active && !stream->stopping) {
		uint64_t current_time = os_gettime_ns();

		if (current_time - stream->last_frame_time >= stream->frame_interval_ns) {
			capture_frame(stream);
			stream->last_frame_time = current_time;
		}

		/* Sleep for a short interval to avoid busy waiting */
		os_sleep_ms(1);
	}

	obs_log(LOG_INFO, "Capture thread stopped. Sent %llu frames, %llu bytes",
		(unsigned long long)stream->frames_sent, (unsigned long long)stream->bytes_sent);

	return NULL;
}

/* Output info callbacks */
static const char *udp_mjpeg_getname(void *unused)
{
	UNUSED_PARAMETER(unused);
	return "UDP MJPEG Preview Streamer";
}

static void *udp_mjpeg_create(obs_data_t *settings, obs_output_t *output)
{
	struct udp_mjpeg_streamer *stream =
		(struct udp_mjpeg_streamer *)bzalloc(sizeof(struct udp_mjpeg_streamer));

	stream->output = output;
	stream->width = (int)obs_data_get_int(settings, "width");
	stream->height = (int)obs_data_get_int(settings, "height");
	stream->fps = (int)obs_data_get_int(settings, "fps");
	stream->port = (int)obs_data_get_int(settings, "port");
	stream->host = bstrdup(obs_data_get_string(settings, "host"));
	stream->quality = (int)obs_data_get_int(settings, "quality");

	stream->frame_interval_ns = 1000000000ULL / (uint64_t)stream->fps;

	/* Allocate JPEG buffer (enough for worst case) */
	stream->jpeg_buffer_size = (size_t)(stream->width * stream->height * 3 + 1024);
	stream->jpeg_buffer = (uint8_t *)bmalloc(stream->jpeg_buffer_size);

	/* Allocate packet buffer for frame fragmentation */
	stream->packet_buffer = (uint8_t *)bmalloc(MAX_UDP_PACKET_SIZE);
	stream->frame_id = 0;

	pthread_mutex_init(&stream->mutex, NULL);

	obs_log(LOG_INFO, "UDP MJPEG Streamer created: %dx%d @ %dfps (quality: %d)",
		stream->width, stream->height, stream->fps, stream->quality);

	return stream;
}

static void udp_mjpeg_destroy(void *data)
{
	struct udp_mjpeg_streamer *stream = (struct udp_mjpeg_streamer *)data;

	if (!stream)
		return;

	cleanup_socket(stream);

	pthread_mutex_destroy(&stream->mutex);

	if (stream->jpeg_buffer) {
		bfree(stream->jpeg_buffer);
	}

	if (stream->packet_buffer) {
		bfree(stream->packet_buffer);
	}

	if (stream->host) {
		bfree(stream->host);
	}

	obs_enter_graphics();
	if (stream->texrender) {
		gs_texrender_destroy(stream->texrender);
	}
	if (stream->stagesurface) {
		gs_stagesurface_destroy(stream->stagesurface);
	}
	obs_leave_graphics();

	bfree(stream);
}

static bool udp_mjpeg_start(void *data)
{
	struct udp_mjpeg_streamer *stream = (struct udp_mjpeg_streamer *)data;

	if (!init_socket(stream)) {
		return false;
	}

	stream->active = true;
	stream->stopping = false;
	stream->frames_sent = 0;
	stream->bytes_sent = 0;
	stream->last_frame_time = os_gettime_ns();

	if (pthread_create(&stream->capture_thread, NULL, capture_thread_func, stream) != 0) {
		obs_log(LOG_ERROR, "Failed to create capture thread");
		cleanup_socket(stream);
		return false;
	}

	obs_log(LOG_INFO, "UDP MJPEG streaming started");
	return true;
}

static void udp_mjpeg_stop(void *data, uint64_t ts)
{
	UNUSED_PARAMETER(ts);
	struct udp_mjpeg_streamer *stream = (struct udp_mjpeg_streamer *)data;

	if (!stream->active)
		return;

	stream->stopping = true;
	stream->active = false;

	pthread_join(stream->capture_thread, NULL);

	cleanup_socket(stream);

	obs_log(LOG_INFO, "UDP MJPEG streaming stopped");
}

static void udp_mjpeg_defaults(obs_data_t *settings)
{
	obs_data_set_default_int(settings, "width", DEFAULT_WIDTH);
	obs_data_set_default_int(settings, "height", DEFAULT_HEIGHT);
	obs_data_set_default_int(settings, "fps", DEFAULT_FPS);
	obs_data_set_default_int(settings, "port", DEFAULT_PORT);
	obs_data_set_default_string(settings, "host", DEFAULT_HOST);
	obs_data_set_default_int(settings, "quality", DEFAULT_QUALITY);
}

static obs_properties_t *udp_mjpeg_properties(void *unused)
{
	UNUSED_PARAMETER(unused);

	obs_properties_t *props = obs_properties_create();

	obs_properties_add_int(props, "width", "Width", 1, 4096, 1);
	obs_properties_add_int(props, "height", "Height", 1, 4096, 1);
	obs_properties_add_int(props, "fps", "FPS", 1, 240, 1);
	obs_properties_add_text(props, "host", "Destination Host", OBS_TEXT_DEFAULT);
	obs_properties_add_int(props, "port", "Destination Port", 1, 65535, 1);
	obs_properties_add_int_slider(props, "quality", "JPEG Quality", 1, 100, 1);

	return props;
}

static struct obs_output_info udp_mjpeg_output_info = {
	.id = "udp_mjpeg_output",
	.flags = OBS_OUTPUT_VIDEO,
	.get_name = udp_mjpeg_getname,
	.create = udp_mjpeg_create,
	.destroy = udp_mjpeg_destroy,
	.start = udp_mjpeg_start,
	.stop = udp_mjpeg_stop,
	.get_defaults = udp_mjpeg_defaults,
	.get_properties = udp_mjpeg_properties,
};

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE(PLUGIN_NAME, "en-US")

static obs_output_t *g_output = NULL;

/* Hotkey callback to toggle streaming */
static void toggle_streaming_hotkey(void *data, obs_hotkey_id id, obs_hotkey_t *hotkey, bool pressed)
{
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(id);
	UNUSED_PARAMETER(hotkey);

	if (!pressed)
		return;

	if (g_output) {
		if (obs_output_active(g_output)) {
			obs_output_stop(g_output);
		} else {
			obs_output_start(g_output);
		}
	}
}

bool obs_module_load(void)
{
	obs_log(LOG_INFO, "UDP MJPEG Preview Streamer plugin loaded (version %s)", PLUGIN_VERSION);

	obs_register_output(&udp_mjpeg_output_info);

	/* Create default output instance */
	obs_data_t *settings = obs_data_create();
	udp_mjpeg_defaults(settings);
	g_output = obs_output_create("udp_mjpeg_output", "UDP MJPEG Stream", settings, NULL);
	obs_data_release(settings);

	/* Register hotkey */
	obs_hotkey_register_frontend("udp_mjpeg_toggle", "Toggle UDP MJPEG Streaming",
				     toggle_streaming_hotkey, NULL);

	obs_log(LOG_INFO,
		"UDP MJPEG Streamer initialized. Default: %dx%d @ %dfps -> %s:%d\n"
		"Use the hotkey 'Toggle UDP MJPEG Streaming' to start/stop.",
		DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_FPS, DEFAULT_HOST, DEFAULT_PORT);

	return true;
}

void obs_module_unload(void)
{
	if (g_output) {
		if (obs_output_active(g_output)) {
			obs_output_stop(g_output);
		}
		obs_output_release(g_output);
		g_output = NULL;
	}

	obs_log(LOG_INFO, "UDP MJPEG Preview Streamer plugin unloaded");
}
