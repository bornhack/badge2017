#ifndef _DISPLAY_
#define _DISPLAY_

#include "font8x8.c"
#include "./images/image.h"

typedef struct {
	uint8_t tx;
	uint8_t ty;
	/* reset holds the pre-ample to send the display
	 * controller before sending the databits in the
	 * frame buffer. must be placed just before
	 * the framebuffer, so we can just send all bytes
	 * starting from reset and continuing into
	 * the frame buffer. */
	uint8_t reset[8];
	uint8_t framebuf[128 * 64 / 8];
} display;

static void
display_clear(display *dp)
{
	dp->tx = 0;
	dp->ty = 0;
	memset(dp->framebuf, 0, sizeof(dp->framebuf));
}

/* set a single pixel in the frame buffer */
static void
display_set(display *dp, unsigned int x, unsigned int y)
{
	unsigned int idx = 8*x + y/8;
	uint8_t mask = 1 << (y & 0x7U);

	dp->framebuf[idx] |= mask;
}

static void
display_image(display *dp, const image *img)
{
	const unsigned int pixel_count = img->width * img->height;

	for(unsigned int i = 0; i < pixel_count; i++) {
		char pixel = img->pixel_data[i * img->bytes_per_pixel];
		if(pixel != 0) {
			display_set(dp, dp->tx, dp->ty);
		}

		dp->tx++;
		if (dp->tx == img->width) {
			dp->tx = 0;
			dp->ty++;
		}
	}
}

static void
display_write(display *dp, const uint8_t *ptr, size_t len)
{
	for (; len; len--) {
		const uint8_t *glyph;
		const uint8_t *glyph_end;
		uint8_t *dest;
		unsigned int c = *ptr++;

		if (c == '\r') {
			dp->tx = 0;
			continue;
		}
		if (c == '\n') {
			dp->tx = 0;
			goto inc_ty;
		}

		if (c < 32 || c > 0x7F)
			c = 0x7F;

		c -= 32;

		glyph = &font8x8[c][0];
		glyph_end = glyph + 8;
		dest = &dp->framebuf[dp->ty + 8*8*dp->tx];
		while (glyph < glyph_end) {
			*dest |= *glyph++;
			dest += 8;
		}

		dp->tx++;
		if (dp->tx == 16) {
			dp->tx = 0;
inc_ty:
			dp->ty++;
			if (dp->ty == 8)
				dp->ty = 0;
		}
	}
}

static void
display_puts(display *dp, const char *str)
{
	display_write(dp, (const uint8_t *)str, strlen(str));
}

#endif
