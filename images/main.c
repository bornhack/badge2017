#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "./images.c"
#include "../display.c"

int main()
{
	const size_t imageCount = sizeof(images) / sizeof(image*);

	printf("static uint8_t frames[%i][1024] = {\n", imageCount);

  display dp;
	const size_t displayBytes = sizeof(dp.framebuf);

  for(size_t currentImage = 0; currentImage < imageCount; currentImage++) {
		display_clear(&dp);
		display_image(&dp, images[currentImage]);

		printf("  {");
		for(size_t displayByte = 0; displayByte < displayBytes; displayByte++){
			printf(" 0x%02X", dp.framebuf[displayByte]);

			if(displayByte < displayBytes - 1) {
				printf(",");
			}
		}
		printf("}");
		if(currentImage < imageCount - 1) {
			printf(",");
		}
		printf("\n");
	}
	printf("};\n");
}
