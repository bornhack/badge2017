__bootloader_size = DEFINED(__bootloader_size) ? __bootloader_size : 0;
__flash_size = DEFINED(__flash_size) ? __flash_size : 64K;

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000 + __bootloader_size, LENGTH = __flash_size - __bootloader_size
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 8K
}

INCLUDE "layout.ld"
