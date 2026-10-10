/* memory.x — LPC1768 memory layout for cortex-m-rt linker script.
 *
 * FLASH: 512 KiB starting at 0x0000_0000
 * RAM:   32 KiB starting at 0x1000_0000
 *
 * (AHB SRAM banks at 0x2007C000/0x20080000 are left for DMA / EMAC buffers
 *  and are NOT included here.)
 */
MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 512K
    RAM   : ORIGIN = 0x10000000, LENGTH = 32K
}
