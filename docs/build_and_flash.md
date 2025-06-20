# Building and Flashing

This guide explains how to compile and program the firmware for the dual-active-bridge converter.

## Open the Project

1. Launch **STM32CubeIDE**.
2. Select **File \> Open Projects from File System...** and choose `dab_stm32g474.ioc` in the repository root.
3. Once imported, generate the project sources with **Project \> Generate Code**.
4. Build the firmware using **Project \> Build Project**.

The linker scripts `STM32G474RETX_FLASH.ld` and `STM32G474RETX_RAM.ld` are provided for flash and RAM configurations.

## Flashing the MCU

You can program the target directly through STM32CubeIDE or by using `st-flash` from the command line:

```bash
st-flash write build/dab_stm32g474.bin 0x08000000
```

## Further Information

See the [project README](../README.md) for an overview of the repository structure and additional documentation links.
