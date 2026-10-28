cmake_minimum_required(VERSION 3.13)

# Board: AudioAnalyzer_PC rev A (RP2040 + 2x PCM1865, 6ch 24-bit/48k USB Audio Class 2 microphone)
set(PICO_BOARD pico CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(audio_analyzer_fw C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(audio_analyzer_fw
    src/main.c
    src/pcm1865.c
    src/usb_descriptors.c
)

pico_generate_pio_header(audio_analyzer_fw ${CMAKE_CURRENT_LIST_DIR}/src/tdm_rx.pio)

target_include_directories(audio_analyzer_fw PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

target_compile_definitions(audio_analyzer_fw PRIVATE
    PICO_DEFAULT_UART=1
    PICO_PANIC_FUNCTION=crash_panic
    PICO_DEFAULT_UART_TX_PIN=4
    PICO_DEFAULT_UART_RX_PIN=5
    PICO_FLASH_SIZE_BYTES=4194304
    PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
    CFG_TUSB_MCU=OPT_MCU_RP2040
    CFG_TUSB_OS=OPT_OS_PICO
)

target_link_libraries(audio_analyzer_fw PRIVATE
    pico_stdlib
    pico_unique_id
    hardware_pio
    hardware_dma
    hardware_i2c
    hardware_irq
    hardware_watchdog
    tinyusb_device
)

# debug output on UART1 (GPIO4 TX / GPIO5 RX, 115200 8N1) - never on USB (USB is the audio device)
pico_enable_stdio_uart(audio_analyzer_fw 1)
pico_enable_stdio_usb(audio_analyzer_fw 0)

pico_add_extra_outputs(audio_analyzer_fw)
