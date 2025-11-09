set(SOURCE
   ${SOURCE}

   ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/class/hid/hid_device.c
   ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/common/tusb_fifo.c
   ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/device/usbd_control.c
   ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/device/usbd.c
   ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
   ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/tusb.c

   PARENT_SCOPE
)
