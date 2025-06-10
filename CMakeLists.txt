cmake_minimum_required(VERSION 3.28)
project(Arduino_Helitocoper)

set(CMAKE_CXX_STANDARD 14)

include_directories(.pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/examples/PS4Data)
include_directories(.pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/examples/PS4ReceiveData)
include_directories(.pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/examples/PS4SendData)
include_directories(.pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/examples/PS4ViewIncomingBits)
include_directories(.pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/osi)
include_directories(.pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/stack)

add_executable(Arduino_Helitocoper
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/examples/PS4Data/PS4Data.ino
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/examples/PS4ReceiveData/PS4ReceiveData.ino
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/examples/PS4SendData/PS4SendData.ino
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/examples/PS4ViewIncomingBits/PS4ViewIncomingBits.ino
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/osi/allocator.h
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/stack/bt_types.h
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/stack/btm_api.h
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/stack/gap_api.h
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/stack/hcidefs.h
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/stack/l2c_api.h
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/stack/l2cdefs.h
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/ps4.c
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/ps4.h
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/ps4_int.h
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/ps4_l2cap.c
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/ps4_parser.c
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/ps4_spp.c
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/PS4Controller.cpp
        .pio/libdeps/adafruit_feather_esp32s3/PS4_Controller_Host/src/PS4Controller.h
        src/main.cpp)
