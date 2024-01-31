# send
echo 'ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", OWNER="user", GROUP="dialout", MODE="0777"' > /etc/udev/rules.d/80-icestick.rules

    // Kode untuk mengirim data (misalnya, ke mikrokontroller atau perangkat lain)
    const char* data_to_send = "Hello, FTDI!";
    int bytes_written = ftdi_write_data(&ctx, reinterpret_cast<const unsigned char*>(data_to_send), strlen(data_to_send));
    if (bytes_written >= 0) {
        printf("Successfully wrote %d bytes to FTDI\n", bytes_written);
    } else {
        fprintf(stderr, "Error writing data to FTDI: %s\n", ftdi_get_error_string(&ctx));
    }


const unsigned char constData[] = "Hello, FTDI!";
unsigned char* nonConstData = const_cast<unsigned char*>(constData);
