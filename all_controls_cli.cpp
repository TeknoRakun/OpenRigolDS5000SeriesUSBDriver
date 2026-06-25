#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <atomic>
#include <libusb-1.0/libusb.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

void set_non_blocking_input() {
    termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

void restore_terminal_settings() {
    termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

#define VENDOR_ID 0x0400
#define PRODUCT_ID 0xc55d
#define INTERFACE_NUMBER 0

const int BULK_IN_ENDPOINT = 0x81;

const unsigned int WIDTH = 320;
const unsigned int HEIGHT = 240;
const std::string PPM_OUTPUT_FILENAME = "bulk_data.ppm";

#define NUM_FULL_TRANSFERS 2
#define FULL_TRANSFER_SIZE 38400
#define NUM_PENDING_TRANSFERS 1

#define TRANSFER_TIMEOUT 200

std::vector<unsigned char> raw_data_buffer;
std::atomic<size_t> total_bytes_received(0);
std::atomic<int> completed_transfers(0);
const int total_expected_bytes = NUM_FULL_TRANSFERS * FULL_TRANSFER_SIZE;
bool all_data_received = false;
bool transfer_failed = false;

void check_libusb_error(int result, const std::string& message) {
    if (result < 0) {
        std::cerr << "Error: " << message << " - " << libusb_error_name(result) << std::endl;
        exit(1);
    }
}

std::vector<unsigned char> hex_to_bytes(const std::string& hex) {
    std::vector<unsigned char> bytes;
    for (unsigned int i = 0; i < hex.length(); i += 2) {
        std::string byte_string = hex.substr(i, 2);
        unsigned char byte = (unsigned char)strtol(byte_string.c_str(), NULL, 16);
        bytes.push_back(byte);
    }
    return bytes;
}

void cont_tr(libusb_device_handle *my_handle, const std::vector<std::string>& string_list){
    for (const auto& str : string_list) {
        unsigned char bmRequestType = hex_to_bytes(str.substr(0, 2))[0];
        unsigned char bRequest = hex_to_bytes(str.substr(2, 2))[0];
        uint16_t wValue = (uint16_t)strtol(str.substr(4, 4).c_str(), NULL, 16);
        uint16_t wIndex = (uint16_t)strtol(str.substr(8, 4).c_str(), NULL, 16);
        uint16_t wLength = (uint16_t)strtol(str.substr(12, 4).c_str(), NULL, 16);

        std::vector<unsigned char> transfer_data;
        if ((bmRequestType & 0x80) != 0) {
            transfer_data.resize(wLength);
        }
        else {
            transfer_data.resize(wLength);
        }
        int transferred_bytes = libusb_control_transfer(my_handle, bmRequestType, bRequest, wValue, wIndex, transfer_data.data(), wLength, 1000);
    }
}

void convert_to_ppm(const std::vector<unsigned char>& raw_data, const std::string& output_filename) {
    if (raw_data.empty()) {
        std::cerr << "Error: Raw data buffer is empty. Cannot create PPM." << std::endl;
        return;
    }

    std::vector<unsigned char> image_pixels;
    image_pixels.reserve(WIDTH * HEIGHT * 3);

    for (unsigned int i = 0; i < raw_data.size() && i < WIDTH * HEIGHT; ++i) {
        unsigned char byte_value = raw_data[i];
        if (byte_value == 0x00) {
            image_pixels.push_back(255); image_pixels.push_back(255); image_pixels.push_back(255);
        } else if (byte_value == 0xC0) {
            image_pixels.push_back(0); image_pixels.push_back(0); image_pixels.push_back(0);
        } else if (byte_value == 0x80) {
            image_pixels.push_back(128); image_pixels.push_back(128); image_pixels.push_back(128);
        } else {
            image_pixels.push_back(255); image_pixels.push_back(0); image_pixels.push_back(0);
        }
    }

    while (image_pixels.size() < WIDTH * HEIGHT * 3) {
        image_pixels.push_back(0); image_pixels.push_back(0); image_pixels.push_back(0);
    }

    std::ofstream output_file(output_filename, std::ios::binary);
    if (!output_file.is_open()) {
        std::cerr << "Error: Could not open output file '" << output_filename << "' for writing." << std::endl;
        return;
    }

    output_file << "P6\n" << WIDTH << " " << HEIGHT << "\n" << "255\n";
    output_file.write(reinterpret_cast<const char*>(image_pixels.data()), image_pixels.size());
    output_file.close();
}

void LIBUSB_CALL bulk_transfer_callback(libusb_transfer *transfer) {
    unsigned char* old_buffer = transfer->buffer;

    if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        size_t bytes_to_write = transfer->actual_length;
        raw_data_buffer.insert(raw_data_buffer.end(), old_buffer, old_buffer + bytes_to_write);
        total_bytes_received += bytes_to_write;
        completed_transfers++;

        if (completed_transfers >= NUM_FULL_TRANSFERS) {
            all_data_received = true;
        }
    } else {
        std::cerr << "Transfer failed with status: " << libusb_error_name(transfer->status) << std::endl;
        transfer_failed = true;
    }

    if (!all_data_received && !transfer_failed) {
        size_t next_transfer_size = FULL_TRANSFER_SIZE;
        libusb_transfer *new_transfer = libusb_alloc_transfer(0);
        unsigned char* new_buffer = new unsigned char[next_transfer_size];
        if (new_transfer && new_buffer) {
            libusb_fill_bulk_transfer(new_transfer, transfer->dev_handle, transfer->endpoint, new_buffer, next_transfer_size, bulk_transfer_callback, nullptr, TRANSFER_TIMEOUT);
            int r = libusb_submit_transfer(new_transfer);
            if (r < 0) {
                std::cerr << "Error resubmitting transfer: " << libusb_error_name(r) << std::endl;
                transfer_failed = true;
                libusb_free_transfer(new_transfer);
                delete[] new_buffer;
            }
        } else {
            std::cerr << "Failed to allocate memory for a new transfer. Stopping." << std::endl;
            transfer_failed = true;
        }
    }
    libusb_free_transfer(transfer);
    delete[] old_buffer;
}

void start_bulk_transfer_sequence(libusb_device_handle* dev_handle, const std::vector<std::string>& bulk_start_cmd) {
    cont_tr(dev_handle, bulk_start_cmd);
    raw_data_buffer.reserve(total_expected_bytes);

    for (int i = 0; i < NUM_PENDING_TRANSFERS; ++i) {
        libusb_transfer *transfer = libusb_alloc_transfer(0);
        unsigned char* buffer = new unsigned char[FULL_TRANSFER_SIZE];
        libusb_fill_bulk_transfer(transfer, dev_handle, BULK_IN_ENDPOINT, buffer, FULL_TRANSFER_SIZE, bulk_transfer_callback, nullptr, TRANSFER_TIMEOUT);
        int r = libusb_submit_transfer(transfer);
        if (r < 0) {
            std::cerr << "Failed to submit transfer " << i << ": " << libusb_error_name(r) << std::endl;
            libusb_free_transfer(transfer);
            delete[] buffer;
            transfer_failed = true;
            break;
        }
    }
}

int main() {
    set_non_blocking_input();

    std::string input_string = "";
    bool running = true;

    std::vector<std::string> init_strings ={
        "c001004100000000",
        "c001005900000000",
        "c001003a00000000",
        "c001004d00000000",
        "c001004500000000",
        "c001004e00000000",
        "c001005500000000",
        "c001002000000000",
        "c001005300000000",
        "c001005400000000",
        "c001004100000000",
        "c001005400000000",
        "c001005500000000",
        "c001005300000000",
        "c001003a00000000",
        "c001004f00000000",
        "c001004600000000",
        "c001004600000000",
        "c001000d00000000",
        "c001003a00000000",
        "c001004b00000000",
        "c001004500000000",
        "c001005900000000",
        "c001003a00000000",
        "c001004c00000000",
        "c001004f00000000",
        "c001004e00000000",
        "c001004100000000",
        "c001004200000000",
        "c001004c00000000",
        "c001004500000000",
        "c001000d00000000",
        "c001002a00000000",
        "c001004900000000",
        "c001004400000000",
        "c001004e00000000",
        "c001003f00000000",
        "c001000d00000000",
        "c000000000000001",
        "c000000100000028"
    };

    std::vector<std::string> zi_string={
        "c001003a00000000",
        "c001004b00000000",
        "c001004500000000",
        "c001005900000000",
        "c001003a00000000",
        "c001004800000000",
        "c001005f00000000",
        "c001005300000000",
        "c001004300000000",
        "c001004100000000",
        "c001004c00000000",
        "c001004500000000",
        "c001005f00000000",
        "c001004400000000",
        "c001004500000000",
        "c001004300000000",
        "c001000d00000000"
    };
    std::vector<std::string> zg_string={
        "c001003a00000000",
        "c001004b00000000",
        "c001004500000000",
        "c001005900000000",
        "c001003a00000000",
        "c001004800000000",
        "c001005f00000000",
        "c001005300000000",
        "c001004300000000",
        "c001004100000000",
        "c001004c00000000",
        "c001004500000000",
        "c001005f00000000",
        "c001004900000000",
        "c001004e00000000",
        "c001004300000000",
        "c001000d00000000"
    };
    std::vector<std::string> bulk_start = {
        "c001002a00000000",
        "c001004900000000",
        "c001004400000000",
        "c001004e00000000",
        "c001003f00000000",
        "c001000d00000000",
        "c000000000000001",
        "c000000100000028",
        "c001003a00000000",
        "c001004800000000",
        "c001004100000000",
        "c001005200000000",
        "c001004400000000",
        "c001005f00000000",
        "c001004300000000",
        "c001004f00000000",
        "c001005000000000",
        "c001005900000000",
        "c001000d00000000",
        "c008000000500000",
        "c0092c0000010000",
        "c007000000090000"
    };

    libusb_device **devs;
    libusb_device_handle *dev_handle = NULL;
    libusb_context *ctx = NULL;
    int r;
    ssize_t cnt;

    r = libusb_init(&ctx);
    check_libusb_error(r, "libusb initialization failed");

    cnt = libusb_get_device_list(ctx, &devs);
    if (cnt < 0) {
        check_libusb_error(cnt, "libusb_get_device_list failed");
    }

    libusb_device *dev = NULL;
    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device_descriptor desc;
        r = libusb_get_device_descriptor(devs[i], &desc);
        if (r < 0) continue;
        if (desc.idVendor == VENDOR_ID && desc.idProduct == PRODUCT_ID) {
            dev = devs[i];
            break;
        }
    }

    if (dev == NULL) {
        std::cerr << "Error: Device (VID=" << std::hex << VENDOR_ID << ", PID=" << PRODUCT_ID << ") not found." << std::endl;
        libusb_free_device_list(devs, 1);
        libusb_exit(ctx);
        return 1;
    }

    r = libusb_open(dev, &dev_handle);
    libusb_free_device_list(devs, 1);
    check_libusb_error(r, "Unable to open device");

    if (libusb_kernel_driver_active(dev_handle, INTERFACE_NUMBER) == 1) {
        std::cout << "Kernel driver attached. Detaching..." << std::endl;
        r = libusb_detach_kernel_driver(dev_handle, INTERFACE_NUMBER);
        check_libusb_error(r, "Unable to detach kernel driver");
    }

    r = libusb_claim_interface(dev_handle, INTERFACE_NUMBER);
    check_libusb_error(r, "Cannot claim interface");

    std::cout << "Successfully connected to device." << std::endl;
    libusb_reset_device(dev_handle);

    cont_tr(dev_handle, init_strings);

    start_bulk_transfer_sequence(dev_handle, bulk_start);


    int turn=0;
    while (running) {
        struct timeval tv = {0, 10000};
        int result = libusb_handle_events_timeout(ctx, &tv);
        if (result < 0) {
            std::cerr << "Error in event loop: " << libusb_error_name(result) << std::endl;
            break;
        }

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        timeval tv_input = {0, 0};

        if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv_input) == 1) {
            int ch = getchar();
            if (ch == '\n' || ch == '\r') {
                if (!input_string.empty()) {
                    if (input_string == "q") {
                        running = false;
                        std::cout << ""<<std::endl;
                    }
                    else if (input_string == "zi") {
                        cont_tr(dev_handle, zi_string);
                        std::cout << ""<<std::endl;
                    }
                    else if (input_string == "zg") {
                        cont_tr(dev_handle, zg_string);
                        std::cout << ""<<std::endl;
                    }
                    else if (input_string=="vi"){

                    }

                    input_string.clear();
                }
            } else {
                input_string += (char)ch;
                std::cout << (char)ch << std::flush;
            }
        }

        if (all_data_received) {
            if (total_bytes_received > 0) {
                convert_to_ppm(raw_data_buffer, PPM_OUTPUT_FILENAME);
            } else {
                std::cerr << "\nNo data was received to convert. PPM file will not be created." << std::endl;
            }


            all_data_received = false;
            total_bytes_received = 0;
            completed_transfers = 0;
            raw_data_buffer.clear();

            start_bulk_transfer_sequence(dev_handle, bulk_start);
        }
    }

    restore_terminal_settings();
    libusb_release_interface(dev_handle, INTERFACE_NUMBER);
    libusb_close(dev_handle);
    libusb_exit(ctx);

    std::cout << "Program finished. Disconnected from device." << std::endl;

    return 0;
}
