#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <atomic>
#include <libusb-1.0/libusb.h>

// --- Configuration Constants ---
// You MUST replace these with your device's actual Vendor ID and Product ID.
#define VENDOR_ID 0x0400
#define PRODUCT_ID 0xc55d
#define INTERFACE_NUMBER 0

// The bulk IN endpoint address is now a fixed constant as per your request
const int BULK_IN_ENDPOINT = 0x81;

// Image dimensions for the PPM file
const unsigned int WIDTH = 320;
const unsigned int HEIGHT = 240;
const std::string PPM_OUTPUT_FILENAME = "bulk_data.ppm";

// The total number of bulk transfers to perform
// Changed to 16 full transfers of 4800 bytes each.
#define NUM_FULL_TRANSFERS 16
#define FULL_TRANSFER_SIZE 4800
#define NUM_PENDING_TRANSFERS 4 // Number of transfers to have pending at all times

// Increased timeout to 200 milliseconds for more resilience
#define TRANSFER_TIMEOUT 200

// --- Global Variables for Asynchronous Transfers ---
// We now store all received data directly in an in-memory vector.
std::vector<unsigned char> raw_data_buffer;
std::atomic<size_t> total_bytes_received(0);
std::atomic<int> completed_transfers(0);
const int total_expected_bytes = NUM_FULL_TRANSFERS * FULL_TRANSFER_SIZE;
bool all_data_received = false;
bool transfer_failed = false; // New flag to indicate a failure

// --- Helper Functions ---
// Function to check libusb errors and exit on failure
void check_libusb_error(int result, const std::string& message) {
    if (result < 0) {
        std::cerr << "Error: " << message << " - " << libusb_error_name(result) << std::endl;
        exit(1);
    }
}

// Function to convert a hex string to a vector of unsigned chars
std::vector<unsigned char> hex_to_bytes(const std::string& hex) {
    std::vector<unsigned char> bytes;
    for (unsigned int i = 0; i < hex.length(); i += 2) {
        std::string byte_string = hex.substr(i, 2);
        unsigned char byte = (unsigned char)strtol(byte_string.c_str(), NULL, 16);
        bytes.push_back(byte);
    }
    return bytes;
}

// New function to convert the received bulk data to a PPM image file
void convert_to_ppm(const std::vector<unsigned char>& raw_data, const std::string& output_filename) {
    if (raw_data.empty()) {
        std::cerr << "Error: Raw data buffer is empty. Cannot create PPM." << std::endl;
        return;
    }


    std::vector<unsigned char> image_pixels;
    image_pixels.reserve(WIDTH * HEIGHT * 3); // Reserve space for RGB pixels

    for (unsigned int i = 0; i < raw_data.size() && i < WIDTH * HEIGHT; ++i) {
        unsigned char byte_value = raw_data[i];
        // Based on the user's previous request, here is a simple mapping from byte values to colors.
        if (byte_value == 0x00) {
            image_pixels.push_back(255); // Red
            image_pixels.push_back(255); // Green
            image_pixels.push_back(255); // Blue (White)
        } else if (byte_value == 0xC0) {
            image_pixels.push_back(0);
            image_pixels.push_back(0);
            image_pixels.push_back(0); // Black
        } else if (byte_value == 0x80) {
            image_pixels.push_back(128);
            image_pixels.push_back(128);
            image_pixels.push_back(128); // Grey
        } else {
            // Highlight unknown bytes with a specific color (e.g., red)
            image_pixels.push_back(255);
            image_pixels.push_back(0);
            image_pixels.push_back(0);
        }
    }

    // Pad the image with black if the received data is less than the expected size
    while (image_pixels.size() < WIDTH * HEIGHT * 3) {
        image_pixels.push_back(0);
        image_pixels.push_back(0);
        image_pixels.push_back(0);
    }

    // Write the PPM file to disk
    std::ofstream output_file(output_filename, std::ios::binary);
    if (!output_file.is_open()) {
        std::cerr << "Error: Could not open output file '" << output_filename << "' for writing." << std::endl;
        return;
    }

    // Write the PPM header
    output_file << "P6\n";
    output_file << WIDTH << " " << HEIGHT << "\n";
    output_file << "255\n"; // Max color value

    // Write the raw pixel data
    output_file.write(reinterpret_cast<const char*>(image_pixels.data()), image_pixels.size());

    output_file.close();
}

// --- Asynchronous Transfer Callback Function ---
void LIBUSB_CALL bulk_transfer_callback(libusb_transfer *transfer) {
    // A temporary pointer to the buffer from the completed transfer
    unsigned char* old_buffer = transfer->buffer;

    // Check the status of the completed transfer
    if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        size_t bytes_to_write = transfer->actual_length;

        // Append data directly to the in-memory vector
        raw_data_buffer.insert(raw_data_buffer.end(), old_buffer, old_buffer + bytes_to_write);

        total_bytes_received += bytes_to_write;
        completed_transfers++;

        // Check if all data has been received based on the total expected size
        if (completed_transfers >= NUM_FULL_TRANSFERS) {
            all_data_received = true;
        }

    } else {
        std::cerr << "Transfer failed with status: " << libusb_error_name(transfer->status) << std::endl;
        transfer_failed = true; // Set the failure flag
    }

    // Resubmit a new transfer only if we haven't received all the data and no failure has occurred
    if (!all_data_received && !transfer_failed) {
        // We know all transfers are the same size, so just resubmit with the same size
        size_t next_transfer_size = FULL_TRANSFER_SIZE;

        // Allocate a new transfer and buffer
        libusb_transfer *new_transfer = libusb_alloc_transfer(0);
        unsigned char* new_buffer = new unsigned char[next_transfer_size];

        if (new_transfer && new_buffer) {
            // Fill and submit the new transfer using the handle and endpoint from the original transfer
            libusb_fill_bulk_transfer(new_transfer, transfer->dev_handle, transfer->endpoint, new_buffer, next_transfer_size, bulk_transfer_callback, nullptr, TRANSFER_TIMEOUT);
            int r = libusb_submit_transfer(new_transfer);
            if (r < 0) {
                std::cerr << "Error resubmitting transfer: " << libusb_error_name(r) << std::endl;
                transfer_failed = true; // Stop the loop on resubmit failure
                libusb_free_transfer(new_transfer);
                delete[] new_buffer;
            }
        } else {
            std::cerr << "Failed to allocate memory for a new transfer. Stopping." << std::endl;
            transfer_failed = true;
        }
    }

    // Now that a new transfer has been submitted (or we are ending), it is safe to free the old one.
    libusb_free_transfer(transfer);
    delete[] old_buffer;
}

int main() {
    std::vector<std::string> data_strings = {
        "c001003a00000000",
        "c001004300000000",
        "c001004800000000",
        "c001004100000000",
        "c001004e00000000",
        "c001004e00000000",
        "c001004500000000",
        "c001004c00000000",
        "c001003100000000",
        "c001003a00000000",
        "c001005200000000",
        "c001004100000000",
        "c001004e00000000",
        "c001004700000000",
        "c001004500000000",
        "c001003a00000000",
        "c001003100000000",
        "c001003000000000",
        "c001003000000000", //kad
        "c001003000000000",
        "c001006d00000000",
        "c001005600000000",
        "c001000d00000000",
        "c001002a00000000",
        "c001004900000000",
        "c001004400000000",
        "c001004e00000000",
        "c001003f00000000",
        "c001000d00000000",
        "c000000000000001",
        "c000000100000028",
        //
        //"c001003a00000000",
       // "c001004b00000000",
     //   "c001004500000000",
       // "c001005900000000",
//        "c001003a00000000",
  //      "c001004800000000",
    //    "c001005f00000000",
      //  "c001005300000000",
//        "c001004300000000",
  //      "c001004100000000",
    //    "c001004c00000000",
      //  "c001004500000000",
//        "c001005f00000000",
  //      "c001004900000000",
    //    "c001004e00000000",
      //  "c001004300000000",
        //"c001000d00000000",
        //
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

    // --- Step 1: Initialize libusb and find the device ---
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

    // --- Step 2: Perform a series of Control Transfers until the trigger command ---
    for (const auto& str : data_strings) {
        unsigned char bmRequestType = hex_to_bytes(str.substr(0, 2))[0];
        unsigned char bRequest = hex_to_bytes(str.substr(2, 2))[0];
        uint16_t wValue = (uint16_t)strtol(str.substr(4, 4).c_str(), NULL, 16);
        uint16_t wIndex = (uint16_t)strtol(str.substr(8, 4).c_str(), NULL, 16);
        uint16_t wLength = (uint16_t)strtol(str.substr(12, 4).c_str(), NULL, 16);

        std::vector<unsigned char> transfer_data;
        if ((bmRequestType & 0x80) != 0) { // Read transfer
            transfer_data.resize(wLength);
        } else { // Write transfer
            transfer_data.resize(wLength);
        }

        int transferred_bytes = libusb_control_transfer(dev_handle, bmRequestType, bRequest, wValue, wIndex, transfer_data.data(), wLength, 1000);
    }


    // Reserve memory for the data
    raw_data_buffer.reserve(total_expected_bytes);

    // Submit the initial transfers
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

    // --- Step 4: Continue processing events until all data is received or a failure occurs ---
    struct timeval tv_long = {1, 0}; // 1-second timeout for the rest of the loop
    while (!all_data_received && !transfer_failed) {
        int result = libusb_handle_events_timeout_completed(ctx, &tv_long, nullptr);
        if (result < 0) {
            std::cerr << "Error in event loop: " << libusb_error_name(result) << std::endl;
            break;
        }
    }

    // --- Step 5: Cleanup resources and convert the data ---
    if (total_bytes_received > 0) {
        // Only attempt conversion if any data was received
        convert_to_ppm(raw_data_buffer, PPM_OUTPUT_FILENAME);
    } else {
        std::cerr << "No data was received to convert. PPM file will not be created." << std::endl;
    }

    libusb_release_interface(dev_handle, INTERFACE_NUMBER);
    libusb_close(dev_handle);
    libusb_exit(ctx);

    std::cout << "Program finished. Disconnected from device." << std::endl;

    return 0;
}
