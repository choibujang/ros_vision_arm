#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include <iostream>

// const char *metaDataTypes[] = { "TIMESTAMP",
//                                 "SENSOR_TIMESTAMP",
//                                 "FRAME_NUMBER",
//                                 "AUTO_EXPOSURE",
//                                 "EXPOSURE",
//                                 "GAIN",
//                                 "AUTO_WHITE_BALANCE",
//                                 "WHITE_BALANCE",
//                                 "BRIGHTNESS",
//                                 "CONTRAST",
//                                 "SATURATION",
//                                 "SHARPNESS",
//                                 "BACKLIGHT_COMPENSATION",
//                                 "HUE",
//                                 "GAMMA",
//                                 "POWER_LINE_FREQUENCY",
//                                 "LOW_LIGHT_COMPENSATION",
//                                 "MANUAL_WHITE_BALANCE",
//                                 "ACTUAL_FRAME_RATE",
//                                 "FRAME_RATE",
//                                 "AE_ROI_LEFT",
//                                 "AE_ROI_TOP",
//                                 "AE_ROI_RIGHT",
//                                 "AE_ROI_BOTTOM",
//                                 "EXPOSURE_PRIORITY",
//                                 "HDR_SEQUENCE_NAME",
//                                 "HDR_SEQUENCE_SIZE",
//                                 "HDR_SEQUENCE_INDEX",
//                                 "LASER_POWER",
//                                 "LASER_POWER_LEVEL",
//                                 "LASER_STATUS",
//                                 "GPIO_INPUT_DATA" };

int main() try {
    // Create a pipeline with default device
    ob::Pipeline pipe;

    // Configure which streams to enable or disable for the Pipeline by creating a Config
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_COLOR);

    // Start the pipeline with config
    pipe.start(config);
    auto currentProfile = pipe.getEnabledStreamProfileList()->getProfile(0)->as<ob::VideoStreamProfile>();
    // Create a window for rendering, and set the resolution of the window

    while(true) {
        // Wait for up to 100ms for a frameset in blocking mode.
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            continue;
        }

        // get color frame from frameset
        auto colorFrame = frameSet->colorFrame();
        if(colorFrame == nullptr) {
            continue;
        }

        uint8_t *rgb_data = (uint8_t *)colorFrame->data();

        std::cout << rgb_data[0] << " " << rgb_data[1] << " " << rgb_data[2] << std::endl;


        // Render frameset in the window, only color frames are rendered here.
    }

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}