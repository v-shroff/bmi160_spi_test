# ESP-IDF BMI160 + Mahony Filter Demo

An ESP-IDF demo that reads a BMI160 IMU over SPI and runs a Mahony filter to estimate orientation.

This was built for another project before switching sensors. There's an official ESP-IDF component for the BMI160 (I think), but I ran into issues getting it working, so I wrote this instead.

## Wiring

| BMI      | ESP GPIO       |
|----------|----------------|
| SCLK     | `GPIO_NUM_23`  |
| MOSI     | `GPIO_NUM_22`  |
| MISO     | `GPIO_NUM_19`  |
| CS       | `GPIO_NUM_21`  |

## Usage

1. Copy all files from this repo into the `main/` directory of your ESP-IDF project.
2. Update `idf_component_register` found in  `main/CMakeLists.txt` to include the sources below:

    ```
    idf_component_register(
        SRCS "gyro.cpp" "filter.cpp"
        INCLUDE_DIRS "."
    )
    ```

3. Build and flash as usual:
                    SRCS "motor_control/motors.cpp"
                    #SRCS "motor_control/motor_
    ```sh
    idf.py build flash monitor
    ```
