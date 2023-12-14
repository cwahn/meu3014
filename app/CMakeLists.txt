add_executable(include_test include_test.cpp)
target_link_libraries(include_test
    PRIVATE
    efp
    efp_logger
    pigpio
)

add_executable(encoder_dev encoder_dev.cpp)
target_link_libraries(encoder_dev
    PRIVATE
    efp
    efp_logger
    pigpio
)

add_executable(motor_dev motor_dev.cpp)
target_link_libraries(motor_dev
    PRIVATE
    efp
    efp_logger
    pigpio
    meu3014_hal
)

add_executable(motor_test motor_test.cpp)
target_link_libraries(motor_test
    PRIVATE
    efp
    efp_logger
    pigpio
    meu3014_hal
)

add_executable(motor_sine_wave_dev motor_sine_wave_dev.cpp)
target_link_libraries(motor_sine_wave_dev
    PRIVATE
    efp
    efp_logger
    pigpio
    meu3014_hal
)

add_executable(voltage_velocity voltage_velocity.cpp)
target_link_libraries(voltage_velocity
    PRIVATE
    efp
    efp_logger
    pigpio
    meu3014_hal
)