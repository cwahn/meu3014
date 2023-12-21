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

add_executable(periodic_test periodic_test.cpp)
target_link_libraries(periodic_test
    PRIVATE
    efp
    efp_logger
    pigpio
    meu3014_hal
)

add_executable(record_and_play_pid record_and_play_pid.cpp)
target_link_libraries(record_and_play_pid
    PRIVATE
    efp
    efp_logger
    pigpio
    meu3014_hal
)

add_executable(record_and_play_smc record_and_play_smc.cpp)
target_link_libraries(record_and_play_smc
    PRIVATE
    efp
    efp_logger
    pigpio
    meu3014_hal
)