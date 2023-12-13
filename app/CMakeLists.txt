add_executable(include_test include_test.cpp)
target_link_libraries(include_test 
    PRIVATE
    efp
    efp_logger 
    pigpio
)

add_executable(encoder_test encoder_test.cpp)
target_link_libraries(encoder_test 
    PRIVATE
    efp
    efp_logger 
    pigpio
)