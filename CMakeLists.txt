add_executable(SOEM-test SOEM-test.c)
target_link_libraries(SOEM-test soem)
install(TARGETS SOEM-test DESTINATION bin)

add_executable(voice-coil
   voice-coil.c
   fieldbus.c
   amc_config.c
   cia402.c
   control_loop.c
   diagnostics.c
   logging.c)
target_link_libraries(voice-coil soem m)
install(TARGETS voice-coil DESTINATION bin)
