add_executable(SOEM-test SOEM-test.c)
target_link_libraries(SOEM-test soem)
install(TARGETS SOEM-test DESTINATION bin)

add_executable(voice-coil voice-coil.c)
target_link_libraries(voice-coil soem m)
install(TARGETS voice-coil DESTINATION bin)
