add_executable(SOEM-test SOEM-test.c)
target_link_libraries(SOEM-test soem)
install(TARGETS SOEM-test DESTINATION bin)
