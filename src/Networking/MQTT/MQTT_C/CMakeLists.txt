cmake_minimum_required(VERSION 3.5)
project(MQTT-C VERSION 1.1.2 LANGUAGES C)

# MQTT-C build options
option(MQTT_C_OpenSSL_SUPPORT "Build MQTT-C with OpenSSL support?" OFF)
option(MQTT_C_MbedTLS_SUPPORT "Build MQTT-C with mbed TLS support?" OFF)
option(MQTT_C_BearSSL_SUPPORT "Build MQTT-C with Bear SSL support?" OFF)
option(MQTT_C_EXAMPLES "Build MQTT-C examples?" ON)
option(MQTT_C_INSTALL_EXAMPLES "Install MQTT-C examples?" OFF)
option(MQTT_C_TESTS "Build MQTT-C tests?" OFF)

list (APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# MQTT-C library
add_library(mqttc STATIC
    src/mqtt_pal.c
    src/mqtt.c
)
target_include_directories(mqttc PUBLIC include)
target_link_libraries(mqttc PUBLIC 
    $<$<C_COMPILER_ID:MSVC>:ws2_32>
)

if(MQTT_C_BearSSL_SUPPORT)
    set(bearssl_root "$ENV{BEARSSL_ROOT}")

    set(bearssl_include "${bearssl_root}/include")
    find_file(bearinc NAMES bearssl.h PATHS "${bearssl_include}")

    if ("${bearinc}" STREQUAL "bearinc-NOTFOUND")
        MESSAGE(FATAL_ERROR "BEARSSL_ROOT does not contain include/bearssl.h")
    endif()

    set(bearssl_lib "${bearssl_root}/lib")
    find_library(bearlib NAMES libbearssl.a bearssl.lib PATHS "{bearssl_lib}")

    if("${bearlib}" STREQUAL "bearlib-NOTFOUND")
        MESSAGE(FATAL_ERROR "BEARSSL_ROOT does not contain lib with bearssl library")
    endif()

    get_filename_component(bearinc "${bearinc}" DIRECTORY)
    get_filename_component(bearlib2 "${bearlib}" DIRECTORY)

    target_include_directories(mqttc PUBLIC bearsslinc)
    target_link_libraries(mqttc INTERFACE "${bearlib}")
    target_compile_definitions(mqttc PUBLIC MQTT_USE_BEARSSL)
endif()

# Configure with OpenSSL support
if(MQTT_C_OpenSSL_SUPPORT)
    find_package(OpenSSL REQUIRED)
    target_link_libraries(mqttc INTERFACE OpenSSL::SSL)
    target_include_directories(mqttc PUBLIC ${OPENSSL_INCLUDE_DIR})
    target_compile_definitions(mqttc PUBLIC MQTT_USE_BIO)
endif()

# Configure with mbed TLS support
if(MQTT_C_MbedTLS_SUPPORT)
    find_package(MbedTLS REQUIRED)
    target_include_directories(mqttc PUBLIC ${MBEDTLS_INCLUDE_DIRS})
    target_link_libraries(mqttc INTERFACE ${MBEDTLS_LIBRARY})
    target_compile_definitions(mqttc PUBLIC MQTT_USE_MBEDTLS)
endif()

# Build examples
if(MQTT_C_EXAMPLES)
    find_package(Threads REQUIRED)

    if(MQTT_C_OpenSSL_SUPPORT)
        if(MSVC)
            add_executable(bio_publisher examples/bio_publisher_win.c)
            add_executable(openssl_publisher examples/openssl_publisher_win.c)
        else()
            add_executable(bio_publisher examples/bio_publisher.c)
            add_executable(openssl_publisher examples/openssl_publisher.c)
        endif()
		if(MQTT_C_INSTALL_EXAMPLES)
			install(TARGETS bio_publisher openssl_publisher)
		endif()
        target_link_libraries(bio_publisher Threads::Threads mqttc)
        target_link_libraries(openssl_publisher Threads::Threads mqttc)

	elseif(MQTT_C_MbedTLS_SUPPORT)
        add_executable(mbedtls_publisher examples/mbedtls_publisher.c)
        target_link_libraries(mbedtls_publisher Threads::Threads mqttc ${MBEDX509_LIBRARY} ${MBEDCRYPTO_LIBRARY})
		if(MQTT_C_INSTALL_EXAMPLES)
			install(TARGETS mbedtls_publisher)
		endif()

    elseif(MQTT_C_BearSSL_SUPPORT)
        add_executable(bearssl_publisher examples/bearssl_publisher.c)
        target_link_libraries(bearssl_publisher mqttc bearssl)
		if(MQTT_C_INSTALL_EXAMPLES)
			install(TARGETS bearssl_publisher)
		endif()
    else()
        add_executable(simple_publisher examples/simple_publisher.c)
        target_link_libraries(simple_publisher Threads::Threads mqttc)
		if(MQTT_C_INSTALL_EXAMPLES)
			install(TARGETS simple_publisher)
		endif()
    endif()

	# Always install subscriber targets
    add_executable(simple_subscriber examples/simple_subscriber.c)
    target_link_libraries(simple_subscriber Threads::Threads mqttc)
    add_executable(reconnect_subscriber examples/reconnect_subscriber.c)
    target_link_libraries(reconnect_subscriber Threads::Threads mqttc)
	if(MQTT_C_INSTALL_EXAMPLES)
		install(TARGETS simple_subscriber reconnect_subscriber)
	endif()
endif()

# Build tests
if(MQTT_C_TESTS)
    find_path(CMOCKA_INCLUDE_DIR cmocka.h)
    find_library(CMOCKA_LIBRARY cmocka)
    if((NOT CMOCKA_INCLUDE_DIR) OR (NOT CMOCKA_LIBRARY))
        message(FATAL_ERROR "Failed to find cmocka! Add cmocka's install prefix to CMAKE_PREFIX_PATH to resolve this error.")
    endif()

    add_executable(tests tests.c)
    target_link_libraries(tests ${CMOCKA_LIBRARY} mqttc)
    target_include_directories(tests PRIVATE ${CMOCKA_INCLUDE_DIR})
endif()

# Handle multi-lib linux systems correctly and allow custom installation locations.
if(UNIX)
	include(GNUInstallDirs)
	mark_as_advanced(CLEAR
		CMAKE_INSTALL_BINDIR
		CMAKE_INSTALL_LIBDIR
		CMAKE_INSTALL_INCLUDEDIR)
else()
	set(CMAKE_INSTALL_LIBDIR "lib")
	set(CMAKE_INSTALL_INCLUDEDIR "include")
endif()

# Install includes and library
install(TARGETS mqttc 
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
