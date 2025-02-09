/**
 * @file all.hpp
 * @author Eliot Hall
 * @brief master header that includes all other ihsboost headers
 * @version 0.1
 * @date 2023-02-24
 *
 * @copyright Copyright (c) 2023
 *
 */
/**
 * @example message_example.cpp
 * @example PosixQreceiver.cpp
 * @example PosixQsender.cpp
 * @example bluetooth_client.cpp
 * @example bluetooth_server.cpp
 *
 * @example movement_example.cpp
 * @example encoder_example.cpp
 * @example pid_example.cpp
 * @example gyro_example.cpp
 * @example gyro_client_example.cpp
 *
 * @example threading_example.cpp
 *
 * @example create_extra_example.cpp
 *
 * @example json_loader.cpp
 * @example json_loader.hpp
 * @example bot-config.json
 * @example other-config.json
 * @example configure_ihsboost.cpp
 * @example ihsboost_config.json
 *
 * @example listener.cpp
 * @example listener.py
 * @example sender.py
 * @example movement_bindings.py
 * @example back_and_forth.py
 * @example back_and_forth.cpp
 */

#include "modules.hpp"

// controllers
#ifdef build_controllers
#include "controllers.hpp"
#endif

//  movement
#ifdef build_movement
#include "movement.hpp"
#endif

// threading
#ifdef build_threading
#include "threading.hpp"
#endif

// util
#ifdef build_util
#include "util.hpp"
#endif

// servos
#ifdef build_servos
#include "servos.hpp"
#endif

// communication
#ifdef build_communicate
#include "communicate.hpp"
#endif

// create_extra
#ifdef build_create_extra
#include "create_extra.hpp"
#endif
