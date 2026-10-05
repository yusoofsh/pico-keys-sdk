#
# This file is part of the Pico Keys SDK distribution (https://github.com/polhenarejos/pico-keys-sdk).
# Copyright (c) 2022 Pol Henarejos.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, version 3.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#

cmake_minimum_required(VERSION 3.13)

project(picokeys_sdk_tests C)

set(CMAKE_C_STANDARD 11)

enable_testing()

set(OBJECT_CRYPTO_MBEDTLS_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/aes.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/ccm.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/chacha20.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/chachapoly.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/cipher.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/cipher_wrap.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/constant_time.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/des.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/gcm.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/hkdf.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/md.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/md5.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/platform_util.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/poly1305.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/ripemd160.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/sha1.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/sha256.c
    ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/library/sha512.c
)
add_library(object_crypto_mbedtls STATIC ${OBJECT_CRYPTO_MBEDTLS_SOURCES})
target_compile_definitions(object_crypto_mbedtls PUBLIC MBEDTLS_CONFIG_FILE="${CMAKE_CURRENT_LIST_DIR}/../config/mbedtls_config.h")
target_include_directories(object_crypto_mbedtls SYSTEM PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../third-party/mbedtls/include)

add_executable(object_store_test
    ${CMAKE_CURRENT_LIST_DIR}/../src/fs/object_store.c
    ${CMAKE_CURRENT_LIST_DIR}/../src/fs/object_store_txn.c
    ${CMAKE_CURRENT_LIST_DIR}/../src/fs/object_container.c
    ${CMAKE_CURRENT_LIST_DIR}/../src/fs/object_container_store.c
    ${CMAKE_CURRENT_LIST_DIR}/object_store_test.c
)
target_compile_definitions(object_store_test PRIVATE ENABLE_EMULATION)
target_include_directories(object_store_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../src ${CMAKE_CURRENT_LIST_DIR}/../src/fs)

if(MSVC)
    target_compile_options(object_store_test PRIVATE /W4 /WX)
else()
    target_compile_options(object_store_test PRIVATE -Wall -Wextra -Werror)
endif()

add_test(NAME object_store_test COMMAND object_store_test)

add_executable(object_crypto_provider_test
    ${CMAKE_CURRENT_LIST_DIR}/../src/fs/object_crypto_provider.c
    ${CMAKE_CURRENT_LIST_DIR}/object_crypto_provider_test.c
)
target_include_directories(object_crypto_provider_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../src ${CMAKE_CURRENT_LIST_DIR}/../src/fs)
target_link_libraries(object_crypto_provider_test PRIVATE object_crypto_mbedtls)

if(MSVC)
    target_compile_options(object_crypto_provider_test PRIVATE /W4 /WX)
else()
    target_compile_options(object_crypto_provider_test PRIVATE -Wall -Wextra -Werror)
endif()

add_test(NAME object_crypto_provider_test COMMAND object_crypto_provider_test)

add_executable(object_policy_test
    ${CMAKE_CURRENT_LIST_DIR}/../src/fs/object_policy.c
    ${CMAKE_CURRENT_LIST_DIR}/object_policy_test.c
)
target_include_directories(object_policy_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../src ${CMAKE_CURRENT_LIST_DIR}/../src/fs)
target_link_libraries(object_policy_test PRIVATE object_crypto_mbedtls)

if(MSVC)
    target_compile_options(object_policy_test PRIVATE /W4 /WX)
else()
    target_compile_options(object_policy_test PRIVATE -Wall -Wextra -Werror)
endif()

add_test(NAME object_policy_test COMMAND object_policy_test)

add_executable(flash_layout_test
    ${CMAKE_CURRENT_LIST_DIR}/../src/fs/flash_layout.c
    ${CMAKE_CURRENT_LIST_DIR}/flash_layout_test.c
)
target_include_directories(flash_layout_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../src ${CMAKE_CURRENT_LIST_DIR}/../src/fs)

if(MSVC)
    target_compile_options(flash_layout_test PRIVATE /W4 /WX)
else()
    target_compile_options(flash_layout_test PRIVATE -Wall -Wextra -Werror)
endif()

add_test(NAME flash_layout_test COMMAND flash_layout_test)

# --- low_flash boot-stage integration harness --------------------------------
# Compiles the UNMODIFIED production fs sources (src/fs/low_flash.c, flash.c,
# file.c, files.c and src/tlv.c) with the RP2040 code paths on the host:
# -DPICO_RP2040, 32-bit uintptr_t semantics (forced-in stubs/force32.h plus
# MOCK_ARM32_POINTERS) and a fake NOR flash mapped at the real XIP address
# (0x10000000) with MAP_FIXED_NOREPLACE. The mock implements NOR semantics and
# records every erase and program call with its offset, length, alignment,
# interrupt state and multicore lockout depth; the scenarios assert on those
# records. One executable per variant, because PICO_FLASH_SIZE_BYTES and
# PICO_FLASH_SIZE_LIMIT_BYTES are compile-time.

set(FLASH_HARNESS_DIR ${CMAKE_CURRENT_LIST_DIR}/flash_harness)
set(FLASH_HARNESS_PROD_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/../src/fs/low_flash.c
    ${CMAKE_CURRENT_LIST_DIR}/../src/fs/flash_layout.c
    ${CMAKE_CURRENT_LIST_DIR}/../src/fs/flash.c
    ${CMAKE_CURRENT_LIST_DIR}/../src/fs/file.c
    ${CMAKE_CURRENT_LIST_DIR}/../src/fs/files.c
    ${CMAKE_CURRENT_LIST_DIR}/../src/tlv.c
    ${FLASH_HARNESS_DIR}/low_flash_mock.c
)
set(FLASH_HARNESS_SCENARIOS ${FLASH_HARNESS_DIR}/low_flash_scenarios.c)

# The production flash_set_bounds body keeps its real pool math, aliased so the
# mock can record every bounds publication and then forward to it.
set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/../src/fs/flash.c
    PROPERTIES COMPILE_DEFINITIONS "flash_set_bounds=pico_flash_set_bounds")

function(add_low_flash_harness name)
    cmake_parse_arguments(HARNESS "" "" "DEFS;SOURCES" ${ARGN})
    if(NOT HARNESS_SOURCES)
        set(HARNESS_SOURCES ${FLASH_HARNESS_SCENARIOS})
    endif()
    add_executable(${name} ${FLASH_HARNESS_PROD_SOURCES} ${HARNESS_SOURCES})
    target_include_directories(${name} BEFORE PRIVATE
        ${FLASH_HARNESS_DIR}       # mock API
        ${FLASH_HARNESS_DIR}/stubs # stub pico-sdk headers shadow the real ones
        ${CMAKE_CURRENT_LIST_DIR}/../src
        ${CMAKE_CURRENT_LIST_DIR}/../src/fs
    )
    target_compile_definitions(${name} PRIVATE PICO_RP2040 MOCK_ARM32_POINTERS ${HARNESS_DEFS})
    if(MSVC)
        message(FATAL_ERROR "the low_flash harness needs mmap and GCC-style flags")
    else()
        target_compile_options(${name} PRIVATE
            -include ${FLASH_HARNESS_DIR}/stubs/force32.h
            -Wall -Wextra -Werror
            -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast
        )
    endif()
    add_test(NAME ${name} COMMAND ${name})
endfunction()

if(NOT MSVC)
    # The Pipico preset: 4 MiB board, 2 MiB cap. Full scenario suite.
    add_low_flash_harness(low_flash_capped4m_test DEFS PICO_FLASH_SIZE_BYTES=0x400000 PICO_FLASH_SIZE_LIMIT_BYTES=0x200000)
    # 2 MiB board with a 2 MiB cap; misaligned and too-small caps fail closed.
    add_low_flash_harness(low_flash_capped2m_test DEFS PICO_FLASH_SIZE_BYTES=0x200000 PICO_FLASH_SIZE_LIMIT_BYTES=0x200000)
    add_low_flash_harness(low_flash_cap_misaligned_test DEFS PICO_FLASH_SIZE_BYTES=0x400000 PICO_FLASH_SIZE_LIMIT_BYTES=0x200800)
    add_low_flash_harness(low_flash_cap_too_small_test DEFS PICO_FLASH_SIZE_BYTES=0x400000 PICO_FLASH_SIZE_LIMIT_BYTES=0x100000)
    # Uncapped boards keep the upstream layout apart from the build-time clamp.
    add_low_flash_harness(low_flash_nolimit2m_test DEFS PICO_FLASH_SIZE_BYTES=0x200000)
    add_low_flash_harness(low_flash_nolimit4m_test DEFS PICO_FLASH_SIZE_BYTES=0x400000)
    add_low_flash_harness(low_flash_nolimit16m_test DEFS PICO_FLASH_SIZE_BYTES=0x1000000)
    # Chips too small for the marker sector: marker handling skipped.
    add_low_flash_harness(low_flash_nolimit1m_test DEFS PICO_FLASH_SIZE_BYTES=0x100000)
    add_low_flash_harness(low_flash_nolimit512k_test DEFS PICO_FLASH_SIZE_BYTES=0x80000)
    # Storage-locked startup continuation: one fresh process per locking cause
    # (invalid JEDEC, cap > chip, foreign marker, readback mismatch), plus an
    # unlocked scan control. The Pipico preset geometry.
    add_low_flash_harness(low_flash_locked_boot_test
        SOURCES ${FLASH_HARNESS_DIR}/low_flash_locked_boot.c
        DEFS PICO_FLASH_SIZE_BYTES=0x400000 PICO_FLASH_SIZE_LIMIT_BYTES=0x200000)
endif()
