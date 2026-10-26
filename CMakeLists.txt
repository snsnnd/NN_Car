cmake_minimum_required(VERSION 3.16)
project(nncar LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(NNCAR_WARN -Wall -Wextra -Wpedantic -Werror)
set(NNCAR_FP -ffp-contract=off -fno-fast-math)

set(GEN_DIR ${CMAKE_BINARY_DIR}/generated)
set(PROFILE_JSON ${CMAKE_CURRENT_SOURCE_DIR}/configs/vehicle_default.json)
set(WEIGHTS_JSON ${CMAKE_CURRENT_SOURCE_DIR}/configs/motor_nn_weights.json)
set(LINE_WEIGHTS_JSON ${CMAKE_CURRENT_SOURCE_DIR}/configs/line_nn_weights.json)
file(MAKE_DIRECTORY ${GEN_DIR})

add_custom_command(
  OUTPUT ${GEN_DIR}/nncar_profile_generated.c
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tools/python/gen_profile.py
          ${PROFILE_JSON} ${GEN_DIR}/nncar_profile_generated.c
  DEPENDS ${PROFILE_JSON} ${CMAKE_CURRENT_SOURCE_DIR}/tools/python/gen_profile.py
  COMMENT "Generating nncar profile from ${PROFILE_JSON}"
  VERBATIM)

add_custom_command(
  OUTPUT ${GEN_DIR}/nncar_motor_nn_generated.c
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tools/python/gen_motor_nn.py
          ${WEIGHTS_JSON} ${GEN_DIR}/nncar_motor_nn_generated.c
          --profile ${PROFILE_JSON}
  DEPENDS ${WEIGHTS_JSON} ${PROFILE_JSON}
          ${CMAKE_CURRENT_SOURCE_DIR}/tools/python/gen_motor_nn.py
  COMMENT "Generating trained motor MLP from ${WEIGHTS_JSON}"
  VERBATIM)

add_custom_command(
  OUTPUT ${GEN_DIR}/nncar_line_nn_generated.c
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tools/python/gen_line_nn.py
          ${LINE_WEIGHTS_JSON} ${GEN_DIR}/nncar_line_nn_generated.c
  DEPENDS ${LINE_WEIGHTS_JSON} ${CMAKE_CURRENT_SOURCE_DIR}/tools/python/gen_line_nn.py
  COMMENT "Generating trained line MLP from ${LINE_WEIGHTS_JSON}"
  VERBATIM)

add_library(nncar_core STATIC
  firmware/core/src/calib.c
  firmware/core/src/gray.c
  firmware/core/src/line.c
  firmware/core/src/mixer.c
  firmware/core/src/motor.c
  firmware/core/src/nn.c
  firmware/core/src/safety.c
  firmware/core/src/sim.c
  firmware/core/src/stack.c
  firmware/core/src/velocity.c
  ${GEN_DIR}/nncar_profile_generated.c
  ${GEN_DIR}/nncar_motor_nn_generated.c
  ${GEN_DIR}/nncar_line_nn_generated.c)

target_include_directories(nncar_core PUBLIC firmware/core/include)
target_compile_options(nncar_core PRIVATE ${NNCAR_WARN} ${NNCAR_FP})
target_link_libraries(nncar_core PUBLIC m)

add_library(nncar_host STATIC
  firmware/host/src/log_csv.c
  firmware/host/src/sim_runner.c)

target_include_directories(nncar_host PUBLIC firmware/host/include)
target_compile_options(nncar_host PRIVATE ${NNCAR_WARN} ${NNCAR_FP})
target_link_libraries(nncar_host PUBLIC nncar_core)

add_executable(nncar_sim firmware/host/src/sim_main.c)
target_compile_options(nncar_sim PRIVATE ${NNCAR_WARN} ${NNCAR_FP})
target_link_libraries(nncar_sim PRIVATE nncar_host)

add_executable(nncar_replay firmware/host/src/replay_main.c)
target_compile_options(nncar_replay PRIVATE ${NNCAR_WARN} ${NNCAR_FP})
target_link_libraries(nncar_replay PRIVATE nncar_core)

add_executable(nncar_bench firmware/host/src/bench_main.c)
target_compile_options(nncar_bench PRIVATE ${NNCAR_WARN} ${NNCAR_FP})
target_link_libraries(nncar_bench PRIVATE nncar_core)

add_executable(nncar_baseline firmware/host/src/baseline_main.c)
target_compile_options(nncar_baseline PRIVATE ${NNCAR_WARN} ${NNCAR_FP})
target_link_libraries(nncar_baseline PRIVATE nncar_core)

add_executable(nncar_line_ab firmware/host/src/line_ab_main.c)
target_compile_options(nncar_line_ab PRIVATE ${NNCAR_WARN} ${NNCAR_FP})
target_link_libraries(nncar_line_ab PRIVATE nncar_core)

add_executable(nncar_calib_bench firmware/host/src/calib_bench_main.c)
target_compile_options(nncar_calib_bench PRIVATE ${NNCAR_WARN} ${NNCAR_FP})
target_link_libraries(nncar_calib_bench PRIVATE nncar_core)

enable_testing()

foreach(t calib gray line mixer motor nn safety sim)
  add_executable(test_${t} firmware/tests/test_${t}.c)
  target_include_directories(test_${t} PRIVATE firmware/tests)
  target_compile_options(test_${t} PRIVATE ${NNCAR_WARN} ${NNCAR_FP})
  target_link_libraries(test_${t} PRIVATE nncar_host)
  add_test(NAME ${t} COMMAND test_${t})
endforeach()

add_test(NAME python_tools
         COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tools/python/selftest.py)

add_test(NAME replay_golden
         COMMAND nncar_replay ${CMAKE_CURRENT_SOURCE_DIR}/golden/run_straight.csv)

add_test(NAME baseline_modes COMMAND nncar_baseline --check)

# On-board identification validated against the simulated plant's ground truth.
add_test(NAME calib_identify COMMAND nncar_calib_bench --check)

# LineFollower A/B harness smoke test. The epsilon gate itself is a milestone
# check (`nncar_line_ab --check`), not a regression test; see docs section 4.4.
add_test(NAME line_ab COMMAND nncar_line_ab)

set(BENCH_CSV ${CMAKE_BINARY_DIR}/bench.csv)
set(BENCH_TRUTH ${CMAKE_BINARY_DIR}/bench_truth.json)
set(BENCH_SCHEMA ${CMAKE_CURRENT_SOURCE_DIR}/tools/schema/bench_log.schema.json)

add_test(NAME bench_excitation
         COMMAND nncar_bench --out ${BENCH_CSV} --truth ${BENCH_TRUTH})

add_test(NAME bench_schema
         COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tools/python/validate_log.py
                 ${BENCH_CSV} --schema ${BENCH_SCHEMA} --quiet)
set_tests_properties(bench_schema PROPERTIES DEPENDS bench_excitation)

add_test(NAME bench_identify
         COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tools/python/identify_motor.py
                 ${BENCH_CSV} --truth ${BENCH_TRUTH}
                 --out ${CMAKE_BINARY_DIR}/motor_identified.json --quiet)
set_tests_properties(bench_identify PROPERTIES DEPENDS bench_excitation)
