# Included by the root project so platform libraries and compile definitions
# come from the runtime targets, not a machine-specific list of archives.
set(generated_manifest "${PS2X_GAME_GENERATED_DIR}/ps2_recompiled_sources.cmake")
if(NOT EXISTS "${generated_manifest}")
    message(FATAL_ERROR "Missing generated manifest: ${generated_manifest}. Run ps2_recomp first.")
endif()
include("${generated_manifest}")

# Keep continuation exports in separate trailing unity batches. Never glob
# stale exports: only the recompiler's active manifest owns this source list.
set(game_sources ${PS2_RECOMPILED_SOURCES})
set(continuation_sources ${game_sources})
list(FILTER game_sources EXCLUDE REGEX "/zz_FUN_[^/]*\\.cpp$")
list(FILTER continuation_sources INCLUDE REGEX "/zz_FUN_[^/]*\\.cpp$")
list(APPEND game_sources ${continuation_sources})
add_executable(ps2xGameRunner main.cpp ${game_sources})
target_link_libraries(ps2xGameRunner PRIVATE ps2_runtime)
target_include_directories(ps2xGameRunner PRIVATE "${PS2X_GAME_GENERATED_DIR}")
set_target_properties(ps2xGameRunner PROPERTIES
    CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON
    UNITY_BUILD ON UNITY_BUILD_BATCH_SIZE 256)

# Large generated units are expensive at -O3; keep research builds bounded.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ps2xGameRunner PRIVATE $<$<CONFIG:Release>:-O1>)
endif()

if(PS2X_GAME_OVERLAY_PACKAGE)
    set(overlay_manifest "${PS2X_GAME_OVERLAY_PACKAGE}/overlay_sources.cmake")
    if(NOT EXISTS "${overlay_manifest}")
        message(FATAL_ERROR "Missing overlay manifest: ${overlay_manifest}. No stale archive fallback.")
    endif()
    include("${overlay_manifest}")
    target_sources(ps2xGameRunner PRIVATE ${PS2_OVERLAY_SOURCES})
    # Overlay units namespace their own declarations; do not combine them
    # with the base export in the same unity translation unit.
    set_source_files_properties(${PS2_OVERLAY_SOURCES} PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
    target_compile_definitions(ps2xGameRunner PRIVATE PS2X_GAME_HAS_OVERLAY=1)
endif()

ps2x_stage_ffmpeg_runtime_dlls(ps2xGameRunner)
