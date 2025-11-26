/**
 * @file trace_event.h
 * @brief Data structure for a single memory access event.
 *
 * Trace format: <timestamp> <op> <address> [pc]
 */

#pragma once

#include <cstdint>
#include <string>

struct TraceEvent {
  uint64_t timestamp{};      // Cycle when access occurs
  uint32_t core_id{};        // Processor core ID
  bool is_write{};           // True = write, False = read
  uint64_t address{};        // Memory address
  uint64_t pc{};             // Program counter (optional)
  std::string raw_line;      // Original line (debugging)
  
  const char* op_str() const { return is_write ? "W" : "R"; }
};
