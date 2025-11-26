/**
 * @file trace_reader.h
 * @brief Streaming trace file parser. Supports formats with/without embedded core ID.
 */

#pragma once

#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>

#include "trace_event.h"

class TraceReader {
 public:
  TraceReader(const std::filesystem::path& path, uint32_t default_core_id);
  ~TraceReader();

  TraceReader(const TraceReader&) = delete;
  TraceReader& operator=(const TraceReader&) = delete;

  std::optional<TraceEvent> Next();

  uint32_t default_core_id() const { return default_core_id_; }
  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
  uint32_t default_core_id_{};
  bool file_embeds_core_{false};
  std::unique_ptr<std::ifstream> stream_;
};
