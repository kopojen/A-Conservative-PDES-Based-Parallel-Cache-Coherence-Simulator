/**
 * @file trace_reader.h
 * @brief Streaming trace file parser. Supports formats with/without embedded core ID.
 */

#pragma once

#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>

#include "trace_event.h"

class TraceReader {
public:
  TraceReader(const std::filesystem::path &path, uint32_t default_core_id);
  ~TraceReader();

  TraceReader(const TraceReader &) = delete;
  TraceReader &operator=(const TraceReader &) = delete;

  /// Read and consume the next event
  std::optional<TraceEvent> Next();

  /// Peek at the next event without consuming it
  std::optional<TraceEvent> Peek();

  /// Check if there are more events
  bool HasNext();

  uint32_t default_core_id() const { return default_core_id_; }
  const std::filesystem::path &path() const { return path_; }

private:
  /// Internal method to read the next event from file
  std::optional<TraceEvent> ReadNextEvent();

  struct MmapDeleter {
    size_t length{0};
    void operator()(const char *data) const;
  };
  using MappingPtr = std::unique_ptr<const char, MmapDeleter>;

  std::filesystem::path path_;
  uint32_t default_core_id_{};
  bool file_embeds_core_{false};
  int fd_{-1};
  MappingPtr mapping_{nullptr, MmapDeleter{0}};
  size_t length_{0};
  size_t offset_{0};

  /// Cached next event for Peek/HasNext
  std::optional<TraceEvent> peeked_event_;
  bool has_peeked_{false};
};
