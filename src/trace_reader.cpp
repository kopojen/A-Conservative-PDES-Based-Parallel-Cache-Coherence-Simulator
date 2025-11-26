/**
 * @file trace_reader.cpp
 * @brief Parses timestamped trace files and extracts memory access events.
 */

#include "trace_reader.h"

#include <charconv>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {

// Parse integer token, handling 'Ox' typo (letter O instead of 0)
uint64_t ParseUint64(const std::string& token) {
  std::string fixed_token = token;
  if (token.size() >= 2 && token[0] == 'O' && token[1] == 'x') {
    fixed_token[0] = '0';  // Fix typo: Ox → 0x
  }
  
  try {
    return std::stoull(fixed_token, nullptr, 0);
  } catch (const std::exception& exc) {
    throw std::runtime_error("Failed to parse integer token '" + token +
                             "': " + exc.what());
  }
}

}  // namespace

TraceReader::TraceReader(const std::filesystem::path& path,
                         uint32_t default_core_id)
    : path_(path), default_core_id_(default_core_id) {
  stream_ = std::make_unique<std::ifstream>(path);
  if (!stream_ || !stream_->is_open()) {
    throw std::runtime_error("Failed to open trace file: " + path.string());
  }
}

TraceReader::~TraceReader() = default;

std::optional<TraceEvent> TraceReader::Next() {
  if (!stream_ || !(*stream_)) {
    return std::nullopt;
  }

  std::string line;
  while (std::getline(*stream_, line)) {
    if (line.empty()) continue;

    // Tokenize line
    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string tok;
    while (iss >> tok) {
      tokens.push_back(tok);
    }

    if (tokens.size() < 3) {
      throw std::runtime_error("Malformed trace line in " + path_.string() +
                               ": " + line);
    }

    TraceEvent ev;
    ev.raw_line = line;
    ev.timestamp = ParseUint64(tokens[0]);

    // Detect format: check if 2nd token is operation (R/W) or core_id
    std::string second_token = tokens[1];
    bool second_is_op = (second_token == "R" || second_token == "r" ||
                         second_token == "W" || second_token == "w");

    if (second_is_op) {
      // Format: timestamp op address [pc]
      ev.core_id = default_core_id_;
      ev.is_write = (second_token == "W" || second_token == "w");
      ev.address = ParseUint64(tokens[2]);
      if (tokens.size() >= 4) {
        ev.pc = ParseUint64(tokens[3]);
      }
    } else {
      // Format: timestamp core_id op address [pc]
      file_embeds_core_ = true;
      ev.core_id = static_cast<uint32_t>(ParseUint64(tokens[1]));
      ev.is_write = (tokens[2] == "W" || tokens[2] == "w");
      ev.address = ParseUint64(tokens[3]);
      if (tokens.size() >= 5) {
        ev.pc = ParseUint64(tokens[4]);
      }
    }

    return ev;
  }

  return std::nullopt;
}
