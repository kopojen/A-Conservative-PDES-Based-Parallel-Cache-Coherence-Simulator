/**
 * @file trace_reader.cpp
 * @brief Parses timestamped trace files and extracts memory access events.
 */

#include "trace_reader.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
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

void TraceReader::MmapDeleter::operator()(const char* data) const {
  if (data && length > 0) {
    munmap(const_cast<char*>(data), length);
  }
}

TraceReader::TraceReader(const std::filesystem::path& path,
                         uint32_t default_core_id)
    : path_(path), default_core_id_(default_core_id) {
  fd_ = ::open(path_.c_str(), O_RDONLY);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open trace file " + path_.string() +
                             ": " + std::strerror(errno));
  }

  struct stat st;
  if (fstat(fd_, &st) != 0) {
    int err = errno;
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("Failed to stat trace file " + path_.string() +
                             ": " + std::strerror(err));
  }

  if (st.st_size == 0) {
    length_ = 0;
    return;
  }

  length_ = static_cast<size_t>(st.st_size);
  void* mapping = mmap(nullptr, length_, PROT_READ, MAP_PRIVATE, fd_, 0);
  if (mapping == MAP_FAILED) {
    int err = errno;
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("Failed to mmap trace file " + path_.string() +
                             ": " + std::strerror(err));
  }

  mapping_ = MappingPtr(static_cast<const char*>(mapping), MmapDeleter{length_});
}

TraceReader::~TraceReader() {
  mapping_.reset();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

std::optional<TraceEvent> TraceReader::Next() {
  if (length_ == 0 || offset_ >= length_ || !mapping_) {
    return std::nullopt;
  }

  while (offset_ < length_) {
    const char* data = mapping_.get();
    const size_t line_start = offset_;
    while (offset_ < length_) {
      char c = data[offset_];
      if (c == '\n' || c == '\r') {
        break;
      }
      offset_++;
    }

    size_t line_len = offset_ - line_start;
    if (offset_ < length_) {
      if (data[offset_] == '\r') {
        offset_++;
        if (offset_ < length_ && data[offset_] == '\n') {
          offset_++;
        }
      } else if (data[offset_] == '\n') {
        offset_++;
      }
    }

    if (line_len == 0) {
      continue;
    }

    std::string line(data + line_start, line_len);
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
