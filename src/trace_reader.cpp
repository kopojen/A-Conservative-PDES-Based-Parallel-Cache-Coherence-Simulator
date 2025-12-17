/**
 * @file trace_reader.cpp
 * @brief Parses timestamped trace files and extracts memory access events.
 */

#include "trace_reader.h"

#include <charconv>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <string_view>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

constexpr bool IsSpace(char c) {
  return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

// Parse integer token, supporting decimal and 0x-prefixed hex.
// Also handles the common 'Ox' typo (letter O instead of zero).
uint64_t ParseUint64(std::string_view token) {
  if (token.empty()) {
    throw std::runtime_error("Failed to parse integer token: empty");
  }

  int base = 10;
  // Fix 'Ox' -> '0x' without allocating.
  if (token.size() >= 2 && (token[0] == 'O' || token[0] == 'o') &&
      (token[1] == 'x' || token[1] == 'X')) {
    base = 16;
    token.remove_prefix(2);
  } else if (token.size() >= 2 && token[0] == '0' &&
             (token[1] == 'x' || token[1] == 'X')) {
    base = 16;
    token.remove_prefix(2);
  }

  uint64_t value = 0;
  const char *begin = token.data();
  const char *end = token.data() + token.size();
  auto res = std::from_chars(begin, end, value, base);
  if (res.ec != std::errc() || res.ptr != end) {
    throw std::runtime_error("Failed to parse integer token '" +
                             std::string(token) + "'");
  }
  return value;
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

std::optional<TraceEvent> TraceReader::Peek() {
  if (has_peeked_) {
    return peeked_event_;
  }

  peeked_event_ = ReadNextEvent();
  has_peeked_ = true;
  return peeked_event_;
}

bool TraceReader::HasNext() { return Peek().has_value(); }

std::optional<TraceEvent> TraceReader::Next() {
  if (has_peeked_) {
    has_peeked_ = false;
    auto result = std::move(peeked_event_);
    peeked_event_.reset();
    return result;
  }

  return ReadNextEvent();
}

std::optional<TraceEvent> TraceReader::ReadNextEvent() {
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

    const char *line_ptr = data + line_start;
    const char *begin = line_ptr;
    const char *end = line_ptr + line_len;

    auto next_token = [&](std::string_view &out) -> bool {
      while (begin < end && IsSpace(*begin)) {
        ++begin;
      }
      if (begin >= end) {
        out = std::string_view{};
        return false;
      }
      const char *tok_begin = begin;
      while (begin < end && !IsSpace(*begin)) {
        ++begin;
      }
      out = std::string_view(tok_begin, static_cast<size_t>(begin - tok_begin));
      return true;
    };

    std::string_view t0, t1, t2, t3, t4;
    if (!next_token(t0)) {
      continue;
    }
    if (!next_token(t1) || !next_token(t2)) {
      throw std::runtime_error("Malformed trace line in " + path_.string() +
                               ": " + std::string(std::string_view(line_ptr, line_len)));
    }
    // Optional fields
    (void)next_token(t3);
    (void)next_token(t4);

    TraceEvent ev;
    // Avoid copying raw line on the fast path (it is only for debugging).
    ev.timestamp = ParseUint64(t0);

    // Detect format: check if 2nd token is operation (R/W) or core_id
    const bool second_is_op =
        (t1.size() == 1 &&
         (t1[0] == 'R' || t1[0] == 'r' || t1[0] == 'W' || t1[0] == 'w'));

    if (second_is_op) {
      // Format: timestamp op address [pc]
      ev.core_id = default_core_id_;
      ev.is_write = (t1[0] == 'W' || t1[0] == 'w');
      ev.address = ParseUint64(t2);
      if (!t3.empty()) {
        ev.pc = ParseUint64(t3);
      }
    } else {
      // Format: timestamp core_id op address [pc]
      file_embeds_core_ = true;
      ev.core_id = static_cast<uint32_t>(ParseUint64(t1));
      if (t2.size() != 1) {
        throw std::runtime_error("Malformed trace line in " + path_.string() +
                                 ": invalid op token");
      }
      ev.is_write = (t2[0] == 'W' || t2[0] == 'w');
      if (t3.empty()) {
        throw std::runtime_error("Malformed trace line in " + path_.string() +
                                 ": missing address field");
      }
      ev.address = ParseUint64(t3);
      if (!t4.empty()) {
        ev.pc = ParseUint64(t4);
      }
    }

    return ev;
  }

  return std::nullopt;
}
