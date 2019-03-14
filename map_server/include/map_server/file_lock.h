#pragma once

#include <fcntl.h>
#include <errno.h>
#include <sys/file.h>
#include <unistd.h>

#include <string>
#include <thread>
#include <stdexcept>

enum LOCK_TYPE { WRITE = LOCK_EX, READ = LOCK_SH };

template <LOCK_TYPE T>
class FileLock {
public:
    FileLock(const FileLock&) = delete;
    FileLock& operator=(FileLock) = delete;
    FileLock(FileLock&&) { _lock_fd = -1; }
    FileLock& operator=(FileLock&&) { _lock_fd = -1; }

    explicit FileLock(const std::string& file_path)
            : _lock_fd(-1) {
        const int FLAGS = O_RDWR | O_CREAT;
        const int MODE = (S_IRUSR + S_IWUSR) | (S_IRGRP + S_IWGRP) | (S_IROTH + S_IWOTH);

        // opens or creates a lock file with read and write permissions
        _lock_fd = ::open(file_path.c_str(), FLAGS, MODE);
        if (_lock_fd == -1) {
            const std::string error(std::string("Could not open lock file ") + file_path + std::string(". Error: ") +
                                    std::string(strerror(errno)));
            throw std::runtime_error(error);
        }

        if (::flock(_lock_fd, T) != 0) {
            const std::string error(std::string("Could not lock file ") + file_path + std::string(". Error: ") +
                                    std::string(strerror(errno)));
            throw std::runtime_error(error);
        }
    }

    ~FileLock() {
        // closing a file descriptor associated with a lock unlocks it
        ::close(_lock_fd);
    }

private:
    int _lock_fd;
};
