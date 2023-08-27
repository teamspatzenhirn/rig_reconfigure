/**
 * @file   queue.hpp
 * @author Dominik Authaler
 * @date   22.01.2023
 *
 * Utility class providing a thread safe queue.
 */

#ifndef RIG_RECONFIGURE_QUEUE_HPP
#define RIG_RECONFIGURE_QUEUE_HPP

#include <mutex>
#include <queue>
#include <optional>
#include <condition_variable>

template <typename T>
class Queue {
  public:
    template<typename F>
    void push(F&& data) {
        std::lock_guard<std::mutex> guard(mutex);
        queue.push_back(std::forward<F>(data));
        cv.notify_one();
    }

    T pop() {
        std::unique_lock<std::mutex> lock(mutex);
        while (queue.empty()) {
            cv.wait(lock);
        }
        T data = std::move(queue.front());
        queue.pop_front();

        return data;
    }

    std::optional<T> try_pop() {
        std::unique_lock<std::mutex> lock(mutex);

        if (queue.empty()) {
            return std::nullopt;
        }

        T data = std::move(queue.front());
        queue.pop_front();

        return data;
    }

    size_t length() {
        std::lock_guard<std::mutex> lock(mutex);
        return queue.size();
    }

  private:
    std::deque<T> queue;
    std::mutex mutex;
    std::condition_variable cv;
};

#endif // RIG_RECONFIGURE_QUEUE_HPP
