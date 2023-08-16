#ifndef TOOLS_H
#define TOOLS_H
#include "opencv2/core.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <coroutine>
#include <mutex>
#include <queue>

template <class E> class BlockingQueue {

  private:
    std::queue<E> _queue;
    int capacity = INT_MAX;
    std::condition_variable takeVariable, putVariable;
    mutable std::mutex lock;

  public:
    BlockingQueue();
    /**
     *
     * @param capacity 队列允许的最大值,在put时size()大于此值时,put方法将会wait
     */
    BlockingQueue(int capacity);
    /**
     * size() == 0 时会阻塞
     * @param e
     * @return -1失败, 0成功
     */
    int take(E &e);
    /**
     * size >= capacity时会阻塞
     * @param e
     * @return
     */
    int put(const E &e);
    bool empty() const;
    unsigned int size() const;
    void pop();
    E back();
    E front();
};

template <class E> BlockingQueue<E>::BlockingQueue() {}

template <class E>
BlockingQueue<E>::BlockingQueue(int capacity) : capacity(capacity) {}

template <class E> int BlockingQueue<E>::take(E &e) {
    std::unique_lock<std::mutex> uniqueLock(lock);
    while (_queue.empty()) {
        takeVariable.wait(uniqueLock);
    }
    if (_queue.empty())
        return -1;
    e = _queue.front();
    _queue.pop();
    putVariable.notify_one();
    return 0;
}

template <class E> int BlockingQueue<E>::put(const E &e) {
    std::unique_lock<std::mutex> uniqueLock(lock);
    while (_queue.size() >= capacity) {
        putVariable.wait(uniqueLock);
    }
    if (_queue.size() >= capacity)
        return -1;
    _queue.push(e);
    takeVariable.notify_one();
    return 0;
}

template <class E> bool BlockingQueue<E>::empty() const {
    std::lock_guard<std::mutex> lockGuard(lock);
    return _queue.empty();
}

template <class E> unsigned int BlockingQueue<E>::size() const {
    std::lock_guard<std::mutex> lockGuard(
        lock); // 利用变量作用域创建加锁,析构自动解锁
    return _queue.size();
}

template <class E> void BlockingQueue<E>::pop() {
    lock.lock();
    _queue.pop();
    lock.unlock();
}

template <class E> E BlockingQueue<E>::back() {
    std::lock_guard<std::mutex> lockGuard(lock);
    return _queue.back();
}

template <class E> E BlockingQueue<E>::front() {
    std::lock_guard<std::mutex> lockGuard(lock);
    return _queue.front();
}

struct MyCoroutine {
    struct promise_type;
    std::coroutine_handle<promise_type> m_handle;
    MyCoroutine(std::coroutine_handle<promise_type> handle) // (3)
        : m_handle(handle) {}
    MyCoroutine() {}
    ~MyCoroutine() {
        m_handle.destroy(); // (11)
    }

    struct promise_type {
        auto get_return_object() {
            return MyCoroutine{
                std::coroutine_handle<promise_type>::from_promise(*this)};
        }
        auto initial_suspend() { return std::suspend_never{}; }
        auto final_suspend() noexcept { return std::suspend_always{}; }
        void return_void() {}
        void unhandled_exception() { std::terminate(); }
    };

    bool resume() {
        if (!m_handle.done())
            m_handle.resume();
        return !m_handle.done();
    }
};

#endif /* ifndef TOOLS_H */
