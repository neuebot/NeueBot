#include "string_concurrent_queue.h"

StringConcurrentQueue::StringConcurrentQueue()
{
}

int StringConcurrentQueue::size() {
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_deque.size();
}

bool StringConcurrentQueue::empty() {
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_deque.empty();
}

void StringConcurrentQueue::clear()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_deque.clear();
}

void StringConcurrentQueue::push_back(const std::string &t)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_deque.push_back(t);
}

std::string StringConcurrentQueue::back()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_deque.back();
}

void StringConcurrentQueue::pop_back()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_deque.pop_back();
}


