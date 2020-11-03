/*****************************************************************************
  File: vector_concurrent_queue.h

  Version: 1.0
  Author: Carlos Faria <carlosfaria89@gmail.com>
  Maintainer: Carlos Faria <carlosfaria89@gmail.com>

  Copyright (C) 2017 Carlos André de Oliveira Faria. All rights reserved.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *****************************************************************************/

#ifndef V_REPEXT_VECTOR_CONCURRENT_QUEUE
#define V_REPEXT_VECTOR_CONCURRENT_QUEUE

#include <deque>
#include <thread>
#include <mutex>
#include <vector>

/*!
 * \brief The VectorConcurrentQueue class
 * Consider template if required other vector types
 */
template <typename T>
class VectorConcurrentQueue {
public:
    VectorConcurrentQueue();

    void push_front(const std::vector<T> &t);
    void push_back(const std::vector<T> &t);
    std::vector<T> back();
    void pop_back();
    bool empty();
    void clear();

private:
    std::deque< std::vector<T> > m_deque;
    std::mutex m_mutex;

};

template <typename T>
VectorConcurrentQueue<T>::VectorConcurrentQueue()
{}

template <typename T>
void VectorConcurrentQueue<T>::push_front(const std::vector<T> &t)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_deque.push_front(t);
}

template <typename T>
void VectorConcurrentQueue<T>::push_back(const std::vector<T> &t)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_deque.push_back(t);
}

template <typename T>
std::vector<T> VectorConcurrentQueue<T>::back()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_deque.back();
}

template <typename T>
void VectorConcurrentQueue<T>::pop_back()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_deque.pop_back();
}

template <typename T>
void VectorConcurrentQueue<T>::clear()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_deque.clear();
}

template <typename T>
bool VectorConcurrentQueue<T>::empty()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_deque.empty();
}

//class VectorConcurrentQueue {
//public:
//    VectorConcurrentQueue();

//    void push_front(const std::vector<double> &t);
//    void push_back(const std::vector<double> &t);
//    std::vector<double> back();
//    void pop_back();
//    bool empty();
//    void clear();

//private:
//    std::deque< std::vector<double> > m_deque;
//    std::mutex m_mutex;

//};

#endif //V_REPEXT_VECTOR_CONCURRENT_QUEUE
