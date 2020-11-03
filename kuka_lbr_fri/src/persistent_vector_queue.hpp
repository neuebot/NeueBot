/*****************************************************************************
  File: persistent_vector_queue.hpp

  Version: 1.0
  Author: Carlos Faria <carlosfaria89@gmail.com>
  Maintainer: Carlos Faria <carlosfaria89@gmail.com>

  Copyright (C) 2018 Carlos André de Oliveira Faria. All rights reserved.

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

#ifndef PERSISTENT_VECTOR_QUEUE_HPP
#define PERSISTENT_VECTOR_QUEUE_HPP

#include <deque>
#include <thread>
#include <mutex>
#include <vector>

/*!
 * \brief The PersistentVectorQueue class
 * Holds a thread-safe LIFO queue object that always keeps the last element.
 * Produced objects are queued on the back.
 * Objects are consumed from the back.
 * Whenever the queue is consumed, the queue is cleared except for the back element.
 * The queue can always be consumed even if no new element arrives.
 */
template <typename T>
class PersistentVectorQueue {
public:
    PersistentVectorQueue();

    /*!
     * \brief produce Pushes a new vector<T> to the end of the queue.
     * \param t Vector<T> to be pushed to the queue.
     */
    void produce(std::vector<T> &t);

    /*!
     * \brief produce Pushes a const new vector<T> reference to the end of the queue.
     * \param t Vector<T> to be pushed to the queue.
     */
    void produce(const std::vector<T> &t);

    /*!
     * \brief consume Returns a reference of the vector<T> at the end of the queue.
     * Make sure the queue is not empty before consuming.
     * \return Reference of vector<T> at the end of the queue.
     */
    std::vector<T> consume();

    /*!
     * \brief consume Returns a const reference of the vector<T> at the end of the queue.
     * Make sure the queue is not empty before consuming.
     * \return Reference of vector<T> at the end of the queue.
     */
    const std::vector<T> consume() const;

    /*!
     * \brief consume_one Returns a reference of the vector<T> at the top of the queue.
     *
     * Make sure the queue is not empty before consuming.
     * \return Reference of vector<T> at the top of the queue.
     */
    std::vector<T> consume_one();

    /*!
     * \brief consume_one Returns a const reference of the vector<T> at the top of the queue.
     *
     * Make sure the queue is not empty before consuming.
     * \return Reference of vector<T> at the top of the queue.
     */
    const std::vector<T> consume_one() const;

    /*!
     * \brief refresh Similar to produce(), but clears the rest of the queue first.
     * Clears queue and then adds the new value.
     * This will be useful to refresh the robot commanded position whenever the modes are switched.
     * When the robot is moved in alternative modes other than FRI, it maintains the last commanded position
     * nonetheless. If we go back to FRI commanded position, the first thing he will try to do is to move
     * back to that stored last command, which will move the robot rapidly.
     * \param t Vector<T> to be pushed to the queue after clearing it.
     */
    void refresh(std::vector<T> &t);

    /*!
     * \brief refresh Similar to produce(), but clears the rest of the queue first.
     * Clears queue and then adds the new value.
     * This will be useful to refresh the robot commanded position whenever the modes are switched.
     * When the robot is moved in alternative modes other than FRI, it maintains the last commanded position
     * nonetheless. If we go back to FRI commanded position, the first thing he will try to do is to move
     * back to that stored last command, which will move the robot rapidly.
     * \param t const Vector<T> to be pushed to the queue after clearing it.
     */
    void refresh(const std::vector<T> &t) const;

    /*!
     * \brief empty Checks whether the queue is empty.
     * \return True if queue empty, false otherwise.
     */
    bool empty();

    /*!
     * \brief clear Clear the queue.
     */
    void clear();

private:
    std::deque< std::vector<T> > m_dq;
    std::mutex m_mutex;

};

template <typename T>
PersistentVectorQueue<T>::PersistentVectorQueue()
{}

template <typename T>
void PersistentVectorQueue<T>::produce(std::vector<T> &t)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_dq.push_back(t);
}

template <typename T>
void PersistentVectorQueue<T>::produce(const std::vector<T> &t)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_dq.push_back(t);
}

template <typename T>
std::vector<T> PersistentVectorQueue<T>::consume()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_dq.erase(m_dq.begin(), m_dq.end()-1);
    return m_dq.back();
}

template <typename T>
const std::vector<T> PersistentVectorQueue<T>::consume() const
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_dq.erase(m_dq.begin(), m_dq.end()-1);
    return m_dq.back();
}

template <typename T>
std::vector<T> PersistentVectorQueue<T>::consume_one()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    std::vector<T> res = m_dq.front();
    if(m_dq.size() > 1)
        m_dq.pop_front();
    return res;
}

template <typename T>
const std::vector<T> PersistentVectorQueue<T>::consume_one() const
{
    std::lock_guard<std::mutex> lk(m_mutex);
    std::vector<T> res = m_dq.front();
    if(m_dq.size() > 1)
        m_dq.pop_front();
    return res;
}

template <typename T>
void PersistentVectorQueue<T>::refresh(std::vector<T> &t)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_dq.clear();
    m_dq.push_back(t);
}

template <typename T>
void PersistentVectorQueue<T>::refresh(const std::vector<T> &t) const
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_dq.clear();
    m_dq.push_back(t);
}

template <typename T>
bool PersistentVectorQueue<T>::empty()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_dq.empty();
}

template <typename T>
void PersistentVectorQueue<T>::clear()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_dq.clear();
}

#endif //CONCURRENT_VECTOR_QUEUE_HPP
