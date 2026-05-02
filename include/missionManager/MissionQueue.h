#pragma once

#include <missionManager/mission.h>

/**
 * Fixed-size circular queue for missions.
 * Avoids heap allocation entirely - no std::vector, no malloc/free.
 * This prevents heap fragmentation which causes segfaults on Arduino.
 */
class MissionQueue
{
public:
    static constexpr int MAX_MISSIONS = 20;

    MissionQueue() : m_head(0), m_tail(0), m_count(0) {}

    bool push(const Mission& mission)
    {
        if (m_count >= MAX_MISSIONS)
            return false; // Queue full
        m_buffer[m_tail] = mission;
        m_tail = (m_tail + 1) % MAX_MISSIONS;
        m_count++;
        return true;
    }

    void pop()
    {
        if (m_count > 0)
        {
            m_head = (m_head + 1) % MAX_MISSIONS;
            m_count--;
        }
    }

    [[nodiscard]] Mission* front()
    {
        if (m_count == 0)
            return nullptr;
        return &m_buffer[m_head];
    }

    [[nodiscard]] const Mission* front() const
    {
        if (m_count == 0)
            return nullptr;
        return &m_buffer[m_head];
    }

    [[nodiscard]] Mission* at(int index)
    {
        if (index < 0 || index >= m_count)
            return nullptr;
        return &m_buffer[(m_head + index) % MAX_MISSIONS];
    }

    [[nodiscard]] const Mission* at(int index) const
    {
        if (index < 0 || index >= m_count)
            return nullptr;
        return &m_buffer[(m_head + index) % MAX_MISSIONS];
    }

    void clear() { m_head = 0; m_tail = 0; m_count = 0; }

    [[nodiscard]] bool empty() const { return m_count == 0; }
    [[nodiscard]] bool full() const { return m_count >= MAX_MISSIONS; }
    [[nodiscard]] int size() const { return m_count; }

private:
    Mission m_buffer[MAX_MISSIONS];
    int m_head;
    int m_tail;
    int m_count;
};
