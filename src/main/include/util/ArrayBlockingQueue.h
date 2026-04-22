#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class ArrayBlockingQueue
{
public:
    ArrayBlockingQueue(int capacity) : capacity(capacity)
    {
    }

    bool add(T element)
    {
        mtx.lock();
        if (elements.size() == capacity)
        {
            throw std::runtime_error("");
        }
        elements.emplace_back(element);
        mtx.unlock();
        return true;
    }

    void clear()
    {
        mtx.lock();
        while (elements.size() > 0)
        {
            elements.erase(elements.begin());
        }
        mtx.unlock();
    }

    bool contains(T el)
    {
        mtx.lock();
        for (auto element : elements)
        {
            if (el == element)
            {
                mtx.unlock();
                return true;
            }
        }
        mtx.unlock();
        return false;
    }

    int size()
    {
        mtx.lock();
        int size = elements.size();
        mtx.unlock();
        return size;
    }

    bool offer(T el)
    {
        mtx.lock();
        if (elements.size() == capacity)
        {
            mtx.unlock();
            return false;
        }
        elements.emplace_back(el);
        mtx.unlock();
        return true;
    }

    std::vector<T> toVector()
    {
        return elements;
    }

private:
    int capacity;
    std::vector<T> elements;

    std::recursive_mutex mtx;
};