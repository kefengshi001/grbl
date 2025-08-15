#include <atomic>
#include <iostream>

template <typename T, size_t Capacity>
class SPSCQueue
{
private:
    T buffer[Capacity];
    std::atomic<size_t> readIndex{0};
    std::atomic<size_t> writeIndex{0};

public:
    bool enqueue(const T &item)
    {
        size_t currentWriteIndex = writeIndex.load(std::memory_order_relaxed);
        size_t nextWriteIndex = (currentWriteIndex + 1) % Capacity;
        if (nextWriteIndex == readIndex.load(std::memory_order_acquire))
        {
            // 队列已满
            return false;
        }
        buffer[currentWriteIndex] = item;
        writeIndex.store(nextWriteIndex, std::memory_order_release);
        return true;
    }

    bool dequeue(T &item)
    {
        size_t currentReadIndex = readIndex.load(std::memory_order_relaxed);
        if (currentReadIndex == writeIndex.load(std::memory_order_acquire))
        {
            // 队列已空
            return false;
        }
        item = buffer[currentReadIndex];
        readIndex.store((currentReadIndex + 1) % Capacity, std::memory_order_release);
        return true;
    }
};