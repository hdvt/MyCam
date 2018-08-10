#include <queue>
#include <condition_variable>

template <typename T>
class Buff
{
    public:
        Buff(int maxSize);
        ~Buff();
        bool Put(const T &data);
        bool Put(T &&data);
        T Take();
        bool isEmpty();
        bool isFull();
        void Release();
        bool thStop;

    private:
    // thread
    std::mutex m_mutex;
    std::condition_variable m_cond;
    // buff
    std::queue<T> m_buff;
    const unsigned int m_maxSize;
};

template <typename T>
Buff<T>::Buff(int maxSize) : m_maxSize(maxSize), thStop(false)
{

}

template <typename T>
Buff<T>::~Buff()
{

}

template <typename T>
bool Buff<T>::isEmpty()
{
    if (m_buff.empty())
       return true;
    return false;
}

template <typename T>
bool Buff<T>::isFull()
{
    if (m_buff.size() == m_maxSize)
       return true;
    return false;
}

template <typename T>
bool Buff<T>::Put(const T &data)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    while (isFull() && !thStop)
        m_cond.wait(lock);
    m_buff.push(data);
    lock.unlock();
    m_cond.notify_all();
    return true;
}

template <typename T>
bool Buff<T>::Put(T &&data)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    while (isFull() && !thStop)
        m_cond.wait(lock);
    m_buff.push(std::move(data));
    lock.unlock();
    m_cond.notify_all();
    return true;
}

template <typename T>
T Buff<T>::Take()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    while (isEmpty() && !thStop)
        m_cond.wait(lock);
    T data;
    if (!isEmpty())
    {
        data = std::move(m_buff.front());    
        m_buff.pop();
        lock.unlock();
        m_cond.notify_all();
    }
    return data;
}
template <typename T>
void Buff<T>::Release()
{
    m_cond.notify_all();
}