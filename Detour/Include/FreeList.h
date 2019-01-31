#pragma once

//#include "Runtime/Logging/LogAssert.h"
#include "DetourAssert.h"

#define FREELIST_EXPENSIVE_TEST 0

// Free list using index as next pointer.
// Note: Only calls constructor when new items are allocated from heap,
// and detructor is called only whwn the whole freelist is deleted.
// This is done intentionally so that we can retain some data (i.e. salt in off-mesh connection).
template<typename T>
class FreeList
{
public:
    enum { kNullLinkId = 0xffffffff };

    FreeList()
        : m_NextFree(kNullLinkId)
        , m_Capacity(0)
        , m_Data(0)
    {
    }

    ~FreeList()
    {
        for (unsigned int i = 0; i < m_Capacity; i++)
            m_Data[i].~T();
       // UNITY_FREE(kMemAI, m_Data);
		dtFree(m_Data);
    }

    T& operator[](unsigned int i)
    {
//         DebugAssertFormatMsg(i < m_Capacity, "FreeList index %u out of range %u", i, m_Capacity);
		dtAssert(i < m_Capacity);
        return m_Data[i];
    }

    const T& operator[](unsigned int i) const
    {
//         DebugAssertFormatMsg(i < m_Capacity, "FreeList index %u out of range %u", i, m_Capacity);
		dtAssert(i < m_Capacity);
        return m_Data[i];
    }

    unsigned int Alloc()
    {
        if (m_NextFree == kNullLinkId)
        {
            Grow(m_Capacity ? 2 * m_Capacity : 4);
            if (m_NextFree == kNullLinkId)
                return kNullLinkId;
        }
        unsigned int id = m_NextFree;
        m_NextFree = m_Data[id].next;
        m_Data[id].next = 0;
        return id;
    }

    void Release(unsigned int id)
    {
//         DebugAssertFormatMsg(id < m_Capacity, "FreeList release %u out of range %u", id, m_Capacity);
		dtAssert(id < m_Capacity);
#if !UNITY_RELEASE && FREELIST_EXPENSIVE_TEST
        for (unsigned int i = m_NextFree; i != DT_NULL_LINK; i = m_Data[i].next)
            //DebugAssertFormatMsg(i != id, "Attempt to release an index which isn't allocated: %u", id);
			dtAssert(i != id);
#endif

        m_Data[id].next = m_NextFree;
        m_NextFree = id;
    }

    inline unsigned int Capacity() const { return m_Capacity; }

    void Clear()
    {
        for (int i = 0; i < m_Capacity; i++)
            m_Data[i].~T();
        //UNITY_FREE(kMemAI, m_Data);
		dtFree(m_Data);

        m_Data = NULL;
        m_NextFree = kNullLinkId;
        m_Capacity = 0;
    }

private:

    void Grow(unsigned int s)
    {
        if (s <= m_Capacity || m_NextFree != kNullLinkId)
            return;
        //T* data = reinterpret_cast<T*>(UNITY_REALLOC(kMemAI, m_Data, sizeof(T) * s));
		T* data = reinterpret_cast<T*>(dtRealloc(m_Data,sizeof(T) * s) );
        if (!data)
            return;
        m_Data = data;
        for (unsigned int i = m_Capacity; i < s - 1; ++i)
        {
            new(&m_Data[i])T;
            m_Data[i].next = i + 1;
        }
        new(&m_Data[s - 1])T;
        m_Data[s - 1].next = kNullLinkId;
        m_NextFree = m_Capacity;
        m_Capacity = s;
    }

    unsigned int m_NextFree;
    unsigned int m_Capacity;
    T* m_Data;
};
