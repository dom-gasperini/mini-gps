/**
 * @file Debugger.h
 * @brief
 * @date 2026-04-20
 */

/*
===============================================================================================
                                    includes
===============================================================================================
*/

#include "Data/Debugger.h"

/*
===============================================================================================
                                function declarations
===============================================================================================
*/

// --- Constructor ---
DebuggerData::DebuggerData()
    : debugEnabled(false),
      ioDebugEnabled(false),
      gpsDebugEnabled(false),
      displayDebugEnabled(false),
      schedulerDebugEnable(false),
      debugText(""),
      executiveTaskCount(0),
      ioTaskCount(0),
      gpsTaskCount(0),
      displayTaskCount(0),
      displayRefreshRate(0),
      executiveTaskPreviousCount(0),
      ioTaskPreviousCount(0),
      gpsTaskPreviousCount(0),
      displayTaskPreviousCount(0)
{
}

// --- Mutex-protected setters ---
void DebuggerData::setDebugEnabled(bool value)
{
    std::lock_guard<std::mutex> lock(mutex);
    debugEnabled = value;
}

void DebuggerData::setIoDebugEnabled(bool value)
{
    std::lock_guard<std::mutex> lock(mutex);
    ioDebugEnabled = value;
}

void DebuggerData::setGpsDebugEnabled(bool value)
{
    std::lock_guard<std::mutex> lock(mutex);
    gpsDebugEnabled = value;
}

void DebuggerData::setDisplayDebugEnabled(bool value)
{
    std::lock_guard<std::mutex> lock(mutex);
    displayDebugEnabled = value;
}

void DebuggerData::setSchedulerDebugEnable(bool value)
{
    std::lock_guard<std::mutex> lock(mutex);
    schedulerDebugEnable = value;
}

void DebuggerData::setDebugText(const String &value)
{
    std::lock_guard<std::mutex> lock(mutex);
    debugText = value;
}

void DebuggerData::setDisplayRefreshRate(int value)
{
    std::lock_guard<std::mutex> lock(mutex);
    displayRefreshRate = value;
}

void DebuggerData::setExecutiveData(ExecutiveData *data)
{
    std::lock_guard<std::mutex> lock(mutex);
    executiveData = data;
}

void DebuggerData::setGpsData(GpsData *data)
{
    std::lock_guard<std::mutex> lock(mutex);
    gpsData = data;
}

void DebuggerData::setIoData(IoData *data)
{
    std::lock_guard<std::mutex> lock(mutex);
    ioData = data;
}

// --- Atomic setters ---
void DebuggerData::setExecutiveTaskCount(unsigned long value)
{
    executiveTaskCount.store(value, std::memory_order_relaxed);
}

void DebuggerData::setIoTaskCount(unsigned long value)
{
    ioTaskCount.store(value, std::memory_order_relaxed);
}

void DebuggerData::setGpsTaskCount(unsigned long value)
{
    gpsTaskCount.store(value, std::memory_order_relaxed);
}

void DebuggerData::setDisplayTaskCount(unsigned long value)
{
    displayTaskCount.store(value, std::memory_order_relaxed);
}

void DebuggerData::setExecutiveTaskPreviousCount(unsigned long value)
{
    executiveTaskPreviousCount.store(value, std::memory_order_relaxed);
}

void DebuggerData::setIoTaskPreviousCount(unsigned long value)
{
    ioTaskPreviousCount.store(value, std::memory_order_relaxed);
}

void DebuggerData::setGpsTaskPreviousCount(unsigned long value)
{
    gpsTaskPreviousCount.store(value, std::memory_order_relaxed);
}

void DebuggerData::setDisplayTaskPreviousCount(unsigned long value)
{
    displayTaskPreviousCount.store(value, std::memory_order_relaxed);
}

// --- Mutex-protected getters ---
bool DebuggerData::getDebugEnabled() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return debugEnabled;
}

bool DebuggerData::getIoDebugEnabled() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return ioDebugEnabled;
}

bool DebuggerData::getGpsDebugEnabled() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return gpsDebugEnabled;
}

bool DebuggerData::getDisplayDebugEnabled() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return displayDebugEnabled;
}

bool DebuggerData::getSchedulerDebugEnable() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return schedulerDebugEnable;
}

String DebuggerData::getDebugText() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return debugText;
}

int DebuggerData::getDisplayRefreshRate() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return displayRefreshRate;
}

ExecutiveData *DebuggerData::getExecutiveData() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return executiveData;
}

GpsData *DebuggerData::getGpsData() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return gpsData;
}

IoData *DebuggerData::getIoData() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return ioData;
}

// --- Atomic getters ---
unsigned long DebuggerData::getExecutiveTaskCount() const
{
    return executiveTaskCount.load(std::memory_order_relaxed);
}

unsigned long DebuggerData::getIoTaskCount() const
{
    return ioTaskCount.load(std::memory_order_relaxed);
}

unsigned long DebuggerData::getGpsTaskCount() const
{
    return gpsTaskCount.load(std::memory_order_relaxed);
}

unsigned long DebuggerData::getDisplayTaskCount() const
{
    return displayTaskCount.load(std::memory_order_relaxed);
}

unsigned long DebuggerData::getExecutiveTaskPreviousCount() const
{
    return executiveTaskPreviousCount.load(std::memory_order_relaxed);
}

unsigned long DebuggerData::getIoTaskPreviousCount() const
{
    return ioTaskPreviousCount.load(std::memory_order_relaxed);
}

unsigned long DebuggerData::getGpsTaskPreviousCount() const
{
    return gpsTaskPreviousCount.load(std::memory_order_relaxed);
}

unsigned long DebuggerData::getDisplayTaskPreviousCount() const
{
    return displayTaskPreviousCount.load(std::memory_order_relaxed);
}

// --- Increment helpers ---
void DebuggerData::incrementExecutiveTaskCount()
{
    executiveTaskCount.fetch_add(1, std::memory_order_relaxed);
}

void DebuggerData::incrementIoTaskCount()
{
    ioTaskCount.fetch_add(1, std::memory_order_relaxed);
}

void DebuggerData::incrementGpsTaskCount()
{
    gpsTaskCount.fetch_add(1, std::memory_order_relaxed);
}

void DebuggerData::incrementDisplayTaskCount()
{
    displayTaskCount.fetch_add(1, std::memory_order_relaxed);
}