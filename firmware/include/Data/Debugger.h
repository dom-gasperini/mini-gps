/**
 * @file Debugger.h
 * @brief
 * @date 2026-04-20
 */

#ifndef DEBUGGER_DATA_H
#define DEBUGGER_DATA_H

/*
===============================================================================================
                                    includes
===============================================================================================
*/

#include <Arduino.h>
#include <mutex>
#include <atomic>

#include <Data/ExecutiveData.h>
#include <Data/GpsData.h>
#include <Data/IoData.h>

/*
===============================================================================================
                                    class
===============================================================================================
*/

/**
 *
 */
class DebuggerData
{
private:
    mutable std::mutex mutex;

    // debug toggle (mutex protected)
    bool debugEnabled;
    bool ioDebugEnabled;
    bool gpsDebugEnabled;
    bool displayDebugEnabled;
    bool schedulerDebugEnable;

    // data
    ExecutiveData *executiveData;
    GpsData *gpsData;
    IoData *ioData;

    // display debugging (mutex protected)
    String debugText;

    // scheduler data (atomic)
    std::atomic<unsigned long> executiveTaskCount;
    std::atomic<unsigned long> ioTaskCount;
    std::atomic<unsigned long> gpsTaskCount;
    std::atomic<unsigned long> displayTaskCount;

    int displayRefreshRate;

    std::atomic<unsigned long> executiveTaskPreviousCount;
    std::atomic<unsigned long> ioTaskPreviousCount;
    std::atomic<unsigned long> gpsTaskPreviousCount;
    std::atomic<unsigned long> displayTaskPreviousCount;

public:
    DebuggerData();

    // --- Mutex-protected setters ---
    void setDebugEnabled(bool value);
    void setIoDebugEnabled(bool value);
    void setGpsDebugEnabled(bool value);
    void setDisplayDebugEnabled(bool value);
    void setSchedulerDebugEnable(bool value);
    void setDebugText(const String &value);
    void setDisplayRefreshRate(int value);

    void setExecutiveData(ExecutiveData *data);
    void setGpsData(GpsData *data);
    void setIoData(IoData *data);

    // --- Atomic setters ---
    void setExecutiveTaskCount(unsigned long value);
    void setIoTaskCount(unsigned long value);
    void setGpsTaskCount(unsigned long value);
    void setDisplayTaskCount(unsigned long value);
    void setExecutiveTaskPreviousCount(unsigned long value);
    void setIoTaskPreviousCount(unsigned long value);
    void setGpsTaskPreviousCount(unsigned long value);
    void setDisplayTaskPreviousCount(unsigned long value);

    // --- Mutex-protected getters ---
    bool getDebugEnabled() const;
    bool getIoDebugEnabled() const;
    bool getGpsDebugEnabled() const;
    bool getDisplayDebugEnabled() const;
    bool getSchedulerDebugEnable() const;
    String getDebugText() const;
    int getDisplayRefreshRate() const;

    ExecutiveData *getExecutiveData() const;
    GpsData *getGpsData() const;
    IoData *getIoData() const;

    // --- Atomic getters ---
    unsigned long getExecutiveTaskCount() const;
    unsigned long getIoTaskCount() const;
    unsigned long getGpsTaskCount() const;
    unsigned long getDisplayTaskCount() const;
    unsigned long getExecutiveTaskPreviousCount() const;
    unsigned long getIoTaskPreviousCount() const;
    unsigned long getGpsTaskPreviousCount() const;
    unsigned long getDisplayTaskPreviousCount() const;

    // --- Increment helpers ---
    void incrementExecutiveTaskCount();
    void incrementIoTaskCount();
    void incrementGpsTaskCount();
    void incrementDisplayTaskCount();
};

#endif // DEBUGGER_DATA_H