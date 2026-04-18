/**
 * @file IoData.cpp
 * @brief
 * @date 2026-04-17
 */

/*
===============================================================================================
                                    includes
===============================================================================================
*/

#include "Data/IoData.h"

/*
===============================================================================================
                                    functions
===============================================================================================
*/

/**
 *
 */
IoData::IoData()
    : selectShortPress_(false),
      selectLongPress_(false),
      optionShortPress_(false),
      optionLongPress_(false),
      returnShortPress_(false),
      returnLongPress_(false),
      specialShortPress_(false),
      specialLongPress_(false)
{
}

// ----- Getters -----
bool IoData::getSelectShortPress() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return selectShortPress_;
}

bool IoData::getSelectLongPress() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return selectLongPress_;
}

bool IoData::getOptionShortPress() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return optionShortPress_;
}

bool IoData::getOptionLongPress() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return optionLongPress_;
}

bool IoData::getReturnShortPress() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return returnShortPress_;
}

bool IoData::getReturnLongPress() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return returnLongPress_;
}

bool IoData::getSpecialShortPress() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return specialShortPress_;
}

bool IoData::getSpecialLongPress() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return specialLongPress_;
}

// ----- Setters -----
void IoData::setSelectShortPress(bool value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    selectShortPress_ = value;
}

void IoData::setSelectLongPress(bool value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    selectLongPress_ = value;
}

void IoData::setOptionShortPress(bool value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    optionShortPress_ = value;
}

void IoData::setOptionLongPress(bool value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    optionLongPress_ = value;
}

void IoData::setReturnShortPress(bool value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    returnShortPress_ = value;
}

void IoData::setReturnLongPress(bool value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    returnLongPress_ = value;
}

void IoData::setSpecialShortPress(bool value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    specialShortPress_ = value;
}

void IoData::setSpecialLongPress(bool value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    specialLongPress_ = value;
}