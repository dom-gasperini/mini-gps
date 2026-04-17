#ifndef IODATA_H
#define IODATA_H

#include <mutex>

class IoData
{
public:
    IoData();

    // Getters
    bool getSelectShortPress() const;
    bool getSelectLongPress() const;
    bool getOptionShortPress() const;
    bool getOptionLongPress() const;
    bool getReturnShortPress() const;
    bool getReturnLongPress() const;
    bool getSpecialShortPress() const;
    bool getSpecialLongPress() const;

    // Setters
    void setSelectShortPress(bool value);
    void setSelectLongPress(bool value);
    void setOptionShortPress(bool value);
    void setOptionLongPress(bool value);
    void setReturnShortPress(bool value);
    void setReturnLongPress(bool value);
    void setSpecialShortPress(bool value);
    void setSpecialLongPress(bool value);

private:
    mutable std::mutex m_mutex;

    bool selectShortPress_;
    bool selectLongPress_;
    bool optionShortPress_;
    bool optionLongPress_;
    bool returnShortPress_;
    bool returnLongPress_;
    bool specialShortPress_;
    bool specialLongPress_;
};

#endif // IODATA_H