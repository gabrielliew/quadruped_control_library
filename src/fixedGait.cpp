#include "quadruped_control_library/fixedGait.hpp"
#include <iostream>
#include <cassert>

FixedGait::FixedGait(uint numLegs, uint numSegments, double gaitDuration, double contactDuration, std::vector<int> offset)
{
    numSegments_ = numSegments;
    gaitDurationNs_ = static_cast<uint64_t>(gaitDuration * 1e9);
    contactDurationNs_ = static_cast<uint64_t>(contactDuration * 1e9);
    swingDurationNs_ = gaitDurationNs_ - contactDurationNs_;
    singleSegmentNs_ = gaitDurationNs_ / numSegments;
    numLegs_ = numLegs;
#ifdef NDEBUG
    assert(numLegs_ == offset.size());
#endif
    for (int i = 0; i < numLegs_; i++)
    {
        offset_.push_back(offset[i] * singleSegmentNs_);
    }
}

std::vector<double> FixedGait::getContactProgression(uint64_t timeFromStart)
{
    // gets current time segment progress
    uint64_t currentTime = timeFromStart % gaitDurationNs_;
    std::vector<double> tempProgress;
    for (int i = 0; i < numLegs_; i++)
    {
        /*
        There are four cases this may happen:
        a) 0000011111 offset=5
        b) 0111110000 offset=1 ()
        c) 1111100000 offset=0 (0)
        d) 1110000011 offset=8 (+10-8)
              ^
        Considering the current progress is at ^ mark, segment =3.5
        0 is swing, 1 is contact so the progress has to be shifted to normalize it to 1111100000 at each leg but shift segment progress
        */
        uint64_t normalizedTimeSegment;
        if (currentTime > offset_[i])
        {
            normalizedTimeSegment = currentTime - offset_[i];
        }
        else
        {
            normalizedTimeSegment = currentTime - offset_[i] + gaitDurationNs_;
        }
        if (normalizedTimeSegment <= contactDurationNs_)
        {
            tempProgress.push_back(static_cast<double>(normalizedTimeSegment) / contactDurationNs_);
        }
        else
        {
            tempProgress.push_back(0);
        }
#ifdef NDEBUG
        std::cout << i << "  " << tempProgress[i] << std::endl;
#endif
    }
    return tempProgress;
}

std::vector<double> FixedGait::getSwingProgression(uint64_t timeFromStart)
{
    // gets current time segment progress
    uint64_t currentTime = timeFromStart % gaitDurationNs_;
    std::vector<double> tempProgress;
    for (int i = 0; i < numLegs_; i++)
    {
        /*
        There are four cases this may happen:
        a) 0000011111 offset=5
        b) 0111110000 offset=1 ()
        c) 1111100000 offset=0 (0)
        d) 1110000011 offset=8 (+10-8)
              ^
        Considering the current progress is at ^ mark, segment =3.5
        0 is swing, 1 is contact so the progress has to be shifted to normalize it to 1111100000 at each leg but shift segment progress
        */
        uint64_t normalizedTimeSegment;
        if (currentTime > offset_[i])
        {
            normalizedTimeSegment = currentTime - offset_[i];
        }
        else
        {
            normalizedTimeSegment = currentTime - offset_[i] + gaitDurationNs_;
        }
        if (normalizedTimeSegment > contactDurationNs_)
        {
            tempProgress.push_back(static_cast<double>(normalizedTimeSegment - contactDurationNs_) / swingDurationNs_);
        }
        else
        {
            tempProgress.push_back(0);
        }
#ifdef NDEBUG
        std::cout << i << "  " << tempProgress[i] << std::endl;
#endif
    }
    return tempProgress;
}

std::vector<bool> FixedGait::getGaitTable(uint64_t timeFromStart)
{
    std::vector<bool> gaitTable;
    for (int i = 0; i < numSegments_; i++)
    {
        auto contactState = getContactProgression(timeFromStart);
        for (int j = 0; j < numLegs_; j++)
        {
            if (contactState[j] != 0)
            {
                gaitTable.push_back(1);
            }
            else
            {
                gaitTable.push_back(0);
            }
        }
        timeFromStart += singleSegmentNs_;
    }
// #ifdef NDEBUG
    for (int i = 0, j = 0; i < numSegments_ * numLegs_; i++, j++)
    {
        std::cout << gaitTable[i] << " ";
        if (j == 3)
        {
            std::cout << std::endl;
            j = -1;
        }
    }
// #endif
    return gaitTable;
}