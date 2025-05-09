// AsyncMipBuilder.h   (fixed 1-May-2025)
#pragma once

#include <JuceHeader.h>
#include <atomic>
#include <memory>
#include <vector>
#include <cstring>

#include "InterpPack.h"
#include "MipMapFlt.hpp"
#include "ResamplerFlt.h"

namespace gw5
{
    /*
    ==============================================================================
    Name: AsyncMipBuilder
    Purpose: Single background thread that converts a *tripled* wavetable block
             into a ready-to-use MipMapFlt and publishes it lock-free.
    ==============================================================================
    */
    class AsyncMipBuilder final
    {
    public:
        /* singleton access */
        static AsyncMipBuilder& instance() noexcept
        {
            static AsyncMipBuilder s;
            return s;
        }

        /* configure once at startup */
        void configure(long tripLen, int mipLevels) noexcept
        {
            _tripLen = tripLen;
            _mipLevels = mipLevels;
            _slot.resize(static_cast<size_t>(tripLen));
        }

        /* producer – WaveMaker worker thread */
        float* writeSlot() noexcept { return _slot.data(); }
        void   commitSlot() noexcept
        {
            _lastTouch.store(Time::getMillisecondCounterHiRes(),
                std::memory_order_release);
            _slotReady.store(true, std::memory_order_release);
        }

        /* consumer – audio / render threads */
        std::shared_ptr<const MipMapFlt> current() const noexcept
        {
            return std::atomic_load(&_active);
        }

        // add just after current()
        const float* currentTableRaw() const noexcept
        {
            auto mp = current();
            return (mp && mp->is_ready()) ? mp->use_table(0) : nullptr;
        }


        /* diagnostic */
        bool isBuilding() const noexcept { return _building.load(std::memory_order_acquire); }

    private:
        AsyncMipBuilder() : _worker(*this) {}
        ~AsyncMipBuilder() = default;            // <-- override removed

        AsyncMipBuilder(const AsyncMipBuilder&) = delete;
        AsyncMipBuilder& operator=(const AsyncMipBuilder&) = delete;

        /* ----------------------------------------------------------------- */
        /* background worker                                                 */
        /* ----------------------------------------------------------------- */
        class Worker : public juce::Thread
        {
        public:
            explicit Worker(AsyncMipBuilder& o)
                : juce::Thread("MipBuilder"), owner(o)
            {
                startThread(1);   // lowest priority
            }

            void run() override
            {
                for (;;)
                {
                    if (threadShouldExit())
                        return;

                    if (!owner._slotReady.load(std::memory_order_acquire))
                    {
                        juce::Thread::sleep(5);
                        continue;
                    }

                    auto now = Time::getMillisecondCounterHiRes();
                    auto last = owner._lastTouch.load(std::memory_order_acquire);
                    if (now - last < 60.0)
                    {
                        juce::Thread::sleep(5);
                        continue;
                    }

                    owner._slotReady.store(false, std::memory_order_release);
                    owner._building.store(true, std::memory_order_release);

                    /* build mip directly from producer slot */
                    auto mp = std::make_shared<MipMapFlt>();
                    mp->init_sample(owner._tripLen,
                        InterpPack::get_len_pre(),
                        InterpPack::get_len_post(),
                        owner._mipLevels,
                        ResamplerFlt::_fir_mip_map_coef_arr,
                        ResamplerFlt::MIP_MAP_FIR_LEN);
                    mp->fill_sample(owner._slot.data(), owner._tripLen);

                    std::atomic_store(&owner._active, std::shared_ptr<const MipMapFlt>(mp));

                    owner._building.store(false, std::memory_order_release);
                }
            }

        private:
            AsyncMipBuilder& owner;
        };

        /* ----------------------------------------------------------------- */
        /* shared state                                                      */
        /* ----------------------------------------------------------------- */
        std::vector<float>               _slot;        // producer buffer
        std::shared_ptr<const MipMapFlt> _active;      // latest built mip

        std::atomic<bool>    _building{ false };
        std::atomic<bool>    _slotReady{ false };
        std::atomic<int64_t> _lastTouch{ 0 };

        long _tripLen = 0;
        int  _mipLevels = 0;

        Worker _worker;
    };

} // namespace gw5

// DONT REMOVE THESE LINES:
// 'NumFilters' 'NumTables' 'NumTables' 'NumSliderPacks'
// 'NumSliderPacks' 'NumFilters' 'NumDisplayBuffers' 'NumDisplayBuffers'
