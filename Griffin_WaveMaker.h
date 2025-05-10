// Griffin_WaveMaker.h   (crash-proof, no raw global publish)
#pragma once

#include <JuceHeader.h>
#include <vector>
#include <atomic>

#include "src/griffinwave5/AsyncMipBuilder.h"




// Use this enum to refer to the cables, eg. this->setGlobalCableValue<GlobalCables::cbl_e1_w1>(0.4)

// Subclass your node from this







namespace project
{
    using namespace juce;
    using namespace hise;
    using namespace scriptnode;




    // Global Cable Interface
    enum class GlobalCables
    {
        cbl_e1_w1 = 0,
        cbl_e1_w2 = 1,
        cbl_e1_w3 = 2
    };
    using cable_manager_t = routing::global_cable_cpp_manager<SN_GLOBAL_CABLE(328105083),
        SN_GLOBAL_CABLE(328105084),
        SN_GLOBAL_CABLE(328105085)>;




    template <int NV>
    struct Griffin_WaveMaker : public data::base, public cable_manager_t
    {
        /* ===== metadata =================================================== */
        SNEX_NODE(Griffin_WaveMaker);
        struct MetadataClass { SN_NODE_ID("Griffin_WaveMaker"); };

        static constexpr bool isModNode() { return false; }
        static constexpr bool isPolyphonic() { return NV > 1; }
        static constexpr bool hasTail() { return false; }
        static constexpr bool isSuspendedOnSilence() { return false; }
        static constexpr int  getFixChannelAmount() { return 2; }

        static constexpr int NumTables = 0;
        static constexpr int NumSliderPacks = 0;
        static constexpr int NumAudioFiles = 2;
        static constexpr int NumFilters = 0;
        static constexpr int NumDisplayBuffers = 0;

        /* ===== wavetable constants ======================================= */
        static constexpr int FrameSize = 2048;
        static constexpr int MaxFrames = 256;
        static constexpr int MaxSamples = FrameSize * MaxFrames;          // 524 288
        static constexpr int DecFactor = 4;
        static constexpr int DecSamples = MaxSamples / DecFactor;         // 131 072
        static constexpr int TripFactor = 3;
        static constexpr int TripledSamples = MaxSamples * TripFactor;        // 1 572 864
        static constexpr int TripledFrame = FrameSize * TripFactor;         // 6 144

        /* ===== storage ==================================================== */
        block              audioBlocks[NumAudioFiles];
        int                numSamples[NumAudioFiles]{ 0, 0 };

        std::vector<float> mixBuf;   // full-rate buffer (MaxSamples)
        std::vector<float> decBuf;   // decimated buffer (DecSamples)

        float* tripleView{ nullptr };   // producer view into AsyncMipBuilder slot

        WaitableEvent      wakeEvt;
        std::atomic<double> mix{ 0.5 };

        /* ===== builder thread ============================================ */
        struct Worker : Thread
        {
            Griffin_WaveMaker& owner;
            Worker(Griffin_WaveMaker& p) : Thread("WTBuilder"), owner(p) {}

            void run() override
            {
                constexpr int N = MaxSamples;

                for (;;)
                {
                    owner.wakeEvt.wait(-1);
                    if (threadShouldExit()) return;

                    /* make sure at least one file is loaded */
                    const bool has0 = (owner.numSamples[0] == N);
                    const bool has1 = (owner.numSamples[1] == N);
                    if (!has0 && !has1) continue;

                    /* --- 1. blend ------------------------------------------------ */
                    const double m = owner.mix.load(std::memory_order_acquire);
                    FloatVectorOperations::clear(owner.mixBuf.data(), N);

                    double g0 = 0.0, g1 = 0.0;
                    if (has0 && has1)
                    {
                        double angle = m * MathConstants<double>::halfPi;
                        g0 = std::cos(angle);
                        g1 = std::sin(angle);
                    }
                    else if (has0) g0 = 1.0;
                    else           g1 = 1.0;

                    if (has0)
                        FloatVectorOperations::addWithMultiply(
                            owner.mixBuf.data(), owner.audioBlocks[0].data, (float)g0, N);
                    if (has1)
                        FloatVectorOperations::addWithMultiply(
                            owner.mixBuf.data(), owner.audioBlocks[1].data, (float)g1, N);

                    /* --- 2. down-sample for the GUI cable ----------------------- */
                    for (int i = 0; i < DecSamples; ++i)
                        owner.decBuf[i] = owner.mixBuf[i * DecFactor];

                    Array<var> decArr;
                    decArr.ensureStorageAllocated(DecSamples);
                    for (float v : owner.decBuf) decArr.add(v);

                    // Send graphics to cbl_e1_w3
                    owner.sendDataToGlobalCable<GlobalCables::cbl_e1_w3>(decArr);

                    /* --- 3. write tripled block straight into builder slot ------ */
                    owner.tripleView = gw5::AsyncMipBuilder::instance().writeSlot();

                    float* dst = owner.tripleView;
                    const float* src = owner.mixBuf.data();
                    for (int f = 0; f < MaxFrames; ++f)
                    {
                        memcpy(dst, src, FrameSize * sizeof(float));
                        memcpy(dst + FrameSize, src, FrameSize * sizeof(float));
                        memcpy(dst + 2 * FrameSize, src, FrameSize * sizeof(float));
                        dst += TripledFrame;
                        src += FrameSize;
                    }

                    /* hand-off to background builder */
                    gw5::AsyncMipBuilder::instance().commitSlot();
                }
            }
        } worker{ *this };

        /* ===== lifecycle ================================================== */
        void prepare(PrepareSpecs) 
        {
            mixBuf.resize(MaxSamples);
            decBuf.resize(DecSamples);

            gw5::AsyncMipBuilder::instance().configure(TripledSamples, 12);
            worker.startThread();
        }

        void reset()  { wakeEvt.signal(); }

        SN_EMPTY_PROCESS_FRAME;
        SN_EMPTY_HANDLE_EVENT;
        SN_EMPTY_PROCESS;

        /* ===== external data ============================================= */
        void setExternalData(const snex::ExternalData& d, int idx) 
        {
            if (d.dataType != snex::ExternalData::DataType::AudioFile || idx >= NumAudioFiles)
                return;

            if (d.numChannels != 1 || d.numSamples != MaxSamples)
            {
                Logger::writeToLog("WaveMaker: wavetable must be mono "
                    + String(MaxSamples) + " samples");
                numSamples[idx] = 0;
            }
            else
            {
                d.referBlockTo(audioBlocks[idx], 0);
                numSamples[idx] = d.numSamples;
            }
            wakeEvt.signal();
        }

        /* ===== parameter ================================================== */
        template<int P>
        void setParameter(double v)
        {
            if constexpr (P == 0)      // Mix
            {
                double prev = mix.load();
                if (std::abs(prev - v) > 1e-6)
                {
                    mix.store(v);
                    wakeEvt.signal();
                }
            }
        }

        void createParameters(ParameterDataList& ps)
        {
            parameter::data p("Mix", { 0.0, 1.0 });
            p.setDefaultValue(0.5);
            registerCallback<0>(p);
            ps.add(std::move(p));
        }

        // DONT REMOVE THE LINES BELOW – build system markers
        // 'NumFilters' 'NumTables' 'NumTables' 'NumSliderPacks'
        // 'NumSliderPacks' 'NumFilters' 'NumDisplayBuffers' 'NumDisplayBuffers'
    };
} // namespace project
