// Griffin_WT.h   (updated 5-May-2025)

#pragma once

#include <JuceHeader.h>
#include <array>
#include <vector>
#include <memory>
#include <atomic>
#include <mutex>

#include "src/griffinwave5/BaseVoiceState.cpp"
#include "src/griffinwave5/BaseVoiceState.h"
#include "src/griffinwave5/rspl.hpp"
#include "src/griffinwave5/InterpPack.cpp"
#include "src/griffinwave5/InterpPack.h"
#include "src/griffinwave5/MipMapFlt.hpp"
#include "src/griffinwave5/ResamplerFlt.cpp"
#include "src/griffinwave5/ResamplerFlt.h"
#include "src/griffinwave5/Wave.h"
#include "src/griffinwave5/AsyncMipBuilder.h"

namespace project
{
    using namespace juce;
    using namespace hise;
    using namespace scriptnode;

    /* --------------------------------------------------------------------- */
    /*  CONSTANTS                                                            */
    /* --------------------------------------------------------------------- */

    template <int NV>
    struct Griffin_WT;

    static constexpr float kVoiceDetuneLUT[24] =
    {
        0.0f, 0.3f, -0.2f, 3.119f, 2.5f, 0.1f, -0.1f, 0.0f,
        4.119f, 1.5f, 2.119f, 3.119f, 1.5f, 0.0f, 0.2f, 0.1f,
        1.5f, 0.0f, 0.0f, 1.0f, 3.119f, 0.5f, 0.0f, 1.5f
    };
    inline float getVoiceDetune(int idx) noexcept { return kVoiceDetuneLUT[idx % 24]; }

    /* --------------------------------------------------------------------- */
    /*  SHARED DEFAULT WAVE – ONE PER PROCESS                                */
    /* --------------------------------------------------------------------- */

    namespace
    {
        inline const std::shared_ptr<const gw5::MipMapFlt>& builtinMip()
        {
            static std::shared_ptr<const gw5::MipMapFlt> s = [] {
                constexpr int FRAME_SIZE = 2048;
                constexpr int MAX_FRAMES = 256;
                constexpr int PADDED = FRAME_SIZE * 3;
                constexpr int TOTAL_LEN = MAX_FRAMES * PADDED;

                auto mp = std::make_shared<gw5::MipMapFlt>();
                mp->init_sample(TOTAL_LEN,
                    gw5::InterpPack::get_len_pre(),
                    gw5::InterpPack::get_len_post(),
                    12,
                    gw5::ResamplerFlt::_fir_mip_map_coef_arr,
                    gw5::ResamplerFlt::MIP_MAP_FIR_LEN);
                mp->fill_sample(wavetable, TOTAL_LEN);
                return mp;
                }();
            return s;
        }
    }

    /* --------------------------------------------------------------------- */
    /*  ONE RESAMPLER LANE                                                   */
    /* --------------------------------------------------------------------- */

    struct Lane
    {
        gw5::ResamplerFlt res;
        int               frameIdx = -1;
        bool              active = false;
    };

    /* --------------------------------------------------------------------- */
    /*  MAIN NODE                                                            */
    /* --------------------------------------------------------------------- */

    template <int NV>
    struct Griffin_WT : public data::base
    {
        SNEX_NODE(Griffin_WT);
        struct MetadataClass { SN_NODE_ID("Griffin_WT"); };

        static constexpr int MAX_FRAMES = 256;
        static constexpr int FRAME_SIZE = 2048;
        static constexpr int PADDED = FRAME_SIZE * 3;
        static constexpr int SLICE = 8;
        static constexpr int FADE_LEN = gw5::BaseVoiceState::FADE_LEN;
        static constexpr int BITS_OCT = gw5::BaseVoiceState::NBR_BITS_PER_OCT;
        static constexpr double TARGET_ROOT_HZ = 32.703195;
        static constexpr double SEMI2BITS = double(1 << BITS_OCT) / 12.0;

        static constexpr bool isModNode() { return false; }
        static constexpr bool isPolyphonic() { return NV > 1; }
        static constexpr bool hasTail() { return false; }
        static constexpr bool isSuspendedOnSilence() { return false; }
        static constexpr int  getFixChannelAmount() { return 2; }

        static constexpr int NumTables = 0;
        static constexpr int NumSliderPacks = 0;
        static constexpr int NumAudioFiles = 0;
        static constexpr int NumFilters = 0;
        static constexpr int NumDisplayBuffers = 0;

        /* ----------------------------------------------------------------- */
        /*  VOICE PACK                                                       */
        /* ----------------------------------------------------------------- */

        struct VoicePack
        {
            Lane   A, B;
            int    pitchBits = 0;
            double semiOff = 0.0;
            double multOff = 1.0;
            bool   toggle = false;
            bool   fading = false;
            float  fadeAlpha = 1.0f;
            int    frameParam = 0;
            int    pendFrame = 0;
            bool   pendFlag = false;
            int    midi = -1;
            float  vel = 1.0f;
            bool   active = false;

            // glide state
            double glideCurBits = 0.0;
            double glideStepBitsPerSample = 0.0;
            int    glideSamplesRemaining = 0;

            static constexpr float fadeDelta() { return 1.0f / float(FADE_LEN); }

            void clear()
            {
                A.active = B.active = active = false;
                fading = toggle = pendFlag = false;
                fadeAlpha = 1.0f;
                glideCurBits = 0.0;
                glideStepBitsPerSample = 0.0;
                glideSamplesRemaining = 0;
            }

            void reset(int note, float v, int gFrame, double semi, double mult)
            {
                clear();
                midi = note;
                vel = v;
                frameParam = pendFrame = gFrame;
                semiOff = semi;
                multOff = mult;
                active = true;
            }
        };

        PolyData<VoicePack, NV> voices;

        /* ----------------------------------------------------------------- */
        /*  PUBLIC API                                                       */
        /* ----------------------------------------------------------------- */

        void reset() { for (auto& v : voices) v.clear(); }

        Griffin_WT()
            : globalVolume(0.8f),
            paramSemi(0.0),
            paramMult(1.0),
            paramGlideOn(false),
            paramGlideTime(0.1),
            paramGlideTarget(1.0),
            _activeMip(builtinMip())
        {
        }

        void prepare(PrepareSpecs spec)
        {
            sr = spec.sampleRate;
            lastSpecs = spec;
            haveSpecs = true;

            rootOffSemis = 12.0 * std::log2(TARGET_ROOT_HZ / (sr / double(FRAME_SIZE)));

            for (int f = 0; f < MAX_FRAMES; ++f)
                frameStart[f] = gw5::Int64(f) * PADDED + FRAME_SIZE;

            voices.prepare(lastSpecs);
            for (auto& v : voices) initVoice(v);

            ready = true;
        }

        void handleHiseEvent(HiseEvent& e)
        {
            if (!ready) return;

            if (e.isNoteOn())
            {
                auto& vp = voices.get();
                vp.reset(e.getNoteNumber(),
                    e.getFloatVelocity(),
                    globalFrame,
                    paramSemi,
                    paramMult);

                vp.A.res.set_sample_sp(_activeMip); vp.A.res.clear_buffers();
                vp.B.res.set_sample_sp(_activeMip); vp.B.res.clear_buffers();

                updatePitch(vp);

                if (paramGlideOn)
                {
                    // multiplier -> semitones -> bits
                    const double glideSemis = std::log2(paramGlideTarget) * 12.0;
                    const double targetBits = glideSemis * SEMI2BITS;
                    const double smoothingSamples = paramGlideTime * sr;
                    if (smoothingSamples > 0.0)
                    {
                        vp.glideStepBitsPerSample = targetBits / smoothingSamples;
                        vp.glideSamplesRemaining = int(smoothingSamples);
                    }
                    else
                    {
                        vp.glideCurBits = targetBits;
                        vp.glideStepBitsPerSample = 0.0;
                        vp.glideSamplesRemaining = 0;
                    }
                }

                // random start phase...
                uint32 rand32 = Random::getSystemRandom().nextInt();
                float  noteFrac = float(e.getNoteNumber()) / 127.0f;
                float  phase = 17.0f + noteFrac * (60.0f - 17.0f);
                gw5::Int64 maxR = (cycle * gw5::Int64(phase)) / 100;
                gw5::Int64 randIp = gw5::Int64(rand32) % maxR;
                gw5::Int64 pos = ((frameStart[vp.frameParam] + randIp) << 32) | gw5::Int64(rand32);

                vp.A.res.set_playback_pos(pos);
                vp.A.frameIdx = vp.frameParam;
                vp.A.active = true;
            }
        }

        template <typename PD>
        void process(PD& d)
        {
            if (auto mp = gw5::AsyncMipBuilder::instance().current();
                mp && mp->is_ready() && mp.get() != _activeMip.get())
            {
                _activeMip = mp;
                for (auto& v : voices)
                {
                    v.A.res.set_sample_sp(_activeMip); v.A.res.clear_buffers();
                    v.B.res.set_sample_sp(_activeMip); v.B.res.clear_buffers();
                    if (v.active)
                    {
                        v.A.res.set_pitch(v.pitchBits);
                        v.B.res.set_pitch(v.pitchBits);
                    }
                }
            }

            if (!ready) return;

            auto blk = d.template as<ProcessData<2>>().toAudioBlock();
            float* L = blk.getChannelPointer(0);
            float* R = blk.getChannelPointer(1);
            const int N = d.getNumSamples();
            std::fill(L, L + N, 0.0f);

            for (int base = 0; base < N; base += SLICE)
            {
                const int len = jmin(SLICE, N - base);
                std::fill(mixBuf, mixBuf + len, 0.0f);

                for (auto& vp : voices)
                {
                    if (!vp.active) continue;
                    if (vp.pendFlag) switchFrame(vp);

                    if (paramGlideOn)
                    {
                        if (vp.glideSamplesRemaining > 0)
                        {
                            int adv = jmin(vp.glideSamplesRemaining, len);
                            vp.glideCurBits += vp.glideStepBitsPerSample * adv;
                            vp.glideSamplesRemaining -= adv;
                            if (vp.glideSamplesRemaining <= 0)
                            {
                                const double glideSemis = std::log2(paramGlideTarget) * 12.0;
                                vp.glideCurBits = glideSemis * SEMI2BITS;
                                vp.glideSamplesRemaining = 0;
                            }
                        }
                    }
                    else if (vp.glideCurBits != 0.0)
                    {
                        vp.glideCurBits = 0.0;
                        vp.glideSamplesRemaining = 0;
                    }

                    int offsetBits = int(std::lround(vp.glideCurBits));
                    Lane& cur = vp.toggle ? vp.B : vp.A;
                    cur.res.set_pitch(vp.pitchBits + offsetBits);
                    cur.res.set_playback_pos(wrap(vp.frameParam, cur.res.get_playback_pos()));
                    cur.res.interpolate_block(laneBuf, len);

                    if (vp.fading)
                    {
                        Lane& prev = vp.toggle ? vp.A : vp.B;
                        prev.res.set_pitch(vp.pitchBits + offsetBits);
                        prev.res.set_playback_pos(
                            wrap(vp.frameParam, prev.res.get_playback_pos()));
                        prev.res.interpolate_block(prevBuf, len);

                        float a = vp.fadeAlpha;
                        FloatVectorOperations::multiply(laneBuf, a, len);
                        FloatVectorOperations::addWithMultiply(
                            laneBuf, prevBuf, 1.0f - a, len);
                    }

                    FloatVectorOperations::add(mixBuf, laneBuf, len);

                    if (vp.fading)
                    {
                        vp.fadeAlpha += VoicePack::fadeDelta() * len;
                        if (vp.fadeAlpha >= 1.0f)
                        {
                            vp.fading = false;
                            (vp.toggle ? vp.A : vp.B).active = false;
                        }
                    }
                }

                FloatVectorOperations::add(L + base, mixBuf, len);
            }

            FloatVectorOperations::multiply(L, globalVolume, N);
            FloatVectorOperations::copy(R, L, N);
        }

        /* ===== parameters ===== */

        template <int P>
        void setParameter(double v)
        {
            if constexpr (P == 1) // Frame select
            {
                globalFrame = jlimit(0, MAX_FRAMES - 1, int(v));
                for (auto& vp : voices)
                    if (vp.active && globalFrame != vp.frameParam)
                    {
                        vp.pendFrame = globalFrame;
                        vp.pendFlag = true;
                    }
            }
            else if constexpr (P == 2) { globalVolume = float(v); }
            else if constexpr (P == 3)
            {
                paramSemi = v;
                for (auto& vp : voices)
                {
                    vp.semiOff = v;
                    if (vp.active) updatePitch(vp);
                }
            }
            else if constexpr (P == 4)
            {
                paramMult = v <= 0.0 ? 1.0 : v;
                for (auto& vp : voices)
                {
                    vp.multOff = paramMult;
                    if (vp.active) updatePitch(vp);
                }
            }
            else if constexpr (P == 5) // Glide On/off
            {
                paramGlideOn = (v >= 0.5);
                for (auto& vp : voices)
                {
                    if (!vp.active) continue;
                    vp.glideCurBits = 0.0;
                    if (paramGlideOn)
                    {
                        const double glideSemis = std::log2(paramGlideTarget) * 12.0;
                        const double targetBits = glideSemis * SEMI2BITS;
                        const double samples = paramGlideTime * sr;
                        if (samples > 0.0)
                        {
                            vp.glideStepBitsPerSample = targetBits / samples;
                            vp.glideSamplesRemaining = int(samples);
                        }
                        else
                        {
                            vp.glideCurBits = targetBits;
                            vp.glideStepBitsPerSample = 0.0;
                            vp.glideSamplesRemaining = 0;
                        }
                    }
                    else
                    {
                        vp.glideStepBitsPerSample = 0.0;
                        vp.glideSamplesRemaining = 0;
                    }
                }
            }
            else if constexpr (P == 6) // Glide Time
            {
                paramGlideTime = v;
                for (auto& vp : voices)
                {
                    if (!vp.active || !paramGlideOn) continue;
                    const double glideSemis = std::log2(paramGlideTarget) * 12.0;
                    const double targetBits = glideSemis * SEMI2BITS;
                    const double samples = paramGlideTime * sr;
                    if (samples > 0.0)
                    {
                        vp.glideStepBitsPerSample = (targetBits - vp.glideCurBits) / samples;
                        vp.glideSamplesRemaining = int(samples);
                    }
                    else
                    {
                        vp.glideCurBits = targetBits;
                        vp.glideStepBitsPerSample = 0.0;
                        vp.glideSamplesRemaining = 0;
                    }
                }
            }
            else if constexpr (P == 7) // Glide-Mult
            {
                paramGlideTarget = v <= 0.0 ? 1.0 : v;
                for (auto& vp : voices)
                {
                    if (!vp.active || !paramGlideOn) continue;
                    const double glideSemis = std::log2(paramGlideTarget) * 12.0;
                    const double targetBits = glideSemis * SEMI2BITS;
                    const double samples = paramGlideTime * sr;
                    if (samples > 0.0)
                    {
                        vp.glideStepBitsPerSample = (targetBits - vp.glideCurBits) / samples;
                        vp.glideSamplesRemaining = int(samples);
                    }
                    else
                    {
                        vp.glideCurBits = targetBits;
                        vp.glideStepBitsPerSample = 0.0;
                        vp.glideSamplesRemaining = 0;
                    }
                }
            }
        }

        void createParameters(ParameterDataList& ps)
        {
            { parameter::data p("Frame", { 0.0, MAX_FRAMES - 1.0, 1.0 });   p.setDefaultValue(0);   registerCallback<1>(p); ps.add(std::move(p)); }
            { parameter::data p("Volume", { 0.0, 1.0,             0.001 }); p.setDefaultValue(0.8); registerCallback<2>(p); ps.add(std::move(p)); }
            { parameter::data p("Semitone", { -72.0, 36.0,          0.1 });   p.setDefaultValue(-12); registerCallback<3>(p); ps.add(std::move(p)); }
            { parameter::data p("Pitch-Mult", { 0.25, 4.0,            0.001 }); p.setDefaultValue(1.0); registerCallback<4>(p); ps.add(std::move(p)); }
            { parameter::data p("Glide On", { 0.0, 1.0,             1.0 });   p.setDefaultValue(0.0); registerCallback<5>(p); ps.add(std::move(p)); }
            { parameter::data p("Glide Time", { 0.0, 5.0,             0.001 }); p.setDefaultValue(0.1); registerCallback<6>(p); ps.add(std::move(p)); }
            { parameter::data p("Glide-Mult", { 0.25, 4.0,            0.001 }); p.setDefaultValue(1.0); registerCallback<7>(p); ps.add(std::move(p)); }
        }

        SN_EMPTY_PROCESS_FRAME;

    private:
        float  globalVolume = 0.8f;
        int    globalFrame = 0;
        double paramSemi = 0.0;
        double paramMult = 1.0;

        bool   paramGlideOn = false;
        double paramGlideTime = 0.1;     // seconds
        double paramGlideTarget = 1.0;   // multiplier

        int    cycle = FRAME_SIZE;
        double sr = 0.0;
        double rootOffSemis = 0.0;

        std::array<gw5::Int64, MAX_FRAMES> frameStart;
        gw5::InterpPack  interp;

        bool  ready = false;
        bool  haveSpecs = false;
        PrepareSpecs lastSpecs;

        float laneBuf[SLICE]{};
        float prevBuf[SLICE]{};
        float mixBuf[SLICE]{};

        std::shared_ptr<const gw5::MipMapFlt> _activeMip;

        void initLane(Lane& l)
        {
            l.res.set_interp(interp);
            l.res.set_sample_sp(_activeMip);
            l.res.clear_buffers();
            l.active = false;
            l.frameIdx = -1;
        }

        void initVoice(VoicePack& vp)
        {
            initLane(vp.A); initLane(vp.B);
            vp.clear();
            int startF = globalFrame;
            vp.frameParam = vp.pendFrame = startF;
            vp.A.res.set_playback_pos(frameStart[startF] << 32);
            vp.A.frameIdx = startF;
            vp.semiOff = paramSemi;
            vp.multOff = paramMult;
        }

        static double centsToSemis(double c) noexcept { return c / 100.0; }

        void updatePitch(VoicePack& vp)
        {
            int vIdx = voices.getVoiceIndexForData(vp);
            double semMul = std::log2(vp.multOff) * 12.0;
            double sem = rootOffSemis + vp.semiOff + semMul
                + (vp.midi - 24) + centsToSemis(getVoiceDetune(vIdx));
            vp.pitchBits = int(std::lround(sem * SEMI2BITS));
            vp.A.res.set_pitch(vp.pitchBits);
            vp.B.res.set_pitch(vp.pitchBits);
        }

        gw5::Int64 wrap(int idx, gw5::Int64 p) const noexcept
        {
            gw5::Int64 ip = p >> 32;
            gw5::Int64 frac = p & 0xffffffff;
            gw5::Int64 st = frameStart[idx];
            return (((ip - st) & (cycle - 1)) + st) << 32 | frac;
        }

        void switchFrame(VoicePack& vp)
        {
            Lane& src = vp.toggle ? vp.B : vp.A;
            Lane& dst = vp.toggle ? vp.A : vp.B;

            gw5::Int64 p = src.res.get_playback_pos();
            gw5::Int64 ip = p >> 32;
            gw5::Int64 frac = p & 0xffffffff;
            gw5::Int64 rel = (ip - frameStart[vp.frameParam]) & (cycle - 1);

            initLane(dst);
            dst.res.set_playback_pos(((frameStart[vp.pendFrame] + rel) << 32) | frac);
            dst.res.set_pitch(vp.pitchBits);
            dst.frameIdx = vp.pendFrame;
            dst.active = true;

            vp.fading = true;
            vp.toggle = !vp.toggle;
            vp.fadeAlpha = 0.0f;
            vp.frameParam = vp.pendFrame;
            vp.pendFlag = false;
        }

        // 'NumFilters'
        // 'NumTables'
        // 'NumTables'
        // 'NumSliderPacks'
        // 'NumSliderPacks'
        // 'NumFilters'
        // 'NumDisplayBuffers'
        // 'NumDisplayBuffers'
    };
} // namespace project
