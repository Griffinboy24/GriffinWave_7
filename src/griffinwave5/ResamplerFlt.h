

/*****************************************************************************

This is the "ready to use" class for sample interpolation. How to proceed:

1. Instanciate an InterpPack. It is just a helper class for ResamplerFlt, and
   can be shared between many instances of ResamplerFlt.

2. Instanciate a MipMapFlt. It will contain your sample data.

3. Initialise MipMapFlt and fill it with the sample. Similarly to InterpPack,
   it can be shared between many instances of ResamplerFlt.

4. Connect the InterpPack instance to the ResamplerFlt

5. Connect a MipMapFlt instance to the ResamplerFlt

6. Set pitch. You cannot do it before this point.

7. Optionally specify a playback position.

8. Generate a block of interpolated data.

9. You can go back to either 5, 6, 7 or 8.

Instance of this class can be assimilated to a "voice", a single unit of
monophonic sound generation. You can change the sample each time it is
needed, making it handy for polyponic synthesiser implementation.

In any case, NEVER EVER let the playback position exceed the sample length.



*Tab=3***********************************************************************/



/*****************************************************************************
ResamplerFlt.h   (2-May-2025 crash-proof)

Adds a tiny smart-pointer guard so the current and fade-out MIP tables stay
alive for as long as any ResamplerFlt instance needs them.
Eight bytes per lane, zero runtime cost.
******************************************************************************/

#pragma once

#include <memory>
#include "BaseVoiceState.h"
#include "Downsampler2Flt.hpp"
#include "rspl.hpp"

namespace gw5
{

    class InterpPack;
    class MipMapFlt;

    class ResamplerFlt
    {
        /* ===== PUBLIC =========================================================== */
    public:
        enum { MIP_MAP_FIR_LEN = 81 };
        enum { NBR_BITS_PER_OCT = BaseVoiceState::NBR_BITS_PER_OCT };

        ResamplerFlt();
        virtual ~ResamplerFlt() = default;

        /* --- original API (unchanged) --------------------------------------- */
        void set_interp(const InterpPack& interp);
        void set_sample(const MipMapFlt& spl);   // << still works
        void remove_sample();

        void set_pitch(long pitch);
        long get_pitch() const;

        void set_playback_pos(Int64 pos);
        Int64 get_playback_pos() const;

        void interpolate_block(float dest_ptr[], long nbr_spl);
        void clear_buffers();

        static const double _fir_mip_map_coef_arr[MIP_MAP_FIR_LEN];

        /* --- new helper: pass a shared_ptr ---------------------------------- */
        inline void set_sample_sp(const std::shared_ptr<const MipMapFlt>& sp) noexcept
        {
            _mip_guard = sp;          // keeps table alive
            set_sample(*sp);          // reuse existing logic (fades, etc.)
        }

        /* ===== PRIVATE ========================================================== */
    private:
        enum VoiceInfo { VoiceInfo_CURRENT = 0, VoiceInfo_FADEOUT, VoiceInfo_NBR_ELT };

        typedef std::vector<float> SplData;

        void reset_pitch_cur_voice();
        void fade_block(float dest_ptr[], long nbr_spl);
        inline int compute_table(long pitch);
        void begin_mip_map_fading();

        SplData            _buf;
        const MipMapFlt* _mip_map_ptr = nullptr;               // fast raw access
        std::shared_ptr<const MipMapFlt> _mip_guard;             // lifetime guard
        const InterpPack* _interp_ptr = nullptr;
        Downsampler2Flt    _dwnspl;
        BaseVoiceState     _voice_arr[VoiceInfo_NBR_ELT];
        long               _pitch = 0;
        long               _buf_len = 0;
        long               _fade_pos = 0;
        bool               _fade_flag = false;
        bool               _fade_needed_flag = false;
        bool               _can_use_flag = false;

        static const double _dwnspl_coef_arr[Downsampler2Flt::NBR_COEFS];

        /* no copy */
        ResamplerFlt(const ResamplerFlt&) = delete;
        ResamplerFlt& operator=(const ResamplerFlt&) = delete;
    };

} // namespace gw5
