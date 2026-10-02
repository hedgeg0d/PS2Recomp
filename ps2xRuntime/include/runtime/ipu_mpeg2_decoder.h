// SPDX-FileCopyrightText: 2026 PS2Recomp contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <cstdint>
#include <functional>

// MPEG-1/2 macroblock decoder used by the PS2 IPU HLE.  This deliberately
// sits below sceMpeg: games may drive IPU_CMD and IPU DMA directly.
class IpuMpeg2Decoder
{
public:
    static constexpr size_t kMacroblockSamples = 256u + 64u + 64u;
    using PeekBits = std::function<bool(uint32_t, uint32_t &)>;
    using AdvanceBits = std::function<bool(uint32_t)>;

    struct BdecConfig
    {
        uint8_t qScaleCode = 0;
        uint8_t intraDcPrecision = 0;
        bool macroblockIntra = false;
        bool dcReset = false;
        bool interlacedDct = false;
        bool alternateScan = false;
        bool intraVlcFormat = false;
        bool nonlinearQScale = false;
        bool mpeg1 = false;
    };

    IpuMpeg2Decoder();

    void reset();
    void setQuantMatrix(bool intra, const std::array<uint8_t, 64> &matrix);

    // Produces the IPU BDEC layout: 16-bit Y[16x16], Cb[8x8], Cr[8x8].
    // Non-intra output is signed residual data stored in the same 16-bit words.
    bool decodeMacroblock(const BdecConfig &config,
                          const PeekBits &peek,
                          const AdvanceBits &advance,
                          std::array<uint16_t, kMacroblockSamples> &output,
                          uint8_t &codedBlockPattern);

private:
    std::array<uint8_t, 64> m_intraMatrix{};
    std::array<uint8_t, 64> m_nonIntraMatrix{};
    std::array<int16_t, 3> m_dcPredictor{};
};
