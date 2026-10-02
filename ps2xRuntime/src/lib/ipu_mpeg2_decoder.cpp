// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-FileCopyrightText: 2000-2002 Michel Lespinasse
// SPDX-FileCopyrightText: 1999-2000 Aaron Holtzman
// SPDX-FileCopyrightText: 2026 PS2Recomp contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The VLC and IDCT logic follows PCSX2's IPU decoder, itself based on
// mpeg2dec.  Kept behind a small callback interface so direct-IPU games and
// future HLE users share one hardware-shaped MPEG macroblock decoder.

#include "runtime/ipu_mpeg2_decoder.h"

#include "ipu_mpeg2_vlc.h"

#include <algorithm>
#include <array>
#include <cstring>

namespace
{
constexpr int kNonlinearQScale[32] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 18, 20, 22,
    24, 28, 32, 36, 40, 44, 48, 52, 56, 64, 72, 80, 88, 96, 104, 112};

constexpr std::array<uint8_t, 64> makeScan(const std::array<uint8_t, 64> &scan)
{
    std::array<uint8_t, 64> packed{};
    for (size_t i = 0; i < scan.size(); ++i)
    {
        const uint8_t j = scan[i];
        packed[i] = static_cast<uint8_t>(((j & 0x36u) >> 1u) | ((j & 0x09u) << 2u));
    }
    return packed;
}

constexpr std::array<uint8_t, 64> kNormalScan = makeScan({
    0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63});

constexpr std::array<uint8_t, 64> kAlternateScan = makeScan({
    0, 8, 16, 24, 1, 9, 2, 10, 17, 25, 32, 40, 48, 56, 57, 49,
    41, 33, 26, 18, 3, 11, 4, 12, 19, 27, 34, 42, 50, 58, 35, 43,
    51, 59, 20, 28, 5, 13, 6, 14, 21, 29, 36, 44, 52, 60, 37, 45,
    53, 61, 22, 30, 7, 15, 23, 31, 38, 46, 54, 62, 39, 47, 55, 63});

constexpr int kW1 = 2841;
constexpr int kW2 = 2676;
constexpr int kW3 = 2408;
constexpr int kW5 = 1609;
constexpr int kW6 = 1108;
constexpr int kW7 = 565;

void butterfly(int &t0, int &t1, int w0, int w1, int d0, int d1)
{
    const int temp = w0 * (d0 + d1);
    t0 = temp + (w1 - w0) * d1;
    t1 = temp - (w1 + w0) * d0;
}

void idctBlock(int16_t *block)
{
    for (int i = 0; i < 8; ++i)
    {
        int16_t *row = block + 8 * i;
        if (!(row[1] | row[2] | row[3] | row[4] | row[5] | row[6] | row[7]))
        {
            std::fill_n(row, 8, static_cast<int16_t>(row[0] << 3));
            continue;
        }

        const int d0 = (row[0] << 11) + 128;
        const int d1 = row[1];
        const int d2 = row[2] << 11;
        const int d3 = row[3];
        const int t0 = d0 + d2;
        const int t1 = d0 - d2;
        int t2, t3;
        butterfly(t2, t3, kW6, kW2, d3, d1);
        const int a0 = t0 + t2;
        const int a1 = t1 + t3;
        const int a2 = t1 - t3;
        const int a3 = t0 - t2;

        int u0, u1, u2, u3;
        butterfly(u0, u1, kW7, kW1, row[7], row[4]);
        butterfly(u2, u3, kW3, kW5, row[5], row[6]);
        const int b0 = u0 + u2;
        const int b3 = u1 + u3;
        u0 -= u2;
        u1 -= u3;
        const int b1 = ((u0 + u1) * 181) >> 8;
        const int b2 = ((u0 - u1) * 181) >> 8;

        row[0] = static_cast<int16_t>((a0 + b0) >> 8);
        row[1] = static_cast<int16_t>((a1 + b1) >> 8);
        row[2] = static_cast<int16_t>((a2 + b2) >> 8);
        row[3] = static_cast<int16_t>((a3 + b3) >> 8);
        row[4] = static_cast<int16_t>((a3 - b3) >> 8);
        row[5] = static_cast<int16_t>((a2 - b2) >> 8);
        row[6] = static_cast<int16_t>((a1 - b1) >> 8);
        row[7] = static_cast<int16_t>((a0 - b0) >> 8);
    }

    for (int i = 0; i < 8; ++i)
    {
        int16_t *column = block + i;
        const int d0 = (column[0] << 11) + 65536;
        const int d1 = column[8];
        const int d2 = column[16] << 11;
        const int d3 = column[24];
        const int t0 = d0 + d2;
        const int t1 = d0 - d2;
        int t2, t3;
        butterfly(t2, t3, kW6, kW2, d3, d1);
        const int a0 = t0 + t2;
        const int a1 = t1 + t3;
        const int a2 = t1 - t3;
        const int a3 = t0 - t2;

        int u0, u1, u2, u3;
        butterfly(u0, u1, kW7, kW1, column[56], column[32]);
        butterfly(u2, u3, kW3, kW5, column[40], column[48]);
        const int b0 = u0 + u2;
        const int b3 = u1 + u3;
        u0 = (u0 - u2) >> 8;
        u1 = (u1 - u3) >> 8;
        const int b1 = (u0 + u1) * 181;
        const int b2 = (u0 - u1) * 181;

        column[0] = static_cast<int16_t>((a0 + b0) >> 17);
        column[8] = static_cast<int16_t>((a1 + b1) >> 17);
        column[16] = static_cast<int16_t>((a2 + b2) >> 17);
        column[24] = static_cast<int16_t>((a3 + b3) >> 17);
        column[32] = static_cast<int16_t>((a3 - b3) >> 17);
        column[40] = static_cast<int16_t>((a2 - b2) >> 17);
        column[48] = static_cast<int16_t>((a1 - b1) >> 17);
        column[56] = static_cast<int16_t>((a0 - b0) >> 17);
    }
}

class MacroblockReader
{
public:
    MacroblockReader(const IpuMpeg2Decoder::BdecConfig &config,
                     const IpuMpeg2Decoder::PeekBits &peek,
                     const IpuMpeg2Decoder::AdvanceBits &advance,
                     const std::array<uint8_t, 64> &intraMatrix,
                     const std::array<uint8_t, 64> &nonIntraMatrix,
                     std::array<int16_t, 3> &dcPredictor)
        : m_config(config), m_peek(peek), m_advance(advance),
          m_intraMatrix(intraMatrix), m_nonIntraMatrix(nonIntraMatrix),
          m_dcPredictor(dcPredictor)
    {
    }

    bool decode(std::array<uint16_t, IpuMpeg2Decoder::kMacroblockSamples> &output,
                uint8_t &codedBlockPattern)
    {
        output.fill(0u);
        if (m_config.dcReset)
        {
            const int16_t predictor = static_cast<int16_t>(128 << m_config.intraDcPrecision);
            m_dcPredictor.fill(predictor);
        }

        m_quantizerScale = m_config.nonlinearQScale
                               ? kNonlinearQScale[m_config.qScaleCode & 31u]
                               : ((m_config.qScaleCode & 31u) << 1u);
        const int dctOffset = m_config.interlacedDct ? 16 : 128;
        const int dctStride = m_config.interlacedDct ? 32 : 16;

        if (m_config.macroblockIntra)
        {
            std::array<uint8_t, IpuMpeg2Decoder::kMacroblockSamples> pixels{};
            if (!decodeIntraBlock(0, pixels.data(), dctStride) ||
                !decodeIntraBlock(0, pixels.data() + 8, dctStride) ||
                !decodeIntraBlock(0, pixels.data() + dctOffset, dctStride) ||
                !decodeIntraBlock(0, pixels.data() + dctOffset + 8, dctStride) ||
                !decodeIntraBlock(1, pixels.data() + 256, 8) ||
                !decodeIntraBlock(2, pixels.data() + 320, 8))
            {
                return false;
            }
            for (size_t i = 0; i < output.size(); ++i)
                output[i] = pixels[i];
            codedBlockPattern = 0x3fu;
            return true;
        }

        uint32_t code = 0u;
        if (!m_peek(16u, code))
            return false;
        const CBPtab *cbp = nullptr;
        if (code >= 0x2000u)
            cbp = &CBP_7[(code >> 9u) - 16u];
        else
            cbp = &CBP_9[code >> 7u];
        if (cbp->len == 0u || !m_advance(cbp->len))
            return false;
        codedBlockPattern = cbp->cbp;

        std::array<int16_t, IpuMpeg2Decoder::kMacroblockSamples> residual{};
        if (((cbp->cbp & 0x20u) && !decodeNonIntraBlock(residual.data(), dctStride)) ||
            ((cbp->cbp & 0x10u) && !decodeNonIntraBlock(residual.data() + 8, dctStride)) ||
            ((cbp->cbp & 0x08u) && !decodeNonIntraBlock(residual.data() + dctOffset, dctStride)) ||
            ((cbp->cbp & 0x04u) && !decodeNonIntraBlock(residual.data() + dctOffset + 8, dctStride)) ||
            ((cbp->cbp & 0x02u) && !decodeNonIntraBlock(residual.data() + 256, 8)) ||
            ((cbp->cbp & 0x01u) && !decodeNonIntraBlock(residual.data() + 320, 8)))
        {
            return false;
        }
        for (size_t i = 0; i < output.size(); ++i)
            output[i] = static_cast<uint16_t>(residual[i]);
        return true;
    }

private:
    bool get(uint32_t bits, uint32_t &value)
    {
        return m_peek(bits, value) && m_advance(bits);
    }

    bool getSigned(uint32_t bits, int32_t &value)
    {
        uint32_t raw = 0u;
        if (!get(bits, raw))
            return false;
        const uint32_t sign = 1u << (bits - 1u);
        value = static_cast<int32_t>((raw ^ sign) - sign);
        return true;
    }

    bool decodeDcDifference(bool luma, int &difference)
    {
        uint32_t code = 0u;
        if (!m_peek(luma ? 9u : 10u, code))
            return false;
        const DCtab *entry = nullptr;
        if ((code >> ((luma ? 9u : 10u) - 5u)) < 31u)
        {
            const uint32_t prefix = code >> ((luma ? 9u : 10u) - 5u);
            entry = luma ? &DCtable.lum0[prefix] : &DCtable.chrom0[prefix];
        }
        else
        {
            entry = luma ? &DCtable.lum1[code - 0x1f0u]
                         : &DCtable.chrom1[code - 0x3e0u];
        }
        if (entry->len == 0u || !m_advance(entry->len))
            return false;
        if (entry->size == 0u)
        {
            difference = 0;
            return true;
        }
        uint32_t magnitude = 0u;
        if (!get(entry->size, magnitude))
            return false;
        difference = static_cast<int>(magnitude);
        if ((magnitude & (1u << (entry->size - 1u))) == 0u)
            difference -= (1 << entry->size) - 1;
        return true;
    }

    const DCTtab *lookupDct(uint16_t code, bool first, bool intra) const
    {
        if (code >= 16384u && (!intra || !m_config.intraVlcFormat || m_config.mpeg1))
            return first ? &DCT.first[(code >> 12u) - 4u] : &DCT.next[(code >> 12u) - 4u];
        if (code >= 1024u)
            return (intra && m_config.intraVlcFormat && !m_config.mpeg1)
                       ? &DCT.tab0a[(code >> 8u) - 4u]
                       : &DCT.tab0[(code >> 8u) - 4u];
        if (code >= 512u)
            return (intra && m_config.intraVlcFormat && !m_config.mpeg1)
                       ? &DCT.tab1a[(code >> 6u) - 8u]
                       : &DCT.tab1[(code >> 6u) - 8u];
        if (code >= 256u) return &DCT.tab2[(code >> 4u) - 16u];
        if (code >= 128u) return &DCT.tab3[(code >> 3u) - 16u];
        if (code >= 64u) return &DCT.tab4[(code >> 2u) - 16u];
        if (code >= 32u) return &DCT.tab5[(code >> 1u) - 16u];
        if (code >= 16u) return &DCT.tab6[code - 16u];
        return nullptr;
    }

    bool decodeCoefficients(bool intra, int &last)
    {
        const auto &scan = m_config.alternateScan ? kAlternateScan : kNormalScan;
        const auto &matrix = intra ? m_intraMatrix : m_nonIntraMatrix;
        int index = intra ? 1 : 0;
        // Even when coefficient 63 is populated, consume the following EOB.
        // Leaving it in the stream shifts the next block's DC/VLC parsing.
        while (true)
        {
            uint32_t raw = 0u;
            if (!m_peek(16u, raw))
                return false;
            const DCTtab *entry = lookupDct(static_cast<uint16_t>(raw), !intra && index == 0, intra);
            if (!entry)
            {
                last = index;
                return true;
            }
            if (!m_advance(entry->len))
                return false;
            if (entry->run == 64u)
            {
                last = index;
                return true;
            }

            uint32_t run = entry->run;
            if (run == 65u && !get(6u, run))
                return false;
            index += static_cast<int>(run);
            if (index >= 64)
            {
                last = index;
                return true;
            }

            int value = 0;
            if (entry->run == 65u)
            {
                if (!m_config.mpeg1)
                {
                    int32_t level = 0;
                    if (!getSigned(12u, level))
                        return false;
                    if (intra)
                        value = (level * m_quantizerScale * matrix[index]) >> 4;
                    else
                        value = ((2 * level + (level >= 0 ? 1 : -1)) * m_quantizerScale * matrix[index]) >> 5;
                }
                else
                {
                    int32_t level = 0;
                    if (!getSigned(8u, level))
                        return false;
                    if ((level & 0x7f) == 0)
                    {
                        uint32_t extension = 0u;
                        if (!get(8u, extension))
                            return false;
                        level = static_cast<int32_t>(extension) + 2 * level;
                    }
                    if (intra)
                        value = (level * m_quantizerScale * matrix[index]) >> 4;
                    else
                        value = ((2 * (level + (level >> 31)) + 1) * m_quantizerScale * matrix[index]) / 32;
                    value = (value + ~(value >> 31)) | 1;
                }
            }
            else
            {
                uint32_t sign = 0u;
                if (!get(1u, sign))
                    return false;
                if (intra)
                {
                    value = (entry->level * m_quantizerScale * matrix[index]) >> 4;
                    if (m_config.mpeg1)
                        value = (value - 1) | 1;
                }
                else
                {
                    value = ((2 * entry->level + 1) * m_quantizerScale * matrix[index]) >> 5;
                }
                if (sign)
                    value = -value;
            }

            value = std::clamp(value, -2048, 2047);
            m_dctBlock[scan[index]] = static_cast<int16_t>(value);
            ++index;
        }
        last = index;
        return true;
    }

    bool decodeIntraBlock(int component, uint8_t *destination, int stride)
    {
        m_dctBlock.fill(0);
        int difference = 0;
        if (!decodeDcDifference(component == 0, difference))
            return false;
        m_dcPredictor[component] = static_cast<int16_t>(m_dcPredictor[component] + difference);
        m_dctBlock[0] = static_cast<int16_t>(m_dcPredictor[component] << (3 - m_config.intraDcPrecision));
        int last = 0;
        if (!decodeCoefficients(true, last))
            return false;
        idctBlock(m_dctBlock.data());
        for (int y = 0; y < 8; ++y)
        {
            for (int x = 0; x < 8; ++x)
                destination[x] = static_cast<uint8_t>(std::clamp<int>(m_dctBlock[y * 8 + x], 0, 255));
            destination += stride;
        }
        return true;
    }

    bool decodeNonIntraBlock(int16_t *destination, int stride)
    {
        m_dctBlock.fill(0);
        int last = 0;
        if (!decodeCoefficients(false, last))
            return false;
        idctBlock(m_dctBlock.data());
        for (int y = 0; y < 8; ++y)
        {
            std::copy_n(m_dctBlock.data() + y * 8, 8, destination);
            destination += stride;
        }
        return true;
    }

    const IpuMpeg2Decoder::BdecConfig &m_config;
    const IpuMpeg2Decoder::PeekBits &m_peek;
    const IpuMpeg2Decoder::AdvanceBits &m_advance;
    const std::array<uint8_t, 64> &m_intraMatrix;
    const std::array<uint8_t, 64> &m_nonIntraMatrix;
    std::array<int16_t, 3> &m_dcPredictor;
    std::array<int16_t, 64> m_dctBlock{};
    int m_quantizerScale = 0;
};
} // namespace

IpuMpeg2Decoder::IpuMpeg2Decoder()
{
    reset();
}

void IpuMpeg2Decoder::reset()
{
    m_intraMatrix.fill(16u);
    m_nonIntraMatrix.fill(16u);
    m_dcPredictor.fill(128);
}

void IpuMpeg2Decoder::setQuantMatrix(bool intra, const std::array<uint8_t, 64> &matrix)
{
    (intra ? m_intraMatrix : m_nonIntraMatrix) = matrix;
}

bool IpuMpeg2Decoder::decodeMacroblock(const BdecConfig &config,
                                       const PeekBits &peek,
                                       const AdvanceBits &advance,
                                       std::array<uint16_t, kMacroblockSamples> &output,
                                       uint8_t &codedBlockPattern)
{
    MacroblockReader reader(config, peek, advance, m_intraMatrix, m_nonIntraMatrix, m_dcPredictor);
    return reader.decode(output, codedBlockPattern);
}
